"""ROS 2 node that turns recognized speech into approved navigation goals.

SAFETY: this node never publishes a velocity and never touches /cmd_vel. A
destination it accepts becomes a Nav2 NavigateToPose goal, and every Nav2
node that can emit a velocity already has cmd_vel remapped to
cmd_vel_requested, so the safety gate still decides what reaches the wheels:

    speech -> Nav2 goal -> Nav2 -> /cmd_vel_requested -> robot_safety -> /cmd_vel

Only destinations the operator has already approved in the location store
become goals; CommandGateway refuses everything else out loud. A heard stop
engages the latched software emergency stop and cancels the trip. Releasing
that latch takes an operator on `emergency_stop_reset`, which this node does
not publish: a voice that can undo a stop is not a stop.
"""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_core import NavigationGoal, Pose2D
from robot_locations import LocationStore
from std_msgs.msg import Bool, String

from robot_voice.command_gateway import CommandGateway


def _optional(value: str) -> str | None:
    """Turn the empty-string sentinel used by the parameters into "unset"."""
    text = value.strip()
    return text or None


class VoiceNode(Node):
    """Answer every heard command, and drive only by asking Nav2 to."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs carries parameter_overrides, which is how the tests
        # point the node at a temporary location store.
        super().__init__("voice_commander", **node_kwargs)

        # ROS parameters have no null, so an unset string is the empty one.
        self.declare_parameter("locations_file", "")
        self.declare_parameter("home_location", "")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("stop_engages_emergency_stop", True)
        self.declare_parameter("location_report_radius", 1.5)

        locations_file = _optional(str(self.get_parameter("locations_file").value))
        if locations_file is None:
            # Started without the allow-list, the node would refuse every
            # trip, which sounds like a broken microphone rather than like
            # the missing parameter it is.
            raise ValueError("locations_file must name the approved destination store")

        home_location = _optional(str(self.get_parameter("home_location").value))
        self._goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self._stop_engages_emergency_stop = bool(
            self.get_parameter("stop_engages_emergency_stop").value
        )
        self._location_report_radius = float(
            self.get_parameter("location_report_radius").value
        )

        self._locations = LocationStore(locations_file)
        self._locations_modified = None
        if not self._locations.path.exists():
            # Said out loud at start-up, because from the outside a mistyped
            # path and a robot that refuses every trip look the same.
            self.get_logger().warning(
                f"no approved destinations in {locations_file}: nowhere to go yet"
            )
        # Raises when home_location is not an approved destination: a home
        # nobody approved is a configuration error, not a runtime refusal.
        self._gateway = CommandGateway(self._locations, home_location=home_location)

        self._position = None
        self._goal_handle = None
        # A stop can arrive while Nav2 is still accepting a goal, so this
        # flag, not the handle, decides whether the trip survives.
        self._trip_wanted = False

        self._response_publisher = self.create_publisher(String, "voice_response", 10)
        self._emergency_stop_publisher = self.create_publisher(
            Bool, "emergency_stop", 10
        )
        self._navigator = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(String, "speech_transcript", self._on_transcript, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_position, 10
        )

    def _on_transcript(self, message: String) -> None:
        self._refresh_locations()
        outcome = self._gateway.handle(message.data)
        response = outcome.response

        if outcome.action == "stop":
            self._stop()
        elif outcome.action == "halt":
            self._halt()
        elif outcome.action == "go_to":
            response = self._start_trip(outcome.goal, response)
        elif outcome.action == "report_location":
            response = self._location_report()
        # Anything else, "refused" included, is answered and nothing more.
        # Dispatching on the named actions rather than on whether a goal came
        # back keeps a later gateway action from driving by default.

        self._respond(response)

    def _start_trip(self, goal: NavigationGoal | None, response: str) -> str:
        if goal is None:
            # The gateway only returns go_to with an approved goal attached.
            # Driving without one is the thing it exists to prevent.
            self.get_logger().error("approved destination arrived without a goal")
            return "I cannot go there."

        if not self._navigator.server_is_ready():
            # Never block the callback waiting for Nav2. A goal sent to an
            # absent server is dropped silently, which sounds like success.
            self.get_logger().warning("navigation is not running: goal not sent")
            return "I cannot go anywhere yet. Navigation is not running."

        self._trip_wanted = True
        future = self._navigator.send_goal_async(self._navigate_to_pose(goal))
        future.add_done_callback(self._on_goal_response)
        return response

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._trip_wanted = False
            self.get_logger().warning("navigation refused the goal")
            self._respond("Navigation would not take that trip.")
            return

        if not self._trip_wanted:
            # A stop arrived while the goal was still being accepted.
            goal_handle.cancel_goal_async()
            return

        self._goal_handle = goal_handle

    def _stop(self) -> None:
        """Latch the gate first, then let go of the trip.

        Cancelling a Nav2 goal unwinds through the behaviour tree and the
        controller; the latch stops the wheels at the gate on its next
        control cycle. Only `emergency_stop_reset` releases it again.
        """
        if self._stop_engages_emergency_stop:
            self._emergency_stop_publisher.publish(Bool(data=True))

        self._abandon_trip()

    def _halt(self) -> None:
        """A stop the recogniser mangled: drop the trip, leave the latch alone.

        "sto" and "hal" are what a real engine makes of someone shouting stop,
        and a refusal would leave a moving robot moving. Cancelling the trip
        stops it without the latch, which takes an operator to release: a
        robot stranded by a misheard syllable cannot fetch help either.
        """
        self._abandon_trip()

    def _abandon_trip(self) -> None:
        self._trip_wanted = False
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _refresh_locations(self) -> None:
        """Pick up destinations the operator approved after start-up.

        The store reads its file once when constructed, so without this the
        node would keep answering with the allow-list it booted with.
        """
        try:
            modified = self._locations.path.stat().st_mtime_ns
        except OSError:
            return  # No file to read: keep whatever was approved before.

        if modified == self._locations_modified:
            return

        try:
            self._locations.load()
        except (OSError, ValueError, KeyError, TypeError) as error:
            # A half-written or malformed file leaves the destinations that
            # were already approved in place, rather than taking the node
            # down or widening the list.
            self.get_logger().error(f"kept the approved destinations: {error}")
            return

        self._locations_modified = modified

    def _on_position(self, message: PoseWithCovarianceStamped) -> None:
        """Track where the robot thinks it is, for spoken reports only.

        How much to trust this pose is the safety gate's decision, not this
        node's: nothing here commands motion from it.
        """
        position = message.pose.pose.position
        self._position = (position.x, position.y)

    def _location_report(self) -> str:
        if self._position is None:
            return "I do not know where I am yet."

        nearest = self._nearest_location()
        if nearest is None:
            return "I am not near any of the places I know."
        return f"I am near the {nearest}."

    def _nearest_location(self) -> str | None:
        candidates = [
            (math.dist((goal.pose.x, goal.pose.y), self._position), goal.location_name)
            for goal in (
                self._locations.get_goal(name) for name in self._locations.names()
            )
        ]
        if not candidates:
            return None

        distance, location_name = min(candidates)
        return location_name if distance <= self._location_report_radius else None

    def _navigate_to_pose(self, goal: NavigationGoal) -> NavigateToPose.Goal:
        request = NavigateToPose.Goal()
        request.pose = self._pose_stamped(goal.pose)
        return request

    def _pose_stamped(self, pose: Pose2D) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = self._goal_frame_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        # Yaw only: the base is a differential drive, so a goal roll or
        # pitch would be a number nothing can honour.
        stamped.pose.orientation.z = math.sin(pose.yaw / 2.0)
        stamped.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return stamped

    def _respond(self, response: str) -> None:
        message = String()
        message.data = response
        self._response_publisher.publish(message)
        self.get_logger().info(response)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
