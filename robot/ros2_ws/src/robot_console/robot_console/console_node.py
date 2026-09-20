"""Serve the operator console against a running robot.

AUTHORITY: this console may ask for a trip and it may engage or release the
software emergency stop. That is the whole of it.

It never publishes a velocity and never touches /cmd_vel. A destination it
accepts becomes a Nav2 NavigateToPose goal, and every Nav2 node that can emit
a velocity has cmd_vel remapped to cmd_vel_requested, so the safety gate still
decides what reaches the wheels:

    console -> Nav2 goal -> Nav2 -> /cmd_vel_requested -> robot_safety -> /cmd_vel

The stop it works is the software one, which is secondary. The physical
emergency stop is wired independently of this console, of this node and of the
computer it runs on; nothing here can engage it and nothing here can release
it. The software latch is released only by `true` on `emergency_stop_reset`,
which takes a deliberate operator press on the page.

The console reports the gate's state and never its own opinion of it. When it
cannot see the gate it says so, refuses to send goals, and shows the latch as
unknown rather than as released.
"""

from http.server import ThreadingHTTPServer
import math
from threading import Thread

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_core import NavigationGoal, Pose2D
from robot_locations import LocationStore
from std_msgs.msg import Bool, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from robot_console.goal_dispatcher import RosGoalDispatcher
from robot_console.map_provider import (
    RosMapProvider,
    pose_from_transform,
    snapshot_from_message,
)
from robot_console.safety_adapter import RosSafetyAdapter
from robot_console.web_console import WEB_ROOT, RobotWebApp, make_handler

# Matches robot_safety.safety_node.STATE_QOS. Imported rather than restated
# would couple this package's import to rclpy being present in the gate's
# package; the shape is two lines and the gate's comment explains the why.
STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# map_server latches the map: one message, kept for whoever subscribes later.
# A volatile subscription to it simply never receives anything.
MAP_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _optional(value: str) -> str | None:
    """Turn the empty-string sentinel used by the parameters into "unset"."""
    text = value.strip()
    return text or None


class ConsoleNode(Node):
    """Wire the console's three boundaries to the graph, and serve the page."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs carries parameter_overrides, which is how the tests
        # point the node at a temporary location store and a free port.
        super().__init__("operator_console", **node_kwargs)

        self.declare_parameter("locations_file", "")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # How long the gate may go quiet before the console stops believing
        # what it last said. The gate runs its control loop at control_rate_hz
        # (20 Hz by default), so this is generous by two orders of magnitude.
        self.declare_parameter("safety_timeout", 2.0)
        # A transform older than this is a robot that has stopped saying where
        # it is, which is reported as an unknown position, not an old one.
        self.declare_parameter("transform_timeout", 2.0)
        self.declare_parameter("map_max_pixels", 640_000)

        locations_file = _optional(str(self.get_parameter("locations_file").value))
        if locations_file is None:
            # Started without the allow-list the console would offer nowhere
            # to go, which looks like an empty robot rather than the missing
            # parameter it is.
            raise ValueError("locations_file must name the approved destination store")

        self._goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._transform_timeout = float(self.get_parameter("transform_timeout").value)
        safety_timeout = float(self.get_parameter("safety_timeout").value)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        store = LocationStore(locations_file)
        if not store.path.exists():
            self.get_logger().warning(
                f"no approved destinations in {locations_file}: nowhere to go yet"
            )

        # Reentrant so a goal sent from an HTTP thread and the action's own
        # answers are not waiting on each other.
        callbacks = ReentrantCallbackGroup()

        self._navigator = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=callbacks
        )
        self.dispatcher = RosGoalDispatcher(
            self._navigator, self._navigate_to_pose, self.get_logger()
        )

        self.safety = RosSafetyAdapter(
            stop_publisher=self.create_publisher(Bool, "emergency_stop", 10),
            reset_publisher=self.create_publisher(Bool, "emergency_stop_reset", 10),
            bool_message=lambda value: Bool(data=value),
            clock=self._now,
            timeout=safety_timeout,
            logger=self.get_logger(),
        )
        # The gate keeps its last state for late joiners, so a console opened
        # onto a quiet robot learns what the gate is doing the moment it
        # subscribes rather than waiting for the next change. Asking for
        # volatile here would throw that kept line away.
        self.create_subscription(
            String,
            "safety_state",
            self._on_safety_state,
            STATE_QOS,
            callback_group=callbacks,
        )
        # Subscribed to, never published to: the gate's velocity output runs
        # every control cycle and is the only thing that says the gate is
        # still deciding. See robot_console.safety_adapter.
        self.create_subscription(
            Twist, "cmd_vel", self._on_gate_heartbeat, 10, callback_group=callbacks
        )

        self.map_provider = RosMapProvider(
            self._robot_pose, int(self.get_parameter("map_max_pixels").value)
        )
        self.create_subscription(
            OccupancyGrid, "map", self._on_map, MAP_QOS, callback_group=callbacks
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.app = RobotWebApp(
            store,
            self.dispatcher,
            safety=self.safety,
            map_provider=self.map_provider,
        )
        self.server = ThreadingHTTPServer(
            (host, port), make_handler(self.app, WEB_ROOT)
        )
        self._server_thread = Thread(
            target=self.server.serve_forever, name="operator-console", daemon=True
        )
        self._server_thread.start()
        self.get_logger().info(
            f"operator console on http://{host}:{self.server.server_address[1]}"
        )

    def _on_safety_state(self, message: String) -> None:
        self.safety.on_state(message.data)

    def _on_gate_heartbeat(self, _message: Twist) -> None:
        self.safety.on_heartbeat()

    def _on_map(self, message: OccupancyGrid) -> None:
        snapshot = snapshot_from_message(message)
        if snapshot is None:
            # Kept, not replaced with a nonsense one: the last usable map is
            # better than a grid whose origin puts the robot nowhere.
            self.get_logger().error("ignored a map with an unusable origin or size")
            return
        self.map_provider.on_grid(snapshot)

    def _robot_pose(self):
        """Where the robot is, according to the transform tree, or nothing.

        Asked on demand rather than cached, so a page redraw cannot show a
        pose the transform tree has since stopped standing behind.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None

        stamp = transform.header.stamp
        age = self._now() - (stamp.sec + stamp.nanosec * 1e-9)
        if not math.isfinite(age) or abs(age) > self._transform_timeout:
            # Either stale, or stamped in the future by a clock that jumped.
            # Neither is a position worth drawing.
            return None
        return pose_from_transform(transform)

    def _now(self) -> float:
        """Seconds from the node clock, so use_sim_time governs the timeouts."""
        return self.get_clock().now().nanoseconds * 1e-9

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
        # Yaw only: the base is a differential drive, so a goal roll or pitch
        # would be a number nothing can honour.
        stamped.pose.orientation.z = math.sin(pose.yaw / 2.0)
        stamped.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return stamped

    def destroy_node(self) -> bool:
        self.server.shutdown()
        self.server.server_close()
        self._server_thread.join(timeout=5.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConsoleNode()
    # Multi-threaded because the page's requests arrive on their own threads
    # and must not be able to wedge the callbacks that keep the safety state
    # fresh; a console whose status went stale would start refusing goals.
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
