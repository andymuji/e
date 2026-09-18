"""ROS 2 node that saves named places and recalls them as Nav2 goals.

SAFETY: this node cannot drive the robot and must never be able to. A recall
publishes a PoseStamped goal and stops there; deciding how to reach it, and
whether to move at all, stays with Nav2 and the safety gate:

    recall_location -> /goal_pose -> Nav2 -> /cmd_vel_requested
                                            -> robot_safety -> /cmd_vel

There is no Twist publisher anywhere in this module, and
LocationNodeMotionTests asserts that stays true.

Saving records where the robot *is*, never a pose supplied by the caller, so
a named place is always somewhere the robot has actually been driven to and
an operator has approved by naming it. A pose older than `pose_timeout` is
refused rather than saved against the name the operator just spoke.

PERSISTENCE: the store file is the `locations_file` parameter, defaulting to
a per-machine runtime path outside the source tree (see `default_store_path`).
It is deliberately *not* web_ui/locations.json. That file is a committed demo
fixture for the local web console, and LocationStore rewrites the whole file
from an in-memory copy taken when it was constructed, with no locking and no
reload. Two writers means the second one silently drops the first one's
places. Sharing one file between this node and the web UI needs the store to
lock and re-read before writing, which is a change to LocationStore's
contract and its own tests, not something to imply by pointing both at the
same path.
"""

import math
import os
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from robot_core import Pose2D
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_locations.location_store import LocationStore


def default_store_path() -> Path:
    """Where saved places live when `locations_file` is left unset.

    Runtime state that the robot writes belongs outside the workspace: with
    --symlink-install an installed share directory points back into the
    source tree, so a node writing there would be editing checked-in files.
    """
    state_home = os.environ.get("XDG_STATE_HOME") or str(Path.home() / ".local/state")
    return Path(state_home) / "robot_locations" / "locations.json"


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """The heading about z of a quaternion, ignoring any roll or pitch."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocationNode(Node):
    """Save the robot's current pose under a name, and recall names as goals."""

    def __init__(self) -> None:
        super().__init__("location_manager")

        self.declare_parameter("locations_file", "")
        self.declare_parameter("goal_frame", "map")
        # A negative timeout means the check is off, matching the sentinel the
        # safety parameters use: ROS parameters have no null, and 0.0 is a
        # meaningful (impossible to satisfy) value.
        self.declare_parameter("pose_timeout", 2.0)

        locations_file = str(self.get_parameter("locations_file").value)
        self._goal_frame = str(self.get_parameter("goal_frame").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout").value)

        self._store_path = (
            Path(locations_file) if locations_file else default_store_path()
        )
        self._store = LocationStore(self._store_path)

        self._current_pose = None
        self._last_pose_time = None

        self._goal_publisher = self.create_publisher(PoseStamped, "goal_pose", 10)
        self._status_publisher = self.create_publisher(String, "location_status", 10)
        self.create_subscription(String, "save_location", self._on_save, 10)
        self.create_subscription(String, "recall_location", self._on_recall, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_pose, 10
        )
        self.create_service(Trigger, "~/list_locations", self._on_list_locations)

        self.get_logger().info(
            f"named places loaded from {self._store_path}: "
            f"{', '.join(self._store.names()) or 'none'}"
        )

    def _now(self) -> float:
        """Seconds from the node clock, so use_sim_time governs the pose age."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        orientation = pose.orientation
        self._current_pose = Pose2D(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
        )
        self._last_pose_time = self._now()

    def _report(self, message: str) -> None:
        status = String()
        status.data = message
        self._status_publisher.publish(status)
        self.get_logger().info(message)

    def _pose_to_save(self) -> Pose2D | None:
        """The current pose, or None when it is missing or too old to trust."""
        if self._current_pose is None or self._last_pose_time is None:
            return None
        if self._pose_timeout >= 0.0:
            age = self._now() - self._last_pose_time
            # A clock that jumped backwards gives a negative age, which is a
            # pose of unknown vintage rather than a very fresh one.
            if not 0.0 <= age <= self._pose_timeout:
                return None
        return self._current_pose

    def _on_save(self, message: String) -> None:
        pose = self._pose_to_save()
        if pose is None:
            self._report(
                f"refused to save {message.data!r}: the robot does not have a "
                "current pose to save"
            )
            return

        try:
            self._store.save_location(message.data, pose)
        except (ValueError, OSError) as error:
            # A mis-heard name must not take the node down with it: the next
            # command, including a recall, still has to be answered.
            self._report(f"refused to save {message.data!r}: {error}")
            return

        saved = self._store.get_goal(message.data)
        self._report(
            f"saved {saved.location_name} at "
            f"x={saved.pose.x:.2f} y={saved.pose.y:.2f} yaw={saved.pose.yaw:.2f}"
        )

    def _on_recall(self, message: String) -> None:
        try:
            goal = self._store.get_goal(message.data)
        except (KeyError, ValueError) as error:
            # KeyError stringifies with quotes around its argument; the store
            # puts a whole sentence in there, so read the argument directly.
            reason = str(error.args[0]) if error.args else str(error)
            self._report(f"cannot recall {message.data!r}: {reason}")
            return

        self._goal_publisher.publish(self._goal_message(goal.pose))
        self._report(f"sent {goal.location_name} to the navigator as a goal")

    def _goal_message(self, pose: Pose2D) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self._goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose.x
        goal.pose.position.y = pose.y
        # A planar goal: only the z/w pair of the quaternion carries the heading.
        goal.pose.orientation.z = math.sin(pose.yaw / 2.0)
        goal.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return goal

    def _on_list_locations(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        names = self._store.names()
        response.success = True
        response.message = ", ".join(names)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
