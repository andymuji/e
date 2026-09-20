"""What can only be checked against a real ROS graph.

Everything about how the console translates an answer is tested without ROS in
the sibling files. What is left here is the wiring: that the topics the safety
rules name are the topics this node actually publishes on, and that the map
subscription's QoS matches a latched map.
"""

from pathlib import Path
import tempfile
import unittest

try:
    from nav_msgs.msg import OccupancyGrid
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from robot_console.console_node import MAP_QOS, STATE_QOS, ConsoleNode
    from robot_console.map_provider import (
        pose_from_transform,
        snapshot_from_message,
    )
    from robot_core import Pose2D
    from robot_locations import LocationStore
    from std_msgs.msg import Bool, String

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def transform(x: float, y: float, z: float = 0.0, w: float = 1.0):
    from geometry_msgs.msg import TransformStamped

    stamped = TransformStamped()
    stamped.transform.translation.x = x
    stamped.transform.translation.y = y
    stamped.transform.rotation.z = z
    stamped.transform.rotation.w = w
    return stamped


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 is not installed in this environment")
class ConsoleNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        path = Path(self.directory.name) / "locations.json"
        LocationStore(path).save_location("Kitchen", Pose2D(1.0, 2.0))

        self.node = ConsoleNode(
            parameter_overrides=[
                Parameter("locations_file", value=str(path)),
                # Port 0 lets the operating system pick a free one, so the
                # test never fights a console someone left running.
                Parameter("port", value=0),
            ]
        )
        self.listener = rclpy.create_node("console_test_listener")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)

    def tearDown(self) -> None:
        self.executor.shutdown()
        self.listener.destroy_node()
        self.node.destroy_node()
        self.directory.cleanup()

    def spin(self, seconds: float = 0.5) -> None:
        self.executor.spin_once(timeout_sec=0.05)
        for _ in range(int(seconds / 0.05)):
            self.executor.spin_once(timeout_sec=0.05)

    def collect(self, topic: str) -> list:
        received = []
        self.listener.create_subscription(
            Bool, topic, lambda message: received.append(message.data), 10
        )
        self.spin(0.2)
        return received

    def test_requires_the_approved_destination_store(self) -> None:
        with self.assertRaises(ValueError):
            ConsoleNode(parameter_overrides=[Parameter("locations_file", value="")])

    def test_the_stop_button_publishes_true_on_emergency_stop(self) -> None:
        stops = self.collect("emergency_stop")
        resets = self.collect("emergency_stop_reset")

        self.node.app.engage_emergency_stop()
        self.spin()

        self.assertEqual(stops, [True])
        self.assertEqual(resets, [])

    def test_the_reset_publishes_true_on_the_reset_topic_only(self) -> None:
        stops = self.collect("emergency_stop")
        resets = self.collect("emergency_stop_reset")

        self.node.app.reset_emergency_stop()
        self.spin()

        self.assertEqual(resets, [True])
        # Never a False on emergency_stop: that does not release the latch,
        # and a console that published one would be pretending it does.
        self.assertEqual(stops, [])

    def test_the_console_publishes_no_velocity(self) -> None:
        # The safety rule this node lives under: only robot_safety publishes
        # /cmd_vel, and the console is a motion source, never an authority.
        published = [
            topic
            for topic, _ in self.node.get_publisher_names_and_types_by_node(
                self.node.get_name(), self.node.get_namespace()
            )
        ]

        self.assertNotIn("/cmd_vel", published)
        self.assertNotIn("/cmd_vel_requested", published)
        self.assertIn("/emergency_stop", published)
        self.assertIn("/emergency_stop_reset", published)

    def test_the_status_follows_the_gates_own_topic(self) -> None:
        # STATE_QOS on both ends, as the gate publishes it: the console asks
        # for the gate's kept last state, and a publisher that does not keep
        # one is not a match at all - it would deliver nothing, silently.
        state = self.listener.create_publisher(String, "safety_state", STATE_QOS)
        state.publish(String(data="caution: obstacle inside caution distance"))
        self.spin()
        self.node.safety.on_heartbeat()

        status = self.node.app.status()
        self.assertEqual(status["safety_state"], "caution")
        self.assertEqual(status["safety_message"], "obstacle inside caution distance")

    def test_a_goal_with_no_navigation_running_is_refused(self) -> None:
        # Nav2 is not running in this test, which is the point: the console
        # must say so rather than report a trip nothing is driving.
        self.node.safety.on_state("clear: path clear")
        self.node.safety.on_heartbeat()

        with self.assertRaises(RuntimeError) as caught:
            self.node.app.send_goal("kitchen")

        self.assertIn("navigation is not running", str(caught.exception))

    def test_the_map_subscription_matches_a_latched_map(self) -> None:
        # map_server publishes the map once, transient local. A volatile
        # subscription to it receives nothing at all, forever.
        from rclpy.qos import DurabilityPolicy

        self.assertEqual(MAP_QOS.durability, DurabilityPolicy.TRANSIENT_LOCAL)

    def test_an_arriving_map_is_drawn(self) -> None:
        message = OccupancyGrid()
        message.info.resolution = 0.5
        message.info.width = 2
        message.info.height = 2
        message.info.origin.position.x = -1.0
        message.data = [0, 0, 100, 100]

        self.node._on_map(message)

        payload = self.node.app.map_data()
        self.assertTrue(payload["available"])
        self.assertEqual(payload["image"]["min_x"], -1.0)

    def test_a_map_with_no_resolution_is_ignored(self) -> None:
        message = OccupancyGrid()
        message.info.width = 2
        message.info.height = 2

        self.assertIsNone(snapshot_from_message(message))

    def test_the_pose_comes_out_of_the_transform(self) -> None:
        pose = pose_from_transform(transform(1.5, -2.5, z=1.0, w=0.0))

        self.assertAlmostEqual(pose.x, 1.5)
        self.assertAlmostEqual(pose.y, -2.5)
        self.assertAlmostEqual(abs(pose.yaw), 3.141592653589793, places=6)

    def test_no_transform_is_no_pose(self) -> None:
        # Nothing is broadcasting map->base_link in this test, so the console
        # has no position to draw and must not invent one.
        self.assertIsNone(self.node._robot_pose())


if __name__ == "__main__":
    unittest.main()
