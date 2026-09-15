import unittest

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool

    from robot_safety.safety_node import SafetyNode

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class PublishedMessages:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class SafetyNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = SafetyNode()
        self.published = PublishedMessages()
        self.node._velocity_publisher = self.published

        requested = Twist()
        requested.linear.x = 1.0
        self.node._on_velocity(requested)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_nearby_obstacle_limits_requested_velocity(self) -> None:
        scan = LaserScan()
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [0.6, float("nan"), 2.0]

        self.node._on_scan(scan)
        self.node._publish_safe_velocity()

        self.assertEqual(len(self.published.messages), 1)
        self.assertAlmostEqual(self.published.messages[-1].linear.x, 0.35)

    def test_emergency_stop_overrides_requested_velocity(self) -> None:
        scan = LaserScan()
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [2.0]

        self.node._on_scan(scan)
        self.node._on_emergency_stop(Bool(data=True))
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()