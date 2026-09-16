import unittest

try:
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    import rclpy
    from robot_safety.safety_controller import SafetyController
    from robot_safety.safety_node import SafetyNode
    from sensor_msgs.msg import BatteryState, LaserScan
    from std_msgs.msg import Bool

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

    def test_silent_commander_stops_robot(self) -> None:
        scan = LaserScan()
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [2.0]
        self.node._on_scan(scan)

        # The commanding node died: the path is clear but nothing is asking
        # for motion any more, so the last request must not keep the wheels on.
        self.node._last_command_time = self.node._now() - 5.0
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

        # The stale request is dropped, not replayed once commands resume.
        self.node._last_command_time = self.node._now()
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_emergency_stop_is_not_released_by_a_false_message(self) -> None:
        scan = LaserScan()
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [2.0]
        self.node._on_scan(scan)

        self.node._on_emergency_stop(Bool(data=True))
        self.node._on_emergency_stop(Bool(data=False))
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

        self.node._on_emergency_stop_reset(Bool(data=True))
        self.node._publish_safe_velocity()

        self.assertAlmostEqual(self.published.messages[-1].linear.x, 1.0)

    def test_missing_scan_stops_robot(self) -> None:
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class SafetyNodeHealthTests(unittest.TestCase):
    """The localization and battery inputs, as the node actually reads them."""

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

        scan = LaserScan()
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [2.0]
        self.node._on_scan(scan)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def require_localization(self, **overrides) -> None:
        """Rebuild the gate with the checks navigation.launch.py turns on."""
        self.node._controller = SafetyController(require_localization=True, **overrides)

    def localization(self, x_variance: float, y_variance: float):
        message = PoseWithCovarianceStamped()
        covariance = [0.0] * 36
        covariance[0] = x_variance
        covariance[7] = y_variance
        message.pose.covariance = covariance
        return message

    def test_node_reads_the_worse_of_the_two_position_variances(self) -> None:
        # A pose that is confident in x and lost in y is lost.
        self.node._on_localization(self.localization(0.01, 4.0))

        self.assertAlmostEqual(self.node._localization_covariance, 4.0)

    def test_missing_pose_stops_the_robot_under_navigation(self) -> None:
        self.require_localization()
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_fresh_confident_pose_allows_motion(self) -> None:
        self.require_localization(max_localization_covariance=0.25)
        self.node._on_localization(self.localization(0.01, 0.01))
        self.node._publish_safe_velocity()

        self.assertAlmostEqual(self.published.messages[-1].linear.x, 1.0)

    def test_diverged_pose_stops_the_robot_while_still_publishing(self) -> None:
        self.require_localization(max_localization_covariance=0.25)
        self.node._on_localization(self.localization(0.01, 1.0))
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_stale_pose_stops_the_robot(self) -> None:
        self.require_localization(localization_timeout=1.0)
        self.node._on_localization(self.localization(0.01, 0.01))
        self.node._last_localization_time = self.node._now() - 5.0
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_battery_percentage_reaches_the_controller(self) -> None:
        self.node._controller = SafetyController(low_battery_fraction=0.15)
        message = BatteryState()
        message.percentage = 0.05
        self.node._on_battery(message)
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_a_charged_battery_allows_motion(self) -> None:
        self.node._controller = SafetyController(low_battery_fraction=0.15)
        message = BatteryState()
        message.percentage = 0.8
        self.node._on_battery(message)
        self.node._publish_safe_velocity()

        self.assertAlmostEqual(self.published.messages[-1].linear.x, 1.0)

    def test_a_driver_reporting_nan_is_not_treated_as_full(self) -> None:
        self.node._controller = SafetyController(low_battery_fraction=0.15)
        message = BatteryState()
        message.percentage = float("nan")
        self.node._on_battery(message)
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
