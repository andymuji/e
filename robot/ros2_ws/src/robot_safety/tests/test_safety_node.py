import time
import unittest

try:
    from geometry_msgs.msg import (
        PoseWithCovarianceStamped,
        TransformStamped,
        Twist,
    )
    import rclpy
    from robot_safety.safety_controller import SafetyController
    from robot_safety.safety_node import STATE_QOS, SafetyNode
    from sensor_msgs.msg import BatteryState, LaserScan
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

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

    def test_a_late_subscriber_is_told_the_state_it_missed(self) -> None:
        """A console opened onto a quiet robot must not be left guessing.

        The gate reports its state only when that state changes, so on a
        settled robot the topic is silent for minutes. Without the kept last
        message, anything that subscribes in that quiet - a console, a
        recorder, an operator's echo - sees nothing and has to fail closed to
        "unknown" on a robot that is working perfectly well.
        """
        self.node._publish_safe_velocity()
        self.assertIsNotNone(self.node._last_status)

        heard = []
        listener = rclpy.create_node("late_console")
        try:
            listener.create_subscription(
                String, "safety_state", lambda message: heard.append(message.data),
                STATE_QOS,
            )
            deadline = time.monotonic() + 5.0
            while not heard and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=0.1)
        finally:
            listener.destroy_node()

        self.assertEqual(heard, [self.node._last_status])


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

    def localizer_heartbeat(self, parent: str = "map", child: str = "odom"):
        """The map->odom transform AMCL broadcasts while it is localizing."""
        message = TFMessage()
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        message.transforms = [transform]
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
        self.node._on_tf(self.localizer_heartbeat())
        self.node._publish_safe_velocity()

        self.assertAlmostEqual(self.published.messages[-1].linear.x, 1.0)

    def test_a_standing_robot_stays_localized_on_the_transform_alone(self) -> None:
        """The deadlock this replaced: no amcl_pose while the robot stands.

        AMCL only publishes amcl_pose once the robot has moved past
        update_min_d/update_min_a, so taking freshness from that topic meant
        the gate declared localization lost a second after the robot stopped
        and then held it at zero - and a robot held at zero can never produce
        the update that would clear the fault. The transform keeps arriving.
        """
        self.require_localization(max_localization_covariance=0.25)
        self.node._on_localization(self.localization(0.01, 0.01))
        # Whatever pose freshness the covariance message might have carried.
        self.node._last_localization_time = None

        self.node._on_tf(self.localizer_heartbeat())
        self.node._publish_safe_velocity()

        self.assertAlmostEqual(self.published.messages[-1].linear.x, 1.0)

    def test_an_unrelated_transform_is_not_a_localization_heartbeat(self) -> None:
        # odom->base_footprint comes from the wheels and is published whether
        # or not the robot knows where it is on the map.
        self.require_localization(max_localization_covariance=0.25)
        self.node._on_localization(self.localization(0.01, 0.01))

        self.node._on_tf(self.localizer_heartbeat("odom", "base_footprint"))
        self.node._publish_safe_velocity()

        self.assertIsNone(self.node._last_localization_time)
        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_a_stale_transform_still_stops_the_robot(self) -> None:
        # The heartbeat has to keep arriving; one old one does not stand in
        # for a localizer that has since died.
        self.require_localization(
            localization_timeout=1.0, max_localization_covariance=0.25
        )
        self.node._on_localization(self.localization(0.01, 0.01))
        self.node._on_tf(self.localizer_heartbeat())
        self.node._last_localization_time = self.node._now() - 5.0
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_diverged_pose_stops_the_robot_while_still_publishing(self) -> None:
        self.require_localization(max_localization_covariance=0.25)
        self.node._on_localization(self.localization(0.01, 1.0))
        self.node._on_tf(self.localizer_heartbeat())
        self.node._publish_safe_velocity()

        self.assertEqual(self.published.messages[-1].linear.x, 0.0)

    def test_stale_pose_stops_the_robot(self) -> None:
        self.require_localization(localization_timeout=1.0)
        self.node._on_localization(self.localization(0.01, 0.01))
        self.node._on_tf(self.localizer_heartbeat())
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
