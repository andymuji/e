import time
import unittest

try:
    from geometry_msgs.msg import Twist
    import rclpy
    from robot_base.base_node import BaseNode
    from robot_base.protocol import parse_pico_line, seal, unseal

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

STOP_LINE = "M 0 0 0 0"


class FakeSerial:
    """A Pico on the end of a cable, as far as the driver can tell."""

    def __init__(self) -> None:
        self.incoming = bytearray()
        self.written: list[str] = []
        self.closed = False
        self.broken = False

    @property
    def in_waiting(self) -> int:
        return len(self.incoming)

    def read(self, size: int) -> bytes:
        data = bytes(self.incoming[:size])
        del self.incoming[:size]
        return data

    def write(self, data: bytes) -> int:
        if self.broken:
            raise OSError("device reports readiness to read but returned no data")
        self.written.append(unseal(data.decode("ascii")))
        return len(data)

    def close(self) -> None:
        self.closed = True

    def say(self, payload: str) -> None:
        self.incoming += (seal(payload) + "\n").encode("ascii")


class PublishedMessages:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def forward(speed: float = 0.2) -> "Twist":
    message = Twist()
    message.linear.x = speed
    return message


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class BaseNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.port = FakeSerial()
        self.opened = []

        def factory(name: str) -> FakeSerial:
            self.opened.append(name)
            return self.port

        self.node = BaseNode(serial_factory=factory)
        self.odom = PublishedMessages()
        self.node._odom_publisher = self.odom
        # The gate is the one publisher on /cmd_vel unless a test says not.
        self.publishers = ["safety_controller"]
        self.node._publishers_on_cmd_vel = lambda: self.publishers

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_the_gates_output_reaches_the_pico(self) -> None:
        self.node._on_cmd_vel(forward())
        self.node._on_timer()

        self.assertEqual(self.opened, ["/dev/ttyACM0"])
        # 0.2 m/s on 0.04 m wheels is 5 rad/s, of 60 at full power.
        self.assertEqual(self.port.written[-1], "M 83 83 83 83")

    def test_a_stale_cmd_vel_sends_zero(self) -> None:
        self.node._on_cmd_vel(forward())
        self.node._controller._command_time = time.monotonic() - 1.0

        self.node._on_timer()

        self.assertEqual(self.port.written[-1], STOP_LINE)

    def test_zero_is_sent_every_cycle_even_with_nothing_to_do(self) -> None:
        # The stream of zeros is what keeps the Pico's own timeout from being
        # the only thing holding the motors off.
        for _ in range(3):
            self.node._on_timer()

        self.assertEqual(self.port.written, [STOP_LINE] * 3)

    def test_a_second_publisher_on_the_wheel_topic_stops_the_wheels(self) -> None:
        self.publishers = ["safety_controller", "teleop_twist_keyboard"]
        self.node._on_cmd_vel(forward())

        self.node._on_timer()

        self.assertEqual(self.port.written[-1], STOP_LINE)

    def test_no_gate_on_the_wheel_topic_stops_the_wheels(self) -> None:
        self.publishers = []
        self.node._on_cmd_vel(forward())

        self.node._on_timer()

        self.assertEqual(self.port.written[-1], STOP_LINE)

    def test_shutdown_sends_zero_and_closes_the_port(self) -> None:
        self.node._on_cmd_vel(forward())
        self.node._on_timer()

        self.node.close()

        self.assertEqual(self.port.written[-1], STOP_LINE)
        self.assertTrue(self.port.closed)

    def test_a_failed_link_is_dropped_and_reopened_later(self) -> None:
        self.node._on_timer()
        self.port.broken = True
        self.node._on_timer()

        self.assertTrue(self.port.closed)
        self.assertIsNone(self.node._port)

        self.port.broken = False
        self.node._next_open = 0.0
        self.node._on_timer()

        self.assertEqual(len(self.opened), 2)

    def test_a_missing_pico_does_not_crash_the_driver(self) -> None:
        def missing(name: str):
            raise OSError(f"could not open port {name}")

        self.node._port = None
        self.node._serial_factory = missing
        self.node._next_open = 0.0

        self.node._on_timer()
        self.node._on_timer()

        self.assertIsNone(self.node._port)

    def test_encoder_reports_become_odometry(self) -> None:
        self.node._on_timer()
        counts = 660  # one wheel turn at the default counts_per_rev
        self.port.say("E 1000 1 0 0 0 0")
        self.port.say(f"E 1500 1 {counts} {counts} {counts} {counts}")

        self.node._on_timer()

        self.assertEqual(len(self.odom.messages), 1)
        odom = self.odom.messages[0]
        circumference = 2 * 3.141592653589793 * 0.04
        self.assertAlmostEqual(odom.pose.pose.position.x, circumference, places=6)
        self.assertAlmostEqual(odom.twist.twist.linear.x, circumference / 0.5, places=6)
        self.assertEqual(odom.header.frame_id, "odom")
        self.assertEqual(odom.child_frame_id, "base_link")

    def test_a_pico_reboot_does_not_become_a_leap_in_odometry(self) -> None:
        self.node._on_timer()
        self.port.say("E 1000 1 5000 5000 5000 5000")
        self.port.say("B watchdog")
        self.port.say("E 10 0 0 0 0 0")
        self.port.say("E 60 0 0 0 0 0")

        self.node._on_timer()

        self.assertEqual(len(self.odom.messages), 1)
        self.assertEqual(self.odom.messages[0].pose.pose.position.x, 0.0)

    def test_garbage_from_the_pico_is_survived(self) -> None:
        self.node._on_timer()
        self.port.incoming += b"Traceback (most recent call last):\n\xff\xfe\n"

        self.node._on_timer()

        self.assertEqual(self.port.written[-1], STOP_LINE)

    def test_the_tf_default_leaves_base_link_to_cartographer(self) -> None:
        # See test_base_mapping_launch.py for why: Cartographer publishes
        # odom -> base_link on the car.
        self.assertFalse(self.node.get_parameter("publish_tf").value)
        self.assertIsNone(self.node._tf)

    def test_every_line_sent_is_one_the_pico_accepts(self) -> None:
        self.node._on_cmd_vel(forward(100.0))
        self.node._on_timer()

        for line in self.port.written:
            fields = line.split(" ")
            self.assertEqual(fields[0], "M")
            for value in fields[1:]:
                self.assertLessEqual(abs(int(value)), 1000)
        # Reports are what the Pico sends; a command is never one.
        with self.assertRaises(ValueError):
            parse_pico_line(seal(self.port.written[-1]))


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class LiveGraphTests(unittest.TestCase):
    """The publisher check reads the real ROS graph, not a stub."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def test_a_teleop_publishing_cmd_vel_directly_is_seen(self) -> None:
        port = FakeSerial()
        driver = BaseNode(serial_factory=lambda name: port)
        teleop = rclpy.create_node("teleop_twist_keyboard")
        try:
            teleop.create_publisher(Twist, "cmd_vel", 10)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if "teleop_twist_keyboard" in driver._publishers_on_cmd_vel():
                    break
                time.sleep(0.05)

            driver._on_cmd_vel(forward())
            driver._on_timer()

            self.assertIn("teleop_twist_keyboard", driver._publishers_on_cmd_vel())
            self.assertEqual(port.written[-1], STOP_LINE)
        finally:
            teleop.destroy_node()
            driver.destroy_node()


if __name__ == "__main__":
    unittest.main()
