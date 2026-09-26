"""The rosbag2 adapter, against a bag written here rather than a fixture.

Writes a small recording with rosbag2's own writer, reads it back through the
adapter, and checks that what comes out is the same run - so the seam between
"a bag on disk" and "a sequence of records" is proven rather than assumed.
Skipped where ROS is not installed; the checks these records feed are tested
without it in test_invariants.py.
"""

from pathlib import Path
import tempfile
import unittest

try:
    from geometry_msgs.msg import Twist
    from rclpy.serialization import serialize_message
    from robot_telemetry.bag import read_publishers, read_records
    import rosbag2_py
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from robot_telemetry.records import (
    CMD_VEL,
    EMERGENCY_STOP,
    SAFETY_STATE,
    SCAN,
    Flag,
    SafetyStatus,
    Scan,
    Velocity,
)


def write_bag(directory: Path) -> None:
    """A three-second run: driving, an emergency stop, and the stop taking."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(directory)),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        CMD_VEL: "geometry_msgs/msg/Twist",
        SAFETY_STATE: "std_msgs/msg/String",
        SCAN: "sensor_msgs/msg/LaserScan",
        EMERGENCY_STOP: "std_msgs/msg/Bool",
    }
    for topic, type_name in types.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0, name=topic, type=type_name, serialization_format="cdr"
            )
        )

    moving = Twist()
    moving.linear.x = 0.25
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 10.0
    scan.ranges = [3.0, 2.5, 4.0]

    def write(topic, message, seconds):
        writer.write(topic, serialize_message(message), int(seconds * 1e9))

    write(SAFETY_STATE, String(data="clear: path clear"), 1.0)
    write(SCAN, scan, 1.1)
    write(CMD_VEL, moving, 1.1)
    write(EMERGENCY_STOP, Bool(data=True), 2.0)
    write(SAFETY_STATE, String(data="stop: emergency stop active"), 2.05)
    write(CMD_VEL, Twist(), 2.1)
    # The writer closes the storage when it goes out of scope here, which is
    # what makes the bag complete enough to read back.


@unittest.skipUnless(ROS_AVAILABLE, "requires a sourced ROS 2 installation")
class BagAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.bag = Path(self.temporary.name) / "run"
        write_bag(self.bag)
        self.records = read_records(self.bag)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_every_message_comes_back_in_time_order(self):
        self.assertEqual(len(self.records), 6)
        times = [record.timestamp for record in self.records]
        self.assertEqual(times, sorted(times))
        self.assertAlmostEqual(times[0], 1.0)

    def test_ros_messages_arrive_as_plain_records(self):
        by_topic = {}
        for record in self.records:
            by_topic.setdefault(record.topic, []).append(record.message)
        self.assertEqual(by_topic[CMD_VEL][0], Velocity(linear_x=0.25))
        self.assertEqual(by_topic[CMD_VEL][1], Velocity())
        self.assertEqual(by_topic[SCAN][0], Scan(2.5))
        self.assertEqual(by_topic[EMERGENCY_STOP][0], Flag(True))
        self.assertEqual(
            by_topic[SAFETY_STATE][1], SafetyStatus("stop", "emergency stop active")
        )

    def test_a_real_bag_analyses_end_to_end(self):
        from robot_telemetry.report import Verdict, build_report

        report = build_report(
            self.records, source=str(self.bag), stop_distance=0.45
        )
        # No topic graph was passed, so one check could not be answered and
        # the run is not a pass - but everything it could answer, it did.
        self.assertIs(report.verdict, Verdict.INCOMPLETE)
        self.assertEqual(len(report.emergency_stops), 1)
        self.assertAlmostEqual(report.emergency_stops[0].seconds, 0.1)

    def test_a_missing_recording_is_an_error_not_an_empty_run(self):
        with self.assertRaises(FileNotFoundError):
            read_records(self.bag.parent / "never-recorded")

    def test_a_bag_with_no_topic_graph_reports_none(self):
        """None, not {}: "nobody was publishing" is a different claim.

        The first means the question was not asked, the second that it was
        asked and the answer was nothing. Only the second would be a finding.
        """
        self.assertIsNone(read_publishers(self.bag))

    def test_a_recorded_topic_graph_is_read_back(self):
        (self.bag / "publishers.json").write_text(
            '{"topics": {"cmd_vel": ["/safety_controller"]}}'
        )
        self.assertEqual(
            read_publishers(self.bag), {CMD_VEL: ["/safety_controller"]}
        )

    def test_a_corrupt_topic_graph_is_treated_as_missing(self):
        (self.bag / "publishers.json").write_text("{not json")
        self.assertIsNone(read_publishers(self.bag))


if __name__ == "__main__":
    unittest.main()
