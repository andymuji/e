"""The adapter's only piece of judgement: turning a message into a record.

Tested with stand-in objects rather than real ROS messages, because the
conversion is deliberately written against the shape of a message and not
against its class. That is what keeps rosbag2 and rclpy out of every other
test in this package.
"""

import math
from types import SimpleNamespace
import unittest

from robot_telemetry.records import (
    Flag,
    Opaque,
    SafetyStatus,
    Scan,
    Velocity,
    canonical_topic,
    convert,
    nearest_range,
)


def twist(**components) -> SimpleNamespace:
    axes = {"x": 0.0, "y": 0.0, "z": 0.0}
    return SimpleNamespace(
        linear=SimpleNamespace(**{**axes, **components.get("linear", {})}),
        angular=SimpleNamespace(**{**axes, **components.get("angular", {})}),
    )


def scan(ranges, range_min=0.1, range_max=10.0) -> SimpleNamespace:
    return SimpleNamespace(ranges=ranges, range_min=range_min, range_max=range_max)


class ConvertTests(unittest.TestCase):
    def test_a_twist_becomes_a_velocity(self):
        converted = convert(twist(linear={"x": 0.3}, angular={"z": -0.1}))
        self.assertEqual(converted, Velocity(linear_x=0.3, angular_z=-0.1))

    def test_a_bool_becomes_a_flag(self):
        self.assertEqual(convert(SimpleNamespace(data=True)), Flag(True))

    def test_a_string_becomes_a_safety_status(self):
        converted = convert(SimpleNamespace(data="caution: obstacle inside caution distance"))
        self.assertEqual(
            converted, SafetyStatus("caution", "obstacle inside caution distance")
        )

    def test_a_scan_becomes_its_nearest_return(self):
        self.assertEqual(convert(scan([2.0, 0.6, 3.0])), Scan(0.6))

    def test_anything_else_is_kept_as_opaque(self):
        class Odometry:
            pass

        self.assertEqual(convert(Odometry()), Opaque("Odometry"))


class NearestRangeTests(unittest.TestCase):
    def test_out_of_range_returns_are_ignored(self):
        """A reading outside the sensor's own limits is not an obstacle.

        The gate discards them, so an analyser that counted them would measure
        a stop distance the gate never saw and report a latency for an
        obstacle that was never there.
        """
        self.assertEqual(nearest_range(scan([0.05, 20.0, 1.5])), 1.5)

    def test_infinities_and_nans_are_ignored(self):
        self.assertEqual(
            nearest_range(scan([float("inf"), float("nan"), 2.0])), 2.0
        )

    def test_a_scan_with_nothing_usable_reports_nothing(self):
        self.assertIsNone(nearest_range(scan([float("nan"), float("inf")])))


class VelocityTests(unittest.TestCase):
    def test_a_tiny_rounding_artefact_counts_as_stopped(self):
        self.assertTrue(Velocity(linear_x=1e-12).is_zero())

    def test_a_real_command_does_not(self):
        self.assertFalse(Velocity(angular_z=0.05).is_zero())

    def test_a_nan_command_is_never_stopped(self):
        """An uninterpretable command is a fault, not a stop.

        `max` over a NaN depends on argument order, so without the explicit
        guard a NaN in one axis could silently report as zero speed.
        """
        self.assertFalse(Velocity(linear_y=float("nan")).is_zero())
        self.assertFalse(math.isfinite(Velocity(linear_y=float("nan")).magnitude))


class SafetyStatusTests(unittest.TestCase):
    def test_a_status_without_a_reason_is_kept(self):
        self.assertEqual(SafetyStatus.parse("stop"), SafetyStatus("stop", ""))

    def test_only_an_emergency_stop_is_an_emergency(self):
        self.assertTrue(SafetyStatus.parse("stop: emergency stop active").emergency)
        self.assertFalse(
            SafetyStatus.parse("stop: obstacle sensor timed out").emergency
        )
        self.assertFalse(SafetyStatus.parse("clear: path clear").stopped)


class TopicTests(unittest.TestCase):
    def test_a_bare_topic_name_matches_an_absolute_one(self):
        self.assertEqual(canonical_topic("cmd_vel"), canonical_topic("/cmd_vel"))


if __name__ == "__main__":
    unittest.main()
