import math
import unittest

from robot_safety import SafetyController, SafetyState


class SafetyControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = SafetyController(
            stop_distance=0.35,
            caution_distance=0.8,
            sensor_timeout=0.5,
        )

    def test_clear_path_allows_full_speed(self) -> None:
        decision = self.controller.evaluate(1.5)

        self.assertIs(decision.state, SafetyState.CLEAR)
        self.assertEqual(decision.speed_scale, 1.0)

    def test_nearby_obstacle_reduces_speed(self) -> None:
        decision = self.controller.evaluate(0.6)

        self.assertIs(decision.state, SafetyState.CAUTION)
        self.assertEqual(decision.speed_scale, 0.35)

    def test_close_obstacle_stops_robot(self) -> None:
        decision = self.controller.evaluate(0.2)

        self.assertIs(decision.state, SafetyState.STOP)
        self.assertEqual(decision.speed_scale, 0.0)

    def test_emergency_stop_overrides_clear_sensor(self) -> None:
        self.controller.engage_emergency_stop()

        decision = self.controller.evaluate(10.0)

        self.assertIs(decision.state, SafetyState.STOP)
        self.assertEqual(decision.speed_scale, 0.0)
        self.assertEqual(decision.reason, "emergency stop active")

    def test_invalid_sensor_reading_stops_robot(self) -> None:
        for reading in (None, math.nan, math.inf, -math.inf):
            with self.subTest(reading=reading):
                decision = self.controller.evaluate(reading)
                self.assertIs(decision.state, SafetyState.STOP)
                self.assertEqual(decision.speed_scale, 0.0)

    def test_stale_sensor_reading_stops_robot(self) -> None:
        decision = self.controller.evaluate(10.0, reading_age=0.51)

        self.assertIs(decision.state, SafetyState.STOP)
        self.assertEqual(decision.speed_scale, 0.0)
        self.assertEqual(decision.reason, "obstacle sensor timed out")

    def test_invalid_sensor_timestamp_stops_robot(self) -> None:
        for reading_age in (-0.01, math.nan, math.inf, -math.inf):
            with self.subTest(reading_age=reading_age):
                decision = self.controller.evaluate(10.0, reading_age=reading_age)
                self.assertIs(decision.state, SafetyState.STOP)
                self.assertEqual(decision.speed_scale, 0.0)
                self.assertEqual(decision.reason, "invalid obstacle sensor timestamp")

    def test_distances_must_be_ordered(self) -> None:
        with self.assertRaises(ValueError):
            SafetyController(stop_distance=0.8, caution_distance=0.8)

    def test_sensor_timeout_must_be_finite_and_positive(self) -> None:
        for sensor_timeout in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(sensor_timeout=sensor_timeout):
                with self.assertRaises(ValueError):
                    SafetyController(sensor_timeout=sensor_timeout)


    def test_emergency_stop_latches_until_explicit_reset(self) -> None:
        self.controller.engage_emergency_stop()

        # A clear sensor picture must not release the latch on its own.
        self.assertIs(self.controller.evaluate(10.0).state, SafetyState.STOP)
        self.assertTrue(self.controller.emergency_stop_engaged)

        self.controller.clear_emergency_stop()

        self.assertIs(self.controller.evaluate(10.0).state, SafetyState.CLEAR)

    def test_stale_motion_command_stops_robot(self) -> None:
        decision = self.controller.evaluate(10.0, command_age=0.51)

        self.assertIs(decision.state, SafetyState.STOP)
        self.assertEqual(decision.speed_scale, 0.0)
        self.assertEqual(decision.reason, "motion command timed out")

    def test_missing_inputs_stop_robot(self) -> None:
        cases = (
            ({"reading_age": None}, "no obstacle sensor received"),
            ({"command_age": None}, "no motion command received"),
        )
        for kwargs, reason in cases:
            with self.subTest(**kwargs):
                decision = self.controller.evaluate(10.0, **kwargs)
                self.assertIs(decision.state, SafetyState.STOP)
                self.assertEqual(decision.speed_scale, 0.0)
                self.assertEqual(decision.reason, reason)

    def test_invalid_command_timestamp_stops_robot(self) -> None:
        for command_age in (-0.01, math.nan, math.inf, -math.inf):
            with self.subTest(command_age=command_age):
                decision = self.controller.evaluate(10.0, command_age=command_age)
                self.assertIs(decision.state, SafetyState.STOP)
                self.assertEqual(decision.speed_scale, 0.0)
                self.assertEqual(decision.reason, "invalid motion command timestamp")

    def test_command_timeout_must_be_finite_and_positive(self) -> None:
        for command_timeout in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(command_timeout=command_timeout):
                with self.assertRaises(ValueError):
                    SafetyController(command_timeout=command_timeout)


if __name__ == "__main__":
    unittest.main()


class LocalizationTests(unittest.TestCase):
    """Localization failure is a stop condition, not a navigation problem.

    A robot that has lost its pose is still perfectly capable of driving, and
    that is the danger: it will drive confidently to the wrong place.
    """

    def test_localization_is_ignored_until_it_is_required(self) -> None:
        # Teleop has no localization to lose. Demanding it unconditionally
        # would mean the gate refuses to move a manually driven robot.
        controller = SafetyController()

        decision = controller.evaluate(2.0, 0.0, 0.0, localization_age=None)

        self.assertEqual(decision.state, SafetyState.CLEAR)

    def test_missing_localization_stops_when_required(self) -> None:
        controller = SafetyController(require_localization=True)

        decision = controller.evaluate(2.0, 0.0, 0.0, localization_age=None)

        self.assertEqual(decision.state, SafetyState.STOP)
        self.assertEqual(decision.speed_scale, 0.0)
        self.assertEqual(decision.reason, "no localization received")

    def test_stale_localization_stops(self) -> None:
        controller = SafetyController(
            require_localization=True, localization_timeout=1.0
        )

        decision = controller.evaluate(2.0, 0.0, 0.0, localization_age=1.5)

        self.assertEqual(decision.state, SafetyState.STOP)
        self.assertEqual(decision.reason, "localization timed out")

    def test_fresh_localization_allows_motion(self) -> None:
        controller = SafetyController(
            require_localization=True, localization_timeout=1.0
        )

        decision = controller.evaluate(2.0, 0.0, 0.0, localization_age=0.2)

        self.assertEqual(decision.state, SafetyState.CLEAR)

    def test_diverged_localization_stops_even_while_publishing(self) -> None:
        # AMCL keeps publishing after it has lost the robot, so freshness
        # alone is not evidence that the pose means anything.
        controller = SafetyController(
            require_localization=True, max_localization_covariance=0.25
        )

        decision = controller.evaluate(
            2.0, 0.0, 0.0, localization_age=0.0, localization_covariance=1.0
        )

        self.assertEqual(decision.state, SafetyState.STOP)
        self.assertEqual(decision.reason, "localization uncertainty too high")

    def test_confident_localization_allows_motion(self) -> None:
        controller = SafetyController(
            require_localization=True, max_localization_covariance=0.25
        )

        decision = controller.evaluate(
            2.0, 0.0, 0.0, localization_age=0.0, localization_covariance=0.05
        )

        self.assertEqual(decision.state, SafetyState.CLEAR)

    def test_missing_or_invalid_covariance_stops(self) -> None:
        controller = SafetyController(
            require_localization=True, max_localization_covariance=0.25
        )

        for covariance in (None, float("nan"), float("inf")):
            with self.subTest(covariance=covariance):
                decision = controller.evaluate(
                    2.0,
                    0.0,
                    0.0,
                    localization_age=0.0,
                    localization_covariance=covariance,
                )

                self.assertEqual(decision.state, SafetyState.STOP)
                self.assertEqual(decision.reason, "invalid localization covariance")

    def test_emergency_stop_outranks_a_healthy_pose(self) -> None:
        controller = SafetyController(require_localization=True)
        controller.engage_emergency_stop()

        decision = controller.evaluate(2.0, 0.0, 0.0, localization_age=0.0)

        self.assertEqual(decision.reason, "emergency stop active")

    def test_rejects_an_unusable_localization_timeout(self) -> None:
        for bad in (0.0, -1.0, float("inf"), float("nan")):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    SafetyController(localization_timeout=bad)

    def test_rejects_an_unusable_covariance_limit(self) -> None:
        for bad in (0.0, -1.0, float("inf"), float("nan")):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    SafetyController(max_localization_covariance=bad)


class BatteryTests(unittest.TestCase):
    def test_battery_is_ignored_until_a_reserve_is_configured(self) -> None:
        # The simulated base has no battery to report.
        controller = SafetyController()

        decision = controller.evaluate(2.0, 0.0, 0.0, battery_fraction=None)

        self.assertEqual(decision.state, SafetyState.CLEAR)

    def test_flat_battery_stops(self) -> None:
        controller = SafetyController(low_battery_fraction=0.15)

        decision = controller.evaluate(2.0, 0.0, 0.0, battery_fraction=0.10)

        self.assertEqual(decision.state, SafetyState.STOP)
        self.assertEqual(decision.speed_scale, 0.0)
        self.assertEqual(decision.reason, "battery below reserve")

    def test_the_reserve_threshold_itself_stops(self) -> None:
        controller = SafetyController(low_battery_fraction=0.15)

        decision = controller.evaluate(2.0, 0.0, 0.0, battery_fraction=0.15)

        self.assertEqual(decision.state, SafetyState.STOP)

    def test_a_charged_battery_allows_motion(self) -> None:
        controller = SafetyController(low_battery_fraction=0.15)

        decision = controller.evaluate(2.0, 0.0, 0.0, battery_fraction=0.9)

        self.assertEqual(decision.state, SafetyState.CLEAR)

    def test_unknown_battery_stops_once_it_is_monitored(self) -> None:
        # A driver that stops reporting is not evidence of a full battery.
        controller = SafetyController(low_battery_fraction=0.15)

        for fraction in (None, float("nan")):
            with self.subTest(fraction=fraction):
                decision = controller.evaluate(
                    2.0, 0.0, 0.0, battery_fraction=fraction
                )

                self.assertEqual(decision.state, SafetyState.STOP)
                self.assertEqual(decision.reason, "battery level unknown")

    def test_rejects_an_unusable_reserve(self) -> None:
        for bad in (0.0, -0.1, 1.5, float("nan")):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    SafetyController(low_battery_fraction=bad)

    def test_localization_is_reported_before_battery(self) -> None:
        # Both are stops; the reason should name the fault that the operator
        # has to act on first.
        controller = SafetyController(
            require_localization=True, low_battery_fraction=0.15
        )

        decision = controller.evaluate(
            2.0, 0.0, 0.0, localization_age=None, battery_fraction=0.05
        )

        self.assertEqual(decision.reason, "no localization received")
