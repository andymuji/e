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
        self.controller.set_emergency_stop(True)

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


if __name__ == "__main__":
    unittest.main()
