"""Tests for the stop and caution distance derivation."""

import unittest

from robot_safety.distances import (
    BaseDynamics,
    derive_safety_distances,
)


def dynamics(**overrides) -> BaseDynamics:
    values = {
        "max_speed": 0.5,
        "max_deceleration": 1.0,
        "sensor_period": 0.1,
        "control_period": 0.05,
        "actuation_lag": 0.1,
        "sensor_to_front_edge": 0.05,
        "caution_speed_scale": 0.35,
        "safety_factor": 1.5,
        "distance_step": 0.05,
    }
    values.update(overrides)
    return BaseDynamics(**values)


class BaseDynamicsTests(unittest.TestCase):
    def test_reaction_time_sums_every_stage_of_the_delay(self) -> None:
        # Missing any one of these underestimates how far the robot travels
        # between seeing an obstacle and braking for it.
        self.assertAlmostEqual(dynamics().reaction_time, 0.25)

    def test_rejects_non_positive_and_non_finite_inputs(self) -> None:
        for field in (
            "max_speed",
            "max_deceleration",
            "sensor_period",
            "control_period",
            "safety_factor",
            "distance_step",
        ):
            for bad in (0.0, -1.0, float("inf"), float("nan")):
                with self.subTest(field=field, value=bad):
                    with self.assertRaises(ValueError):
                        dynamics(**{field: bad})

    def test_allows_zero_lag_and_zero_overhang_but_not_negative(self) -> None:
        self.assertIsNotNone(dynamics(actuation_lag=0.0, sensor_to_front_edge=0.0))
        with self.assertRaises(ValueError):
            dynamics(actuation_lag=-0.01)
        with self.assertRaises(ValueError):
            dynamics(sensor_to_front_edge=-0.01)

    def test_rejects_a_safety_factor_below_one(self) -> None:
        # Below 1.0 the published distance is shorter than the physics says.
        with self.assertRaises(ValueError):
            dynamics(safety_factor=0.9)

    def test_rejects_a_caution_speed_that_is_not_a_reduction(self) -> None:
        for bad in (1.0, 1.5, -0.1):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    dynamics(caution_speed_scale=bad)

    def test_from_mapping_rejects_unknown_keys(self) -> None:
        values = {
            "max_speed": 0.5,
            "max_deceleration": 1.0,
            "sensor_period": 0.1,
            "control_period": 0.05,
            "actuation_lag": 0.1,
            "sensor_to_front_edge": 0.05,
            "caution_speed_scale": 0.35,
            "safety_factor": 1.5,
            "distance_step": 0.05,
        }
        # A typo would otherwise leave the stricter default silently in place.
        with self.assertRaises(ValueError):
            BaseDynamics.from_mapping({**values, "max_sped": 2.0})

    def test_from_mapping_rejects_a_missing_key(self) -> None:
        with self.assertRaises(ValueError):
            BaseDynamics.from_mapping({"max_speed": 0.5})

    def test_from_mapping_defaults_to_unmeasured(self) -> None:
        values = {
            "max_speed": 0.5,
            "max_deceleration": 1.0,
            "sensor_period": 0.1,
            "control_period": 0.05,
            "actuation_lag": 0.1,
            "sensor_to_front_edge": 0.05,
            "caution_speed_scale": 0.35,
            "safety_factor": 1.5,
            "distance_step": 0.05,
        }
        self.assertFalse(BaseDynamics.from_mapping(values).measured)


class DerivationTests(unittest.TestCase):
    def test_caution_zone_is_outside_the_stop_zone(self) -> None:
        derived = derive_safety_distances(dynamics())

        self.assertGreater(derived.caution_distance, derived.stop_distance)

    def test_stop_distance_covers_reaction_braking_and_overhang(self) -> None:
        derived = derive_safety_distances(dynamics())

        self.assertGreaterEqual(
            derived.stop_distance,
            0.05 + derived.reaction_distance + derived.braking_distance,
        )

    def test_a_faster_base_needs_more_room(self) -> None:
        slow = derive_safety_distances(dynamics(max_speed=0.3))
        fast = derive_safety_distances(dynamics(max_speed=1.0))

        self.assertGreater(fast.stop_distance, slow.stop_distance)
        self.assertGreater(fast.caution_distance, slow.caution_distance)

    def test_a_base_that_brakes_worse_needs_more_room(self) -> None:
        good = derive_safety_distances(dynamics(max_deceleration=2.0))
        poor = derive_safety_distances(dynamics(max_deceleration=0.5))

        self.assertGreater(poor.stop_distance, good.stop_distance)

    def test_a_slower_sensor_needs_more_room(self) -> None:
        # A 5 Hz lidar means a reading can be twice as old before it is acted
        # on, and the robot travels that whole time.
        fast = derive_safety_distances(dynamics(sensor_period=0.1))
        slow = derive_safety_distances(dynamics(sensor_period=0.2))

        self.assertGreater(slow.stop_distance, fast.stop_distance)

    def test_overhang_shifts_both_distances_outward(self) -> None:
        flush = derive_safety_distances(dynamics(sensor_to_front_edge=0.0))
        nose = derive_safety_distances(dynamics(sensor_to_front_edge=0.3))

        self.assertGreater(nose.stop_distance, flush.stop_distance)
        self.assertGreater(nose.caution_distance, flush.caution_distance)

    def test_rounding_never_shortens_a_distance(self) -> None:
        for step in (0.01, 0.05, 0.1, 0.25):
            with self.subTest(step=step):
                derived = derive_safety_distances(dynamics(distance_step=step))
                raw = 0.05 + 1.5 * (
                    derived.reaction_distance + derived.braking_distance
                )

                self.assertGreaterEqual(derived.stop_distance + 1e-9, raw)

                # A value sitting exactly on the grid shows up as either ~0
                # or ~step under floating-point modulo, so measure the
                # distance to the nearer grid line.
                offset = derived.stop_distance % step
                self.assertLess(min(offset, step - offset), 1e-6)

    def test_derivation_is_deterministic(self) -> None:
        self.assertEqual(
            derive_safety_distances(dynamics()),
            derive_safety_distances(dynamics()),
        )

    def test_the_gate_accepts_every_derived_pair(self) -> None:
        from robot_safety import SafetyController

        for speed in (0.2, 0.5, 1.0, 1.5):
            for decel in (0.5, 1.0, 2.0):
                with self.subTest(speed=speed, decel=decel):
                    derived = derive_safety_distances(
                        dynamics(max_speed=speed, max_deceleration=decel)
                    )
                    controller = SafetyController(
                        stop_distance=derived.stop_distance,
                        caution_distance=derived.caution_distance,
                    )

                    self.assertEqual(
                        controller.stop_distance, derived.stop_distance
                    )


if __name__ == "__main__":
    unittest.main()
