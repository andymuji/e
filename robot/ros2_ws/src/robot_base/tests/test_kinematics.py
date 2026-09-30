import math
import unittest

from robot_base.kinematics import (
    MecanumGeometry,
    OdometryIntegrator,
    limit_body,
    wheel_commands,
)

GEOMETRY = MecanumGeometry(wheel_radius=0.05, half_wheelbase=0.1, half_track=0.15)


def signs(values) -> tuple[int, ...]:
    return tuple(0 if abs(v) < 1e-12 else (1 if v > 0 else -1) for v in values)


class InverseKinematicsTests(unittest.TestCase):
    def test_forward_turns_every_wheel_forward_equally(self) -> None:
        wheels = GEOMETRY.inverse(0.5, 0.0, 0.0)

        self.assertEqual(signs(wheels), (1, 1, 1, 1))
        for speed in wheels:
            self.assertAlmostEqual(speed, 0.5 / 0.05)

    def test_sideways_left_turns_the_diagonals_opposite_ways(self) -> None:
        # Front-left and rear-right back, front-right and rear-left forward.
        self.assertEqual(signs(GEOMETRY.inverse(0.0, 0.3, 0.0)), (-1, 1, 1, -1))

    def test_turning_anticlockwise_runs_the_left_side_backwards(self) -> None:
        self.assertEqual(signs(GEOMETRY.inverse(0.0, 0.0, 1.0)), (-1, 1, -1, 1))

    def test_forward_kinematics_undoes_the_inverse(self) -> None:
        for body in [(0.3, 0.0, 0.0), (0.0, -0.2, 0.0), (0.1, 0.2, -0.7), (0, 0, 0)]:
            with self.subTest(body=body):
                for got, want in zip(
                    GEOMETRY.forward(GEOMETRY.inverse(*body)), body, strict=True
                ):
                    self.assertAlmostEqual(got, want)

    def test_impossible_geometry_is_refused(self) -> None:
        for bad in [0.0, -0.1, math.nan, math.inf]:
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                MecanumGeometry(bad, 0.1, 0.1)


class LimitTests(unittest.TestCase):
    def test_a_diagonal_request_is_capped_on_its_length(self) -> None:
        vx, vy, _ = limit_body(0.3, 0.3, 0.0, max_linear=0.3, max_angular=1.0)

        self.assertAlmostEqual(math.hypot(vx, vy), 0.3)
        self.assertAlmostEqual(vx, vy)

    def test_turn_rate_is_capped_both_ways(self) -> None:
        self.assertEqual(limit_body(0, 0, 5.0, 0.3, 1.0)[2], 1.0)
        self.assertEqual(limit_body(0, 0, -5.0, 0.3, 1.0)[2], -1.0)

    def test_a_request_inside_the_limits_is_untouched(self) -> None:
        self.assertEqual(limit_body(0.1, -0.1, 0.5, 0.3, 1.0), (0.1, -0.1, 0.5))


class WheelCommandTests(unittest.TestCase):
    def test_speeds_become_thousandths_of_full_power(self) -> None:
        self.assertEqual(wheel_commands((5.0, -5.0, 10.0, 0.0), 10.0), (500, -500, 1000, 0))

    def test_saturation_scales_all_wheels_together_to_keep_direction(self) -> None:
        commands = wheel_commands((20.0, 10.0, 20.0, 10.0), 10.0)

        self.assertEqual(commands, (1000, 500, 1000, 500))

    def test_never_outside_the_range_the_pico_accepts(self) -> None:
        for speeds in [(1e9, 0, 0, 0), (-1e9, 1e9, 0, 0)]:
            for command in wheel_commands(speeds, 10.0):
                self.assertLessEqual(abs(command), 1000)


class OdometryTests(unittest.TestCase):
    def test_driving_straight_moves_along_x(self) -> None:
        odometry = OdometryIntegrator(GEOMETRY)
        angle = 1.0 / 0.05  # 1 m of wheel travel

        vx, vy, wz = odometry.update([angle] * 4, dt=2.0)

        self.assertAlmostEqual(odometry.x, 1.0)
        self.assertAlmostEqual(odometry.y, 0.0)
        self.assertAlmostEqual(vx, 0.5)
        self.assertAlmostEqual(vy, 0.0)
        self.assertAlmostEqual(wz, 0.0)

    def test_a_quarter_turn_then_forward_moves_along_y(self) -> None:
        odometry = OdometryIntegrator(GEOMETRY)
        odometry.update(GEOMETRY.inverse(0.0, 0.0, math.pi / 2), dt=1.0)
        odometry.update(GEOMETRY.inverse(1.0, 0.0, 0.0), dt=1.0)

        self.assertAlmostEqual(odometry.theta, math.pi / 2)
        self.assertAlmostEqual(odometry.x, 0.0)
        self.assertAlmostEqual(odometry.y, 1.0)

    def test_heading_stays_within_plus_minus_pi(self) -> None:
        odometry = OdometryIntegrator(GEOMETRY)
        for _ in range(10):
            odometry.update(GEOMETRY.inverse(0.0, 0.0, 1.0), dt=1.0)

        self.assertLessEqual(abs(odometry.theta), math.pi)


if __name__ == "__main__":
    unittest.main()
