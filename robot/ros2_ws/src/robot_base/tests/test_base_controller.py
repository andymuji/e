import math
import unittest

from robot_base import BaseController, DriveLimits, MecanumGeometry
from robot_base.base_controller import STOPPED, wheel_topic_problem

GATE = "safety_controller"


def controller(timeout: float = 0.25) -> BaseController:
    return BaseController(
        MecanumGeometry(wheel_radius=0.05, half_wheelbase=0.1, half_track=0.1),
        DriveLimits(max_linear_speed=0.3, max_angular_speed=1.0, max_wheel_speed=10.0),
        command_timeout=timeout,
    )


class StaleCommandTests(unittest.TestCase):
    def test_nothing_moves_before_the_first_command(self) -> None:
        decision = controller().decide(now=100.0)

        self.assertEqual(decision.commands, STOPPED)
        self.assertEqual(decision.reason, "no motion command received")

    def test_a_fresh_command_drives(self) -> None:
        base = controller()
        base.on_command(0.2, 0.0, 0.0, now=100.0)

        decision = base.decide(now=100.1)

        self.assertEqual(decision.commands, (400, 400, 400, 400))
        self.assertEqual(decision.reason, "driving")

    def test_a_stale_command_becomes_zero(self) -> None:
        # The gate publishes every cycle, so silence means it has died.
        base = controller()
        base.on_command(0.2, 0.0, 0.0, now=100.0)

        decision = base.decide(now=100.26)

        self.assertEqual(decision.commands, STOPPED)
        self.assertEqual(decision.reason, "motion command timed out")

    def test_a_stale_command_is_not_replayed_if_the_clock_steps_back(self) -> None:
        base = controller()
        base.on_command(0.2, 0.0, 0.0, now=100.0)
        base.decide(now=101.0)

        self.assertEqual(base.decide(now=100.1).commands, STOPPED)

    def test_a_command_from_the_future_is_not_trusted(self) -> None:
        base = controller()
        base.on_command(0.2, 0.0, 0.0, now=100.0)

        self.assertEqual(base.decide(now=99.0).commands, STOPPED)

    def test_a_command_that_is_not_a_number_stops_rather_than_keeps_going(
        self,
    ) -> None:
        base = controller()
        base.on_command(0.2, 0.0, 0.0, now=100.0)
        base.on_command(math.nan, 0.0, 0.0, now=100.05)

        decision = base.decide(now=100.1)

        self.assertEqual(decision.commands, STOPPED)
        self.assertEqual(decision.reason, "invalid motion command")

    def test_the_gate_saying_zero_stops_the_wheels(self) -> None:
        base = controller()
        base.on_command(0.0, 0.0, 0.0, now=100.0)

        self.assertEqual(base.decide(now=100.0).commands, STOPPED)

    def test_the_driver_caps_speed_whatever_arrives(self) -> None:
        base = controller()
        base.on_command(5.0, 0.0, 0.0, now=100.0)

        # 0.3 m/s cap on 0.05 m wheels is 6 rad/s, of 10 at full power.
        self.assertEqual(base.decide(now=100.0).commands, (600, 600, 600, 600))


class TankAndRampTests(unittest.TestCase):
    def ramped(self, max_step: int = 100, allow_sideways: bool = True) -> BaseController:
        return BaseController(
            MecanumGeometry(wheel_radius=0.05, half_wheelbase=0.1, half_track=0.1),
            DriveLimits(0.3, 1.0, 10.0),
            command_timeout=0.25,
            allow_sideways=allow_sideways,
            max_step=max_step,
        )

    def test_tank_mode_drops_sideways_requests(self) -> None:
        base = self.ramped(max_step=None, allow_sideways=False)
        base.on_command(0.0, 0.3, 0.0, now=0.0)

        self.assertEqual(base.decide(now=0.0).commands, STOPPED)

    def test_power_rises_no_faster_than_the_step(self) -> None:
        base = self.ramped()
        base.on_command(0.3, 0.0, 0.0, now=0.0)

        seen = [base.decide(now=0.01 * i).commands[0] for i in range(8)]

        self.assertEqual(seen, [100, 200, 300, 400, 500, 600, 600, 600])

    def test_a_stop_is_never_ramped(self) -> None:
        base = self.ramped()
        base.on_command(0.3, 0.0, 0.0, now=0.0)
        for i in range(6):
            base.decide(now=0.01 * i)

        # Stale: zero at once, not 500, 400, ...
        self.assertEqual(base.decide(now=1.0).commands, STOPPED)

    def test_slowing_down_is_immediate(self) -> None:
        base = self.ramped()
        base.on_command(0.3, 0.0, 0.0, now=0.0)
        for i in range(6):
            base.decide(now=0.01 * i)
        base.on_command(0.05, 0.0, 0.0, now=0.1)

        self.assertEqual(base.decide(now=0.1).commands[0], 100)

    def test_a_reversal_passes_through_rest(self) -> None:
        base = self.ramped()
        base.on_command(0.3, 0.0, 0.0, now=0.0)
        for i in range(6):
            base.decide(now=0.01 * i)
        base.on_command(-0.3, 0.0, 0.0, now=0.1)

        self.assertEqual(base.decide(now=0.1).commands[0], 0)
        self.assertEqual(base.decide(now=0.11).commands[0], -100)


class WheelTopicTests(unittest.TestCase):
    def test_the_gate_alone_is_trusted(self) -> None:
        self.assertIsNone(wheel_topic_problem([GATE], GATE))

    def test_no_publisher_means_no_gate(self) -> None:
        self.assertIn("not running", wheel_topic_problem([], GATE))

    def test_a_teleop_left_on_the_wheel_topic_is_named(self) -> None:
        problem = wheel_topic_problem([GATE, "teleop_twist_keyboard"], GATE)

        self.assertIn("teleop_twist_keyboard", problem)

    def test_two_gates_are_refused(self) -> None:
        # Observed on a live run: two gates on /cmd_vel, one of them without
        # the checks the other was trying to enforce.
        self.assertIn("2 safety gates", wheel_topic_problem([GATE, GATE], GATE))

    def test_a_topic_problem_overrides_a_fresh_command(self) -> None:
        base = controller()
        base.on_command(0.2, 0.0, 0.0, now=100.0)

        decision = base.decide(now=100.0, topic_problem="someone else")

        self.assertEqual(decision.commands, STOPPED)
        self.assertEqual(decision.reason, "someone else")


class LimitValidationTests(unittest.TestCase):
    def test_nonsense_limits_are_refused(self) -> None:
        with self.assertRaises(ValueError):
            DriveLimits(0.0, 1.0, 10.0)
        with self.assertRaises(ValueError):
            DriveLimits(0.3, math.inf, 10.0)
        with self.assertRaises(ValueError):
            controller(timeout=0.0)


if __name__ == "__main__":
    unittest.main()
