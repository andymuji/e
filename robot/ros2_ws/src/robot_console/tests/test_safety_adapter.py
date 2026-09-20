import unittest

from robot_console.safety_adapter import RosSafetyAdapter, parse_state


class FakePublisher:
    def __init__(self) -> None:
        self.published = []

    def publish(self, message) -> None:
        self.published.append(message)


class Clock:
    def __init__(self) -> None:
        self.seconds = 100.0

    def __call__(self) -> float:
        return self.seconds


def adapter(timeout: float = 2.0):
    clock = Clock()
    stop, reset = FakePublisher(), FakePublisher()
    return (
        RosSafetyAdapter(stop, reset, lambda value: value, clock, timeout=timeout),
        clock,
        stop,
        reset,
    )


def alive(subject, clock, text: str = "clear: path clear") -> None:
    """A gate that has said something and is still running its control loop."""
    subject.on_state(text)
    subject.on_heartbeat()


class ParseStateTests(unittest.TestCase):
    def test_reads_the_gates_line(self) -> None:
        self.assertEqual(
            parse_state("caution: obstacle inside caution distance"),
            ("caution", "obstacle inside caution distance"),
        )

    def test_an_unreadable_line_is_unknown_not_a_guess(self) -> None:
        for text in ("", "clear", "running fine", "green: all good"):
            with self.subTest(text=text):
                state, _ = parse_state(text)

                self.assertEqual(state, "unknown")


class SafetyAdapterLatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.safety, self.clock, self.stop, self.reset = adapter()

    def test_engaging_publishes_true_on_emergency_stop(self) -> None:
        self.safety.engage_emergency_stop()

        self.assertEqual(self.stop.published, [True])
        self.assertEqual(self.reset.published, [])

    def test_releasing_publishes_true_on_the_reset_topic_only(self) -> None:
        # The rule this enforces: a False on emergency_stop never releases the
        # latch, so the console must never send one and call it a release.
        self.safety.reset_emergency_stop()

        self.assertEqual(self.reset.published, [True])
        self.assertEqual(self.stop.published, [])

    def test_the_console_never_publishes_false_on_either_topic(self) -> None:
        self.safety.engage_emergency_stop()
        self.safety.reset_emergency_stop()

        self.assertNotIn(False, self.stop.published)
        self.assertNotIn(False, self.reset.published)


class SafetyAdapterStatusTests(unittest.TestCase):
    def setUp(self) -> None:
        self.safety, self.clock, self.stop, self.reset = adapter()

    def test_before_the_gate_has_spoken_everything_is_unknown(self) -> None:
        status = self.safety.status()

        self.assertEqual(status["state"], "unknown")
        self.assertIsNone(status["emergency_stop"])
        self.assertTrue(self.safety.emergency_stop_engaged)
        self.assertIsNotNone(self.safety.motion_refusal())

    def test_a_gate_that_is_absent_reads_differently_from_one_that_is_quiet(
        self,
    ) -> None:
        # Both are unknown, and both refuse motion. They are told apart
        # because a console opened after the robot settled sees the second
        # one - safety_state is published on change - and the operator needs
        # to know the robot is there rather than that it is missing.
        absent = self.safety.status()["reason"]

        self.safety.on_heartbeat()
        quiet = self.safety.status()

        self.assertNotEqual(quiet["reason"], absent)
        self.assertEqual(quiet["state"], "unknown")
        self.assertIsNone(quiet["emergency_stop"])
        self.assertTrue(self.safety.emergency_stop_engaged)

    def test_a_running_gate_is_reported_as_it_reported_itself(self) -> None:
        alive(self.safety, self.clock, "caution: obstacle inside caution distance")

        status = self.safety.status()
        self.assertEqual(status["state"], "caution")
        self.assertEqual(status["reason"], "obstacle inside caution distance")
        self.assertFalse(status["emergency_stop"])
        self.assertIsNone(self.safety.motion_refusal())

    def test_the_latch_is_read_from_the_gates_own_reason(self) -> None:
        alive(self.safety, self.clock, "stop: emergency stop active")

        self.assertTrue(self.safety.status()["emergency_stop"])
        self.assertTrue(self.safety.emergency_stop_engaged)
        self.assertEqual(self.safety.motion_refusal(), "the emergency stop is engaged")

    def test_a_stop_for_another_reason_is_not_the_latch(self) -> None:
        alive(self.safety, self.clock, "stop: obstacle inside stop distance")

        status = self.safety.status()
        self.assertEqual(status["state"], "stop")
        self.assertFalse(status["emergency_stop"])

    def test_a_silent_gate_goes_unknown_rather_than_staying_clear(self) -> None:
        # The failure this guards: the gate dies while the light is green and
        # the console keeps showing green.
        alive(self.safety, self.clock)
        self.assertEqual(self.safety.status()["state"], "clear")

        self.clock.seconds += 5.0

        status = self.safety.status()
        self.assertEqual(status["state"], "unknown")
        self.assertIsNone(status["emergency_stop"])
        self.assertTrue(self.safety.emergency_stop_engaged)

    def test_the_heartbeat_keeps_a_settled_gate_fresh(self) -> None:
        # safety_state is published on change, so a robot standing in a clear
        # room is silent on it for minutes. The gate's control loop is what
        # says it is alive.
        alive(self.safety, self.clock)
        for _ in range(10):
            self.clock.seconds += 1.0
            self.safety.on_heartbeat()

        self.assertEqual(self.safety.status()["state"], "clear")

    def test_a_backwards_clock_is_not_a_fresh_reading(self) -> None:
        alive(self.safety, self.clock)
        self.clock.seconds -= 30.0

        self.assertEqual(self.safety.status()["state"], "unknown")

    def test_an_unreadable_state_refuses_motion(self) -> None:
        alive(self.safety, self.clock, "everything is fine")

        status = self.safety.status()
        self.assertEqual(status["state"], "unknown")
        self.assertIsNone(status["emergency_stop"])
        self.assertTrue(self.safety.emergency_stop_engaged)

    def test_a_timeout_of_zero_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            RosSafetyAdapter(
                FakePublisher(), FakePublisher(), bool, Clock(), timeout=0.0
            )


if __name__ == "__main__":
    unittest.main()
