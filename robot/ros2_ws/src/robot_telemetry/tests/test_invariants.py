"""The safety checks, tested against runs no robot was harmed producing.

This is the whole reason the analyser is split the way it is. A gate that is
bypassed, a latch that lets go on its own, and a robot that keeps rolling after
a stop are the three things that must never happen on the hardware - so they
can only be tested here, by handing the checks a recording of them.
"""

import unittest

from robot_telemetry.invariants import (
    Outcome,
    check_latch_released_only_by_reset,
    check_motion_only_when_permitted,
    check_run_is_evidence,
    check_sole_cmd_vel_publisher,
    emergency_stop_latencies,
    ended_mid_stop,
    obstacle_stop_latencies,
    safety_transitions,
)
from robot_telemetry.records import (
    CMD_VEL,
    CMD_VEL_REQUESTED,
    EMERGENCY_STOP,
    EMERGENCY_STOP_RESET,
    SAFETY_STATE,
    SCAN,
    Flag,
    Record,
    SafetyStatus,
    Scan,
    Velocity,
)

GATE = "/safety_controller"


def moving(speed: float = 0.2) -> Velocity:
    return Velocity(linear_x=speed)


def stopped() -> Velocity:
    return Velocity()


def state(text: str) -> SafetyStatus:
    return SafetyStatus.parse(text)


def run(*entries: tuple[float, str, object]) -> list[Record]:
    """A recording, written as (when, which topic, what)."""
    return [Record(when, topic, message) for when, topic, message in entries]


def clear_drive(start: float = 0.0, ticks: int = 5) -> list[Record]:
    """A stretch of ordinary driving: clear path, non-zero velocity."""
    records = [Record(start, SAFETY_STATE, state("clear: path clear"))]
    for tick in range(ticks):
        when = start + 0.05 * (tick + 1)
        records.append(Record(when, SCAN, Scan(3.0)))
        records.append(Record(when, CMD_VEL_REQUESTED, moving()))
        records.append(Record(when, CMD_VEL, moving()))
    return records


class MotionWhileStoppedTests(unittest.TestCase):
    def test_a_gate_that_stayed_stopped_passes(self):
        records = clear_drive() + run(
            (1.0, SAFETY_STATE, state("stop: obstacle inside stop distance")),
            (1.05, CMD_VEL, stopped()),
            (1.10, CMD_VEL, stopped()),
            (1.15, CMD_VEL, stopped()),
        )
        finding = check_motion_only_when_permitted(records)
        self.assertIs(finding.outcome, Outcome.PASSED)
        self.assertIn("3 velocity commands", finding.detail)

    def test_motion_after_a_stop_state_fails(self):
        records = clear_drive() + run(
            (1.0, SAFETY_STATE, state("stop: obstacle inside stop distance")),
            (1.05, CMD_VEL, stopped()),
            # Something other than the gate put a velocity on the wire.
            (1.10, CMD_VEL, moving(0.4)),
        )
        finding = check_motion_only_when_permitted(records)
        self.assertIs(finding.outcome, Outcome.FAILED)
        self.assertIn("1 occurrence", finding.detail)
        self.assertIn("path around the gate", finding.meaning)

    def test_the_last_moving_command_before_a_stop_is_not_an_offence(self):
        """The gate publishes the velocity first and the new state after it.

        A naive check that looked forward from a stop state would call that
        ordering a violation on every single stop, and a report that cries
        wolf on every run is a report nobody reads.
        """
        records = run(
            (0.00, SAFETY_STATE, state("clear: path clear")),
            (0.05, CMD_VEL, moving()),
            (0.10, CMD_VEL, moving()),
            (0.15, SAFETY_STATE, state("stop: obstacle inside stop distance")),
            (0.20, CMD_VEL, stopped()),
        )
        self.assertIs(
            check_motion_only_when_permitted(records).outcome, Outcome.PASSED
        )

    def test_a_velocity_recorded_a_hair_before_the_release_is_not_an_offence(self):
        """The case a real recording found, twice in twenty seconds.

        On releasing a stop the gate publishes the new velocity and then the
        new state, from one callback. The recorder timestamps them as it
        receives them, and the order inverts by microseconds - so without the
        settle window a correct gate is accused at every single release.
        """
        records = run(
            (0.00, SAFETY_STATE, state("stop: obstacle inside stop distance")),
            (0.05, CMD_VEL, stopped()),
            (0.10, CMD_VEL, stopped()),
            # One callback: velocity first, state second, recorded inverted.
            (0.149983, CMD_VEL, moving()),
            (0.150000, SAFETY_STATE, state("clear: path clear")),
            (0.20, CMD_VEL, moving()),
        )
        finding = check_motion_only_when_permitted(records)
        self.assertIs(finding.outcome, Outcome.PASSED)
        self.assertIn("cannot tell which of the two", finding.detail)

    def test_a_velocity_well_inside_a_stop_is_still_an_offence(self):
        """The window forgives a boundary, not a sustained bypass."""
        records = run(
            (0.00, SAFETY_STATE, state("stop: emergency stop active")),
            (0.50, CMD_VEL, moving()),
            (0.55, CMD_VEL, moving()),
            (5.00, SAFETY_STATE, state("clear: path clear")),
        )
        finding = check_motion_only_when_permitted(records)
        self.assertIs(finding.outcome, Outcome.FAILED)
        self.assertIn("2 occurrence", finding.detail)

    def test_a_run_that_never_stopped_is_not_a_pass(self):
        finding = check_motion_only_when_permitted(clear_drive())
        self.assertIs(finding.outcome, Outcome.NOT_EXERCISED)

    def test_a_nan_velocity_is_not_read_as_stopped(self):
        records = run(
            (0.0, SAFETY_STATE, state("stop: emergency stop active")),
            (0.1, CMD_VEL, Velocity(linear_x=float("nan"))),
        )
        self.assertIs(
            check_motion_only_when_permitted(records).outcome, Outcome.FAILED
        )


class LatchTests(unittest.TestCase):
    def test_a_release_after_a_reset_passes(self):
        records = run(
            (0.0, SAFETY_STATE, state("clear: path clear")),
            (1.0, EMERGENCY_STOP, Flag(True)),
            (1.05, SAFETY_STATE, state("stop: emergency stop active")),
            (5.0, EMERGENCY_STOP_RESET, Flag(True)),
            (5.05, SAFETY_STATE, state("clear: path clear")),
        )
        finding = check_latch_released_only_by_reset(records)
        self.assertIs(finding.outcome, Outcome.PASSED)
        self.assertIn("released 1 time", finding.detail)

    def test_a_release_without_a_reset_fails(self):
        records = run(
            (1.0, EMERGENCY_STOP, Flag(True)),
            (1.05, SAFETY_STATE, state("stop: emergency stop active")),
            (5.05, SAFETY_STATE, state("clear: path clear")),
        )
        finding = check_latch_released_only_by_reset(records)
        self.assertIs(finding.outcome, Outcome.FAILED)
        self.assertIn("no reset message in between", finding.detail)

    def test_a_false_on_the_stop_topic_does_not_count_as_a_reset(self):
        """Precedence: only `emergency_stop_reset` releases the latch.

        A `false` on `emergency_stop` is the mistake the gate is written to
        survive, so a report that accepted it as a release would bless exactly
        the wrong behaviour.
        """
        records = run(
            (1.0, EMERGENCY_STOP, Flag(True)),
            (1.05, SAFETY_STATE, state("stop: emergency stop active")),
            (4.0, EMERGENCY_STOP, Flag(False)),
            (5.05, SAFETY_STATE, state("clear: path clear")),
        )
        self.assertIs(
            check_latch_released_only_by_reset(records).outcome, Outcome.FAILED
        )

    def test_a_reset_before_the_latch_does_not_release_a_later_one(self):
        records = run(
            (0.5, EMERGENCY_STOP_RESET, Flag(True)),
            (1.0, EMERGENCY_STOP, Flag(True)),
            (1.05, SAFETY_STATE, state("stop: emergency stop active")),
            (5.05, SAFETY_STATE, state("clear: path clear")),
        )
        self.assertIs(
            check_latch_released_only_by_reset(records).outcome, Outcome.FAILED
        )

    def test_still_latched_at_the_end_passes(self):
        records = run(
            (1.0, EMERGENCY_STOP, Flag(True)),
            (1.05, SAFETY_STATE, state("stop: emergency stop active")),
        )
        finding = check_latch_released_only_by_reset(records)
        self.assertIs(finding.outcome, Outcome.PASSED)
        self.assertIn("still engaged", finding.detail)

    def test_a_run_without_an_emergency_stop_is_not_a_pass(self):
        self.assertIs(
            check_latch_released_only_by_reset(clear_drive()).outcome,
            Outcome.NOT_EXERCISED,
        )

    def test_a_stale_sensor_stop_is_not_mistaken_for_the_latch(self):
        """Only the emergency stop latches. An ordinary stop may clear itself.

        A stale scan stops the robot and releases on its own the moment scans
        come back, with no operator reset anywhere - correct behaviour that a
        check watching for "any stop that ended" would report as a violation.
        """
        records = run(
            (0.0, SAFETY_STATE, state("clear: path clear")),
            (1.0, SAFETY_STATE, state("stop: obstacle sensor timed out")),
            (2.0, SAFETY_STATE, state("clear: path clear")),
        )
        self.assertIs(
            check_latch_released_only_by_reset(records).outcome,
            Outcome.NOT_EXERCISED,
        )


class PublisherTests(unittest.TestCase):
    def test_the_gate_alone_passes(self):
        finding = check_sole_cmd_vel_publisher({CMD_VEL: [GATE]}, GATE)
        self.assertIs(finding.outcome, Outcome.PASSED)

    def test_a_second_publisher_fails(self):
        finding = check_sole_cmd_vel_publisher(
            {CMD_VEL: [GATE, "/joystick_driver"]}, GATE
        )
        self.assertIs(finding.outcome, Outcome.FAILED)
        self.assertIn("/joystick_driver", finding.detail)
        self.assertIn("cmd_vel_requested", finding.meaning)

    def test_no_recorded_graph_is_not_a_pass(self):
        finding = check_sole_cmd_vel_publisher(None, GATE)
        self.assertIs(finding.outcome, Outcome.NOT_EXERCISED)

    def test_nobody_publishing_is_not_a_pass(self):
        finding = check_sole_cmd_vel_publisher({CMD_VEL: []}, GATE)
        self.assertIs(finding.outcome, Outcome.NOT_EXERCISED)


class LatencyTests(unittest.TestCase):
    def test_obstacle_stop_latency_is_measured_from_the_first_close_scan(self):
        records = run(
            (0.00, SAFETY_STATE, state("clear: path clear")),
            (0.00, CMD_VEL, moving()),
            (0.10, SCAN, Scan(1.2)),
            (0.20, SCAN, Scan(0.40)),
            (0.25, CMD_VEL, moving()),
            (0.30, CMD_VEL, stopped()),
        )
        latencies = obstacle_stop_latencies(records, stop_distance=0.45)
        self.assertEqual(len(latencies), 1)
        self.assertAlmostEqual(latencies[0].seconds, 0.10)

    def test_an_obstacle_seen_by_a_standing_robot_is_not_a_measurement(self):
        records = run(
            (0.00, CMD_VEL, stopped()),
            (0.20, SCAN, Scan(0.40)),
            (0.30, CMD_VEL, stopped()),
        )
        self.assertEqual(obstacle_stop_latencies(records, 0.45), [])

    def test_one_approach_is_reported_once(self):
        records = run(
            (0.00, CMD_VEL, moving()),
            (0.10, SCAN, Scan(0.40)),
            (0.20, SCAN, Scan(0.35)),
            (0.30, SCAN, Scan(0.30)),
            (0.40, CMD_VEL, stopped()),
        )
        self.assertEqual(len(obstacle_stop_latencies(records, 0.45)), 1)

    def test_a_stop_that_never_arrived_is_reported_without_a_time(self):
        records = run(
            (0.00, CMD_VEL, moving()),
            (0.10, SCAN, Scan(0.40)),
            (0.20, CMD_VEL, moving()),
        )
        latencies = obstacle_stop_latencies(records, 0.45)
        self.assertEqual(len(latencies), 1)
        self.assertIsNone(latencies[0].seconds)

    def test_emergency_stop_latency(self):
        records = run(
            (0.00, CMD_VEL, moving()),
            (1.00, EMERGENCY_STOP, Flag(True)),
            (1.04, CMD_VEL, moving()),
            (1.09, CMD_VEL, stopped()),
        )
        latencies = emergency_stop_latencies(records)
        self.assertEqual(len(latencies), 1)
        self.assertAlmostEqual(latencies[0].seconds, 0.09)
        self.assertFalse(latencies[0].already_stopped)

    def test_an_emergency_stop_on_a_standing_robot_is_flagged_as_such(self):
        records = run(
            (0.00, CMD_VEL, stopped()),
            (1.00, EMERGENCY_STOP, Flag(True)),
            (1.05, CMD_VEL, stopped()),
        )
        self.assertTrue(emergency_stop_latencies(records)[0].already_stopped)

    def test_a_false_on_the_stop_topic_is_not_a_stop_event(self):
        records = run(
            (0.00, CMD_VEL, moving()),
            (1.00, EMERGENCY_STOP, Flag(False)),
        )
        self.assertEqual(emergency_stop_latencies(records), [])


class TransitionTests(unittest.TestCase):
    def test_transitions_carry_reason_and_time(self):
        records = run(
            (0.0, SAFETY_STATE, state("clear: path clear")),
            (1.0, SAFETY_STATE, state("caution: obstacle inside caution distance")),
            (2.0, SAFETY_STATE, state("stop: obstacle inside stop distance")),
        )
        transitions = safety_transitions(records)
        self.assertEqual(
            [(t.timestamp, t.status.state) for t in transitions],
            [(0.0, "clear"), (1.0, "caution"), (2.0, "stop")],
        )
        self.assertEqual(transitions[2].status.reason, "obstacle inside stop distance")
        self.assertEqual(transitions[2].previous.state, "caution")

    def test_a_repeated_status_is_not_a_transition(self):
        records = run(
            (0.0, SAFETY_STATE, state("clear: path clear")),
            (1.0, SAFETY_STATE, state("clear: path clear")),
        )
        self.assertEqual(len(safety_transitions(records)), 1)


class EvidenceTests(unittest.TestCase):
    def test_an_empty_recording_is_not_a_pass(self):
        finding = check_run_is_evidence([])
        self.assertIs(finding.outcome, Outcome.NOT_EXERCISED)
        self.assertIn("empty", finding.detail)

    def test_a_recording_without_the_gate_is_not_a_pass(self):
        records = run((0.0, SCAN, Scan(2.0)), (0.1, SCAN, Scan(2.0)))
        finding = check_run_is_evidence(records)
        self.assertIs(finding.outcome, Outcome.NOT_EXERCISED)

    def test_a_real_run_is_evidence(self):
        self.assertIs(check_run_is_evidence(clear_drive()).outcome, Outcome.PASSED)

    def test_a_recording_that_ends_mid_stop_is_flagged(self):
        records = clear_drive() + run(
            (1.0, SAFETY_STATE, state("stop: obstacle inside stop distance")),
            (1.05, CMD_VEL, stopped()),
        )
        unfinished = ended_mid_stop(records)
        self.assertIsNotNone(unfinished)
        self.assertEqual(unfinished.state, "stop")

    def test_a_recording_that_ends_clear_is_not_flagged(self):
        self.assertIsNone(ended_mid_stop(clear_drive()))


if __name__ == "__main__":
    unittest.main()
