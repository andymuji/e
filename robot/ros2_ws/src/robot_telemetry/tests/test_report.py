"""The verdict and the words it is delivered in.

The exit code is what makes a recording evidence rather than a claim, so the
mapping from "what the run did" to "what a pull request is told" is tested as
carefully as the checks themselves.
"""

import unittest

from robot_telemetry.records import (
    CMD_VEL,
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
from robot_telemetry.report import Verdict, build_report

GATE = "/safety_controller"
GOOD_GRAPH = {CMD_VEL: [GATE]}


def run(*entries: tuple[float, str, object]) -> list[Record]:
    return [Record(when, topic, message) for when, topic, message in entries]


def report_for(records, publishers=GOOD_GRAPH):
    return build_report(
        records,
        source="test",
        stop_distance=0.45,
        publishers=publishers,
        gate_node=GATE,
    )


def good_run() -> list[Record]:
    """A robot that drove, saw a table, stopped, and was reset properly."""
    return run(
        (0.00, SAFETY_STATE, SafetyStatus.parse("clear: path clear")),
        (0.05, SCAN, Scan(2.0)),
        (0.05, CMD_VEL, Velocity(linear_x=0.2)),
        (0.10, SCAN, Scan(0.40)),
        (0.15, CMD_VEL, Velocity()),
        (0.15, SAFETY_STATE, SafetyStatus.parse("stop: obstacle inside stop distance")),
        (0.20, CMD_VEL, Velocity()),
        (1.00, EMERGENCY_STOP, Flag(True)),
        (1.05, SAFETY_STATE, SafetyStatus.parse("stop: emergency stop active")),
        (1.10, CMD_VEL, Velocity()),
        (2.00, EMERGENCY_STOP_RESET, Flag(True)),
        (2.05, SAFETY_STATE, SafetyStatus.parse("clear: path clear")),
        (2.10, CMD_VEL, Velocity(linear_x=0.2)),
    )


class VerdictTests(unittest.TestCase):
    def test_a_clean_run_passes(self):
        report = report_for(good_run())
        self.assertIs(report.verdict, Verdict.PASS)
        self.assertEqual(report.verdict.exit_code, 0)

    def test_a_violation_fails(self):
        records = good_run() + run(
            (3.00, SAFETY_STATE, SafetyStatus.parse("stop: emergency stop active")),
            # Well clear of the transition, so the settle window in
            # check_motion_only_when_permitted cannot excuse it.
            (3.50, CMD_VEL, Velocity(linear_x=0.3)),
            (3.55, CMD_VEL, Velocity(linear_x=0.3)),
        )
        report = report_for(records)
        self.assertIs(report.verdict, Verdict.FAIL)
        self.assertEqual(report.verdict.exit_code, 1)

    def test_a_second_publisher_fails_even_on_a_well_behaved_run(self):
        report = report_for(good_run(), {CMD_VEL: [GATE, "/rogue_teleop"]})
        self.assertIs(report.verdict, Verdict.FAIL)
        self.assertIn("/rogue_teleop", report.render())

    def test_an_empty_recording_is_inconclusive_not_a_pass(self):
        report = report_for([], publishers=None)
        self.assertIs(report.verdict, Verdict.INCONCLUSIVE)
        self.assertEqual(report.verdict.exit_code, 2)
        self.assertIn("cannot answer", report.render())

    def test_a_run_nobody_watched_the_graph_for_still_passes_what_it_can(self):
        """A missing topic graph must not hide a real violation either way.

        Recording without the probe leaves one question unanswerable; the
        other three were still answered, so the verdict is a qualified pass
        with the gap stated, not a blanket INCONCLUSIVE.
        """
        report = report_for(good_run(), publishers=None)
        self.assertIs(report.verdict, Verdict.PASS)
        self.assertIn("topic graph was not", report.render())


class RenderingTests(unittest.TestCase):
    def test_the_report_says_what_was_checked_and_what_it_means(self):
        rendered = report_for(good_run()).render()
        self.assertIn("WHAT WAS CHECKED", rendered)
        self.assertIn("What happened:", rendered)
        self.assertIn("What it means:", rendered)
        self.assertIn("VERDICT: PASS", rendered)

    def test_the_report_refuses_to_stand_in_for_the_physical_test(self):
        self.assertIn(
            "not a substitute for", report_for(good_run()).render()
        )

    def test_the_report_lists_every_transition_with_its_reason(self):
        rendered = report_for(good_run()).render()
        self.assertIn("obstacle inside stop distance", rendered)
        self.assertIn("emergency stop active", rendered)

    def test_the_report_gives_both_stopping_times(self):
        report = report_for(good_run())
        self.assertEqual(len(report.obstacle_stops), 1)
        self.assertEqual(len(report.emergency_stops), 1)
        self.assertIn("ms later", report.render())

    def test_a_recording_that_ends_mid_stop_says_so(self):
        records = run(
            (0.00, SAFETY_STATE, SafetyStatus.parse("clear: path clear")),
            (0.05, CMD_VEL, Velocity(linear_x=0.2)),
            (0.10, SAFETY_STATE, SafetyStatus.parse("stop: motion command timed out")),
            (0.15, CMD_VEL, Velocity()),
        )
        report = report_for(records)
        self.assertIs(report.verdict, Verdict.PASS)
        self.assertIn("ends while the gate was holding the robot stopped", report.render())
        self.assertIn("cannot vouch for what happened next", report.render())

    def test_a_run_with_nothing_to_stop_for_says_so_rather_than_claiming_zero(self):
        records = run(
            (0.00, SAFETY_STATE, SafetyStatus.parse("clear: path clear")),
            (0.05, CMD_VEL, Velocity(linear_x=0.2)),
            (0.10, CMD_VEL, Velocity(linear_x=0.2)),
        )
        self.assertIn(
            "Nothing in this run asked a moving robot to stop",
            report_for(records).render(),
        )


if __name__ == "__main__":
    unittest.main()
