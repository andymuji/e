"""Turn a recorded run into a safety report a non-programmer can read.

The report has one job: replace "we tested it and it stopped" in a pull
request with a file that says what was checked, what happened, and what it
means - and that disagrees out loud when the run did not behave.
"""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
import textwrap

from .invariants import (
    Finding,
    Latency,
    Outcome,
    Transition,
    check_latch_released_only_by_reset,
    check_motion_only_when_permitted,
    check_run_is_evidence,
    check_sole_cmd_vel_publisher,
    emergency_stop_latencies,
    ended_mid_stop,
    obstacle_stop_latencies,
    safety_transitions,
)
from .records import Record

# The gate's node name in every launch file that starts it.
DEFAULT_GATE_NODE = "/safety_controller"

WIDTH = 78


class Verdict(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    INCONCLUSIVE = "INCONCLUSIVE"

    @property
    def exit_code(self) -> int:
        """0 only for a clean run. INCONCLUSIVE is not success.

        A recording that could not answer the questions must not be able to
        turn a pull request green, so it gets its own non-zero code rather
        than being folded into either of the other two.
        """
        return {Verdict.PASS: 0, Verdict.FAIL: 1, Verdict.INCONCLUSIVE: 2}[self]


@dataclass(frozen=True)
class SafetyReport:
    source: str
    findings: tuple[Finding, ...]
    transitions: tuple[Transition, ...] = ()
    obstacle_stops: tuple[Latency, ...] = ()
    emergency_stops: tuple[Latency, ...] = ()
    notes: tuple[str, ...] = ()
    stop_distance: float = 0.0
    message_count: int = 0
    duration: float = 0.0
    topics: Mapping[str, int] = field(default_factory=dict)

    @property
    def verdict(self) -> Verdict:
        if any(finding.failed for finding in self.findings):
            return Verdict.FAIL
        if all(
            finding.outcome is Outcome.NOT_EXERCISED for finding in self.findings
        ):
            return Verdict.INCONCLUSIVE
        return Verdict.PASS

    def render(self) -> str:
        return "\n".join(self._lines())

    def _lines(self) -> list[str]:
        lines = [
            "=" * WIDTH,
            "SAFETY REPORT FOR ONE RECORDED RUN",
            "=" * WIDTH,
            f"Recording: {self.source}",
            f"Contents:  {self.message_count} messages over "
            f"{self.duration:.1f} seconds",
            f"Judged against a stop distance of {self.stop_distance:.2f} m",
            "",
            f"VERDICT: {self.verdict.value}  -  {_verdict_meaning(self.verdict)}",
            "",
            "This report is about this one recording. It is not a substitute for",
            "the physical emergency-stop test in docs/safety-test-procedure.md.",
            "",
            "-" * WIDTH,
            "WHAT WAS CHECKED",
            "-" * WIDTH,
        ]
        for finding in self.findings:
            lines.append("")
            lines.append(f"[{finding.outcome.value}] {finding.check}")
            lines.extend(
                _wrap(f"What happened: {finding.detail}", "  ", "                 ")
            )
            lines.extend(
                _wrap(f"What it means: {finding.meaning}", "  ", "                 ")
            )

        lines += ["", "-" * WIDTH, "HOW LONG THE ROBOT TOOK TO STOP", "-" * WIDTH]
        lines.extend(self._latency_lines())

        lines += ["", "-" * WIDTH, "WHAT THE SAFETY GATE DID, IN ORDER", "-" * WIDTH]
        lines.extend(self._transition_lines())

        if self.notes:
            lines += ["", "-" * WIDTH, "WORTH KNOWING", "-" * WIDTH]
            for note in self.notes:
                lines.append("")
                lines.extend(_wrap(f"- {note}", "  ", "    "))

        lines += ["", "=" * WIDTH]
        return lines

    def _latency_lines(self) -> list[str]:
        lines: list[str] = []
        if not self.obstacle_stops and not self.emergency_stops:
            lines.append("")
            lines.extend(
                _wrap(
                    "Nothing in this run asked a moving robot to stop, so there "
                    "is no stopping time to report. An obstacle inside the stop "
                    "distance while the robot was standing still does not count."
                )
            )
            return lines

        start = self.transitions[0].timestamp if self.transitions else 0.0
        for latency in self.obstacle_stops + self.emergency_stops:
            lines.append("")
            offset = latency.trigger_time - start
            lines.extend(
                _wrap(f"- {latency.trigger}, at {offset:+.3f} s", "  ", "    ")
            )
            if latency.seconds is None:
                outcome = (
                    "The robot was never commanded to zero afterwards in this "
                    "recording. Investigate before any further test."
                )
            elif latency.already_stopped:
                outcome = (
                    f"Zero was commanded {latency.seconds * 1000:.0f} ms later, "
                    "but the robot was already stopped, so this is not a "
                    "measurement of stopping time."
                )
            else:
                outcome = (
                    f"Zero speed was commanded {latency.seconds * 1000:.0f} ms "
                    "later."
                )
            lines.extend(_wrap(outcome, "    "))
        return lines

    def _transition_lines(self) -> list[str]:
        if not self.transitions:
            return ["", "The safety gate did not report a state at all."]
        start = self.transitions[0].timestamp
        lines = [""]
        for transition in self.transitions:
            offset = transition.timestamp - start
            lines.append(
                f"  {offset:8.3f} s  {transition.status.state:<8} "
                f"{transition.status.reason}"
            )
        return lines


def build_report(
    records: Sequence[Record],
    *,
    source: str,
    stop_distance: float,
    publishers: Mapping[str, Sequence[str]] | None = None,
    gate_node: str = DEFAULT_GATE_NODE,
) -> SafetyReport:
    """Answer every question about a run, in one pass with no ROS in sight."""
    ordered = sorted(records, key=lambda record: record.timestamp)
    findings = (
        check_run_is_evidence(ordered),
        check_sole_cmd_vel_publisher(publishers, gate_node),
        check_motion_only_when_permitted(ordered),
        check_latch_released_only_by_reset(ordered),
    )

    notes = []
    unfinished = ended_mid_stop(ordered)
    if unfinished is not None:
        notes.append(
            f'The recording ends while the gate was holding the robot stopped '
            f'("{unfinished}"). Nothing in this file shows that stop being '
            "released, so the recording cannot vouch for what happened next."
        )
    if any(finding.outcome is Outcome.NOT_EXERCISED for finding in findings):
        notes.append(
            "One or more checks were not exercised by this run. A check that "
            "was never put to the test is not a check that passed."
        )

    topics: dict[str, int] = {}
    for record in ordered:
        topics[record.topic] = topics.get(record.topic, 0) + 1

    return SafetyReport(
        source=source,
        findings=findings,
        transitions=tuple(safety_transitions(ordered)),
        obstacle_stops=tuple(obstacle_stop_latencies(ordered, stop_distance)),
        emergency_stops=tuple(emergency_stop_latencies(ordered)),
        notes=tuple(notes),
        stop_distance=stop_distance,
        message_count=len(ordered),
        duration=(
            ordered[-1].timestamp - ordered[0].timestamp if ordered else 0.0
        ),
        topics=topics,
    )


def _verdict_meaning(verdict: Verdict) -> str:
    return {
        Verdict.PASS: "this recording broke none of the safety rules checked below",
        Verdict.FAIL: "this recording broke a safety rule; read the FAILED entries",
        Verdict.INCONCLUSIVE: "this recording cannot answer any of the questions",
    }[verdict]


def _wrap(text: str, indent: str = "  ", hanging: str | None = None) -> list[str]:
    """Wrap a paragraph to the report width.

    `hanging` indents the continuation lines further, so a wrapped bullet
    still reads as one bullet rather than as two.
    """
    return textwrap.wrap(
        text.strip(),
        width=WIDTH,
        initial_indent=indent,
        subsequent_indent=hanging if hanging is not None else indent,
    ) or [indent]
