"""The questions asked of a recorded run, as plain functions over records.

Nothing here imports ROS. Each check takes the sequence of `Record`s the
adapter produced and answers one question about the run, so the answers can be
tested against handwritten runs that no robot ever produced - including the
failures a real robot must never be driven into on purpose, like a second node
publishing velocity behind the gate's back.

A check reports `NOT_EXERCISED` when the run never put it to the test. That is
deliberately not a pass: a recording in which the emergency stop was never
pressed proves nothing about the emergency stop, and a report that called it
green would be worse than no report.
"""

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from enum import Enum

from .records import (
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
    canonical_topic,
)

# How many offending moments a finding quotes before it stops listing them.
# The point is to show the reader what happened, not to reprint the bag.
MAX_EXAMPLES = 5

# How close to a change of safety state a velocity may be before the recording
# stops being able to say which came first. One control period of the gate at
# its configured 20 Hz, which is the longest two messages sent from the same
# callback can plausibly be separated by in a bag. See
# check_motion_only_when_permitted.
BOUNDARY_SETTLE = 0.06


class Outcome(str, Enum):
    PASSED = "PASSED"
    FAILED = "FAILED"
    NOT_EXERCISED = "NOT EXERCISED"


@dataclass(frozen=True)
class Finding:
    """One answered question, written for someone who does not read code."""

    check: str
    outcome: Outcome
    detail: str
    meaning: str

    @property
    def failed(self) -> bool:
        return self.outcome is Outcome.FAILED


@dataclass(frozen=True)
class Transition:
    """A moment the gate changed its mind about whether the robot may move."""

    timestamp: float
    status: SafetyStatus
    previous: SafetyStatus | None


@dataclass(frozen=True)
class Latency:
    """How long the robot took to stop after something said it must."""

    trigger: str
    trigger_time: float
    stopped_time: float | None
    already_stopped: bool = False

    @property
    def seconds(self) -> float | None:
        if self.stopped_time is None:
            return None
        return self.stopped_time - self.trigger_time


def _on(records: Iterable[Record], topic: str) -> list[Record]:
    """Records from one topic, in time order."""
    wanted = canonical_topic(topic)
    return sorted(
        (record for record in records if canonical_topic(record.topic) == wanted),
        key=lambda record: record.timestamp,
    )


def _velocity_at_or_before(
    velocities: Sequence[Record], timestamp: float
) -> Velocity | None:
    """The last velocity the gate had published when `timestamp` happened."""
    latest = None
    for record in velocities:
        if record.timestamp > timestamp:
            break
        if isinstance(record.message, Velocity):
            latest = record.message
    return latest


def _first_zero_after(
    velocities: Sequence[Record], timestamp: float
) -> float | None:
    """When the gate first commanded zero at or after `timestamp`."""
    for record in velocities:
        if record.timestamp < timestamp:
            continue
        if isinstance(record.message, Velocity) and record.message.is_zero():
            return record.timestamp
    return None


def safety_transitions(records: Iterable[Record]) -> list[Transition]:
    """Every change of safety state, with its reason and its timestamp.

    The gate publishes `/safety_state` only when the status changes, so each
    recorded message is already a transition; repeats are still collapsed, in
    case a future gate publishes it on a timer instead.
    """
    transitions: list[Transition] = []
    previous: SafetyStatus | None = None
    for record in _on(records, SAFETY_STATE):
        status = record.message
        if not isinstance(status, SafetyStatus) or status == previous:
            continue
        transitions.append(Transition(record.timestamp, status, previous))
        previous = status
    return transitions


def obstacle_stop_latencies(
    records: Iterable[Record], stop_distance: float
) -> list[Latency]:
    """How long from seeing an obstacle too close to commanding zero.

    Only scans that arrived while the robot was actually moving count. A
    stationary robot staring at a wall produces thousands of scans inside the
    stop distance and no stop to measure, and averaging those in would report
    a reassuring zero for a gate that was never asked to do anything.
    """
    velocities = _on(records, CMD_VEL)
    latencies: list[Latency] = []
    settled_until = float("-inf")

    for record in _on(records, SCAN):
        scan = record.message
        if not isinstance(scan, Scan) or scan.nearest is None:
            continue
        if scan.nearest > stop_distance or record.timestamp < settled_until:
            continue
        moving = _velocity_at_or_before(velocities, record.timestamp)
        if moving is None or moving.is_zero():
            continue

        stopped_at = _first_zero_after(velocities, record.timestamp)
        latencies.append(
            Latency(
                f"obstacle at {scan.nearest:.2f} m, inside the "
                f"{stop_distance:.2f} m stop distance",
                record.timestamp,
                stopped_at,
            )
        )
        # One approach to one obstacle is one event; without this every scan
        # for the rest of the approach would be reported again.
        settled_until = stopped_at if stopped_at is not None else float("inf")
    return latencies


def emergency_stop_latencies(records: Iterable[Record]) -> list[Latency]:
    """How long from an emergency-stop message to a zero velocity command."""
    velocities = _on(records, CMD_VEL)
    latencies: list[Latency] = []
    for record in _on(records, EMERGENCY_STOP):
        flag = record.message
        if not isinstance(flag, Flag) or not flag.value:
            continue
        before = _velocity_at_or_before(velocities, record.timestamp)
        latencies.append(
            Latency(
                "emergency stop requested",
                record.timestamp,
                _first_zero_after(velocities, record.timestamp),
                already_stopped=before is None or before.is_zero(),
            )
        )
    return latencies


def _stop_boundaries(records: Iterable[Record]) -> list[float]:
    """When the gate crossed into or out of a stop."""
    boundaries = []
    was_stopped: bool | None = None
    for transition in safety_transitions(records):
        if transition.status.stopped != was_stopped:
            boundaries.append(transition.timestamp)
            was_stopped = transition.status.stopped
    return boundaries


def check_motion_only_when_permitted(
    records: Iterable[Record], settle: float = BOUNDARY_SETTLE
) -> Finding:
    """Did a velocity ever reach the wheels while the gate said stop?

    This is the check an operator cannot run by eye. A stop that the robot
    obeys and a stop that it ignores look identical from across the room for
    the tenth of a second it takes to matter, and anything that found a way
    around the gate - a second publisher, a driver holding its last command -
    shows up here as motion the gate never authorised.

    `settle` is the blind spot this check has and cannot remove. The gate
    publishes the velocity and then the new state from one callback, and a
    recorder timestamps the two as it receives them, so at every transition
    the two can land microseconds apart in either order. Without the window a
    correct gate is reported as violating the rule at every single stop and
    every single release - measured on a real recording, twice in a
    twenty-second run - and a report that cries wolf on every run is a report
    nobody reads. Anything that persists past one control period is still
    caught, and a real path around the gate publishes continuously.
    """
    check = "Nothing moved while the safety gate said stop"
    velocities = _on(records, CMD_VEL)
    statuses = _on(records, SAFETY_STATE)
    if not velocities or not statuses:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            "The recording has no velocity commands, no safety states, or neither.",
            "Nothing was recorded that could answer this. The run proves nothing "
            "either way.",
        )

    boundaries = _stop_boundaries(records)
    offences: list[str] = []
    excused = 0
    stopped_moments = 0
    index = 0
    current: SafetyStatus | None = None
    for record in velocities:
        while index < len(statuses) and statuses[index].timestamp <= record.timestamp:
            candidate = statuses[index].message
            if isinstance(candidate, SafetyStatus):
                current = candidate
            index += 1
        if current is None or not current.stopped:
            continue
        stopped_moments += 1
        velocity = record.message
        if not isinstance(velocity, Velocity) or velocity.is_zero():
            continue
        if any(abs(record.timestamp - when) <= settle for when in boundaries):
            excused += 1
            continue
        offences.append(
            f"at {record.timestamp:.3f}s the gate was in "
            f'"{current}" and {velocity.magnitude:.3f} still went to the wheels'
        )

    if not stopped_moments:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            "The gate never reported a stop during this recording.",
            "Nothing stopped the robot in this run, so this recording says "
            "nothing about whether a stop is obeyed.",
        )
    if offences:
        return Finding(
            check,
            Outcome.FAILED,
            _examples(offences),
            "Something drove the robot while the safety gate was holding it "
            "stopped. Treat this as a path around the gate and stop hardware "
            "testing until it is explained.",
        )
    aside = (
        f" {excused} more fell within {settle * 1000:.0f} ms of the gate "
        "changing state, where the recording cannot tell which of the two "
        "messages the gate sent first; they are not counted either way."
        if excused
        else ""
    )
    return Finding(
        check,
        Outcome.PASSED,
        f"{stopped_moments} velocity commands were recorded while the gate was "
        f"stopped, and every one of them was zero.{aside}",
        "Whenever the gate said stop in this run, the wheels were told zero.",
    )


def check_latch_released_only_by_reset(records: Iterable[Record]) -> Finding:
    """Did the emergency stop ever let go on its own?

    The software stop is latched on purpose: a clear sensor reading, a
    restarted publisher, or a `false` on the stop topic must not release it.
    Only an explicit message on `emergency_stop_reset` may.
    """
    check = "The emergency stop stayed latched until an operator reset it"
    resets = [
        record.timestamp
        for record in _on(records, EMERGENCY_STOP_RESET)
        if isinstance(record.message, Flag) and record.message.value
    ]
    transitions = safety_transitions(records)

    latched_at: float | None = None
    releases: list[str] = []
    honest_releases = 0
    for transition in transitions:
        if transition.status.emergency:
            if latched_at is None:
                latched_at = transition.timestamp
            continue
        if latched_at is None:
            continue
        reset = [
            when for when in resets if latched_at <= when <= transition.timestamp
        ]
        if reset:
            honest_releases += 1
        else:
            releases.append(
                f"the latch was engaged at {latched_at:.3f}s and the gate "
                f'reported "{transition.status}" at {transition.timestamp:.3f}s '
                "with no reset message in between"
            )
        latched_at = None

    if latched_at is None and not honest_releases and not releases:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            "The emergency stop was never engaged during this recording.",
            "Nobody pressed stop in this run, so this recording says nothing "
            "about the latch.",
        )
    if releases:
        return Finding(
            check,
            Outcome.FAILED,
            _examples(releases),
            "The emergency stop released itself. A latched stop that lets go "
            "on its own is the failure this design exists to prevent.",
        )
    if not honest_releases:
        return Finding(
            check,
            Outcome.PASSED,
            "The emergency stop was engaged and was still engaged when the "
            "recording ended.",
            "The latch held for as long as this recording can show.",
        )
    return Finding(
        check,
        Outcome.PASSED,
        f"The latch was released {honest_releases} time(s), each after an "
        "explicit operator reset.",
        "Only a deliberate reset ever let the robot move again.",
    )


def check_sole_cmd_vel_publisher(
    publishers: Mapping[str, Sequence[str]] | None, gate_node: str
) -> Finding:
    """Was the safety gate the only thing publishing to the wheels?

    A bag stores messages, not who sent them, so this reads the record of the
    live topic graph that `graph_probe` wrote next to the bag while the run
    was being recorded. Without that file the question is unanswerable from
    the recording alone, and the report says so rather than guessing.
    """
    check = f"Only {gate_node} published {CMD_VEL}"
    if not publishers:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            "This recording carries no record of which nodes were publishing. "
            "Record with robot_telemetry's record.launch.py to capture one.",
            "Who was driving the wheels cannot be established from this "
            "recording. The messages were saved; the topic graph was not.",
        )

    observed = sorted(
        {name for name in publishers.get(canonical_topic(CMD_VEL), ()) if name}
    )
    if not observed:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            f"No node was seen publishing {CMD_VEL} while recording.",
            "The gate itself was not running, or was not observed. This run "
            "cannot show that the gate is the only way to the wheels.",
        )

    intruders = [name for name in observed if name != gate_node]
    if intruders:
        return Finding(
            check,
            Outcome.FAILED,
            f"{CMD_VEL} was published by: {', '.join(observed)}.",
            "Something other than the safety gate can drive the wheels "
            "directly. Every motion source must publish to "
            f"{canonical_topic('cmd_vel_requested')} instead.",
        )
    return Finding(
        check,
        Outcome.PASSED,
        f"{CMD_VEL} was published by {gate_node} and by nothing else.",
        "Every command that reached the wheels in this run had to pass "
        "through the safety gate first.",
    )


def check_run_is_evidence(records: Sequence[Record]) -> Finding:
    """Is there enough in this recording to judge anything at all?

    An empty or near-empty bag has no violations in it, which is exactly why
    it must not be reported as a pass. Attaching one to a pull request would
    be a claim that nothing went wrong, made from a file that says nothing.
    """
    check = "The recording contains something to judge"
    if not records:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            "The recording is empty.",
            "No run was captured. Nothing here is evidence of anything.",
        )
    velocities = _on(records, CMD_VEL)
    statuses = _on(records, SAFETY_STATE)
    if not velocities and not statuses:
        return Finding(
            check,
            Outcome.NOT_EXERCISED,
            f"{len(records)} messages were recorded, but none on {CMD_VEL} or "
            f"{SAFETY_STATE}.",
            "The safety gate was not running, or was not recorded. Nothing "
            "here shows how the robot was being controlled.",
        )
    span = records[-1].timestamp - records[0].timestamp
    return Finding(
        check,
        Outcome.PASSED,
        f"{len(records)} messages over {span:.1f} s, including "
        f"{len(velocities)} velocity commands and {len(statuses)} safety states.",
        "There is a run here to inspect.",
    )


def ended_mid_stop(records: Iterable[Record]) -> SafetyStatus | None:
    """The gate's last word, if the recording stops while it was stopping.

    Not a violation - a run may legitimately end with the robot held still -
    but the reader needs to know that nothing in the file shows the stop being
    released, so the recording cannot vouch for what happened next.
    """
    transitions = safety_transitions(records)
    if transitions and transitions[-1].status.stopped:
        return transitions[-1].status
    return None


def _examples(offences: Sequence[str]) -> str:
    shown = "; ".join(offences[:MAX_EXAMPLES])
    if len(offences) > MAX_EXAMPLES:
        shown += f"; and {len(offences) - MAX_EXAMPLES} more"
    return f"{len(offences)} occurrence(s): {shown}."
