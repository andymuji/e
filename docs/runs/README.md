# Recorded runs

Reports from `robot_telemetry`'s `analyse_run`, kept as committed evidence
rather than as claims in a commit message. Each one is about **one recording**
and is not a substitute for the physical test in
[../safety-test-procedure.md](../safety-test-procedure.md).

Both were produced on 2026-09-21 against the Gazebo simulation running
headless, with the safety gate enabled. No hardware was involved, and no
robot has yet driven to a goal.

The recordings themselves (`.mcap`, 2-3 MB each) are not committed. They are
in `/root/robot_runs/` on the machine that made them; ask before assuming they
still exist.

## `idle-gate-check-20260921-164231.txt`

A stationary robot, 75.9 s. The gate held the robot stopped the whole time
because nothing ever sent a motion command, which is the correct fail-closed
behaviour.

**Read this one for what it gets wrong.** Nobody touched the emergency stop,
so the latch check reports `NOT EXERCISED` - and the report still opens with
`VERDICT: PASS` and exits `0`.

## `estop-via-voice-20260921-164558.txt`

70.8 s, with the emergency stop actually exercised through the voice path:
`ros2 run robot_voice say "stop"` latched the gate, and a following
`say "go to the kitchen"` did **not** release it. All four checks passed,
including the latch, and this time the pass is earned.

The report also declines to call the 55 ms zero-command a stopping time,
because the robot was already stationary. That is the analyser being honest
about what a recording can and cannot show, and it is the behaviour the
verdict logic should match.

## The defect these two demonstrate together

`robot_telemetry/report.py`, `Report.verdict`: the verdict softens to
`INCONCLUSIVE` only when **every** finding is `NOT_EXERCISED`. One passing
check is enough to print `VERDICT: PASS` and exit `0`.

So the idle run - in which the emergency stop was never touched - is
attachable to a pull request under a green headline and a zero exit code.
`safety-test-procedure.md` says an unexercised check "is deliberately not a
pass". The body of the report honours that; the headline and the exit code,
which are the two things a reviewer and a CI job actually read, do not.

Suggested shape: any `NOT_EXERCISED` finding yields `INCOMPLETE`, distinct
from both `PASS` and `INCONCLUSIVE`, with a non-zero exit. This belongs to
whoever owns `robot_telemetry`.
