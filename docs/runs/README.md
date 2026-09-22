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

**Read this one for what it caught.** Nobody touched the emergency stop, so
the latch check reports `NOT EXERCISED` - and the report still opens with
`VERDICT: PASS` and exits `0`. That was a defect in the analyser, and this
file is the evidence of it. The text is kept exactly as it was produced on
the day; see "The defect these two demonstrate together" below for what
changed as a result.

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

**Found here, fixed on 2026-09-22.** Kept on the record, because the fix is
only meaningful next to the report that proved it was needed.

The verdict used to soften to `INCONCLUSIVE` only when **every** finding was
`NOT_EXERCISED`. One passing check was enough to print `VERDICT: PASS` and
exit `0`. So the idle run - in which the emergency stop was never touched -
was attachable to a pull request under a green headline and a zero exit code.
`safety-test-procedure.md` says an unexercised check "is deliberately not a
pass": the body of the report honoured that, and the headline and the exit
code, which are the two things a reviewer and a CI job actually read, did not.

There is now a fourth verdict. Any `NOT_EXERCISED` finding yields
`INCOMPLETE` - distinct from both `PASS` and `INCONCLUSIVE` - and exit code
`4`.

Re-run against the same recordings today, the two files above would read:

- the idle run: `VERDICT: INCOMPLETE`, exit `4`. No longer green.
- the emergency-stop run: `VERDICT: PASS`, exit `0`, unchanged. All four of
  its checks were genuinely exercised, so its pass was earned and stays.

That is the shape a fix should have: it changes the verdict on the run that
proved nothing, and leaves alone the run that proved something.
