# Safety Controller Test Procedure

## Software test

Run from the repository root:

```bash
PYTHONPATH=robot/ros2_ws/src/robot_safety python3 -m unittest discover -s robot/ros2_ws/src/robot_safety/tests -v
```

The test suite must verify that:

- A clear path permits full commanded speed.
- An obstacle inside the caution distance reduces speed.
- An obstacle inside the stop distance commands zero speed.
- Emergency stop commands zero speed even with a clear sensor reading.
- Missing, infinite, or NaN sensor data commands zero speed.
- Invalid stop and caution thresholds are rejected.

## Hardware test procedure

Do not substitute this software test for a physical emergency-stop test.

1. Raise the robot so the wheels cannot contact people or furniture.
2. Confirm the normally-closed emergency-stop circuit removes motor power.
3. Release the emergency stop and verify the robot remains stopped until a deliberate enable action.
4. At the lowest available speed, place a non-damaging obstacle in front of the robot.
5. Verify the robot slows inside the caution distance and stops inside the stop distance.
6. Disconnect or block the obstacle sensor and verify the robot stops on invalid or stale data.
7. Repeat on the floor inside a marked exclusion zone with a tether and a second person operating the stop.

Record sensor type, threshold values, robot speed, payload, test result, and any unexpected behavior in the issue or pull request for the change.

## Recording a run as evidence

Every step above still has to be performed and written up by hand. Recording
adds a second, independent account of the same run: one that nobody has to
remember accurately, and that a reviewer can check for the things an operator
cannot see from across the room.

Start the recorder in its own terminal, before the test and alongside whatever
is already running (the simulator, or the robot's bringup):

```bash
ros2 launch robot_telemetry record.launch.py label:=estop-test
```

Name the run after the step it covers, so the evidence can be matched back to
the procedure. Recordings are written to `~/robot_runs/<label>-<timestamp>/`
by default; `output_dir:=...` puts them somewhere else.

Perform the test. Then stop the recorder with a single `Ctrl-C` in its own
terminal and **wait for it to exit on its own**.

Stopping it any other way - from a script, or by closing the window - leaves a
directory with the messages in it but no `metadata.yaml`, and nothing can read
that. The recording is not lost: `ros2 bag reindex <directory>` rebuilds the
index, and `analyse_run` says so when it hits one.

Analyse what was caught:

```bash
ros2 run robot_telemetry analyse_run ~/robot_runs/estop-test-20260919-101500 \
  --output estop-test-report.txt
```

Attach both the report and the recording directory to the pull request or the
issue, next to the hand-written notes required above.

### What the report checks

- Every change of safety state, with the gate's own reason and its timestamp.
- How long the robot took to be commanded to zero after a scan first saw
  something inside the stop distance, and after an emergency stop was pressed.
- Whether anything other than the safety gate published `/cmd_vel`.
- Whether a non-zero velocity ever reached the wheels while the gate was
  reporting a stop. **This is the one no operator can perform.** A stop that
  the robot obeys and a stop it ignores look identical for the tenth of a
  second in which the difference matters, and anything that found a path
  around the gate shows up here and nowhere else.
- Whether the latched emergency stop ever released without an explicit
  operator reset on `emergency_stop_reset`.

A check the run never put to the test is reported as `NOT EXERCISED`, which is
deliberately not a pass: a recording in which nobody pressed the emergency
stop proves nothing about the emergency stop. The headline verdict and the
exit code both say so, so neither a reviewer reading the first line nor a CI
job reading the exit code can miss it:

| Exit | Verdict | Meaning |
|---|---|---|
| `0` | `PASS` | Every check was exercised, and the run broke none of the rules |
| `1` | `FAIL` | The run broke a safety rule |
| `2` | `INCONCLUSIVE` | The recording cannot answer any of the questions |
| `4` | `INCOMPLETE` | Nothing failed, but the run left at least one check unexercised |
| `3` | *(none)* | The recording could not be read at all |

Only `0` is success, and only a run that put every check to the test can reach
it. `INCOMPLETE` was added after a recorded run in `docs/runs/` was found to
be attachable to a pull request under a green headline with the emergency-stop
check never exercised.

If you record without `record.launch.py` - with a bare `ros2 bag record` - the
question of who was publishing `/cmd_vel` cannot be answered, because a bag
stores messages and not their senders. The launch file also runs a probe that
writes the observed topic graph into the recording.

### What a green report is not

A green report is evidence about **one recording**. It is never a substitute
for the physical emergency-stop test above, and it can never be counted as one
of the seven steps.

- It says the software behaved on that run. The physical stop circuit is
  hardware, and a report cannot observe a wire.
- It cannot say the robot would behave on the next run, at a different speed,
  with a payload, or on a different floor.
- `NOT EXERCISED` is not a pass. A run that leaves any check unexercised can
  no longer reach `PASS` at all - it reports `INCOMPLETE` - so a green report
  now means every question was actually asked. It still does not mean the
  answer will be the same on the next run.

`robot_telemetry` only listens. It contains no publisher of any kind: not a
velocity, and above all not an `emergency_stop_reset`, which is an operator
action and must never be something a recording tool can perform. A witness
that can also act is no longer a witness.
