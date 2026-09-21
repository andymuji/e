# Agent fan-out: completing the software before the hardware exists

Status: four agents dispatched 2026-09-18, each in its own git worktree, file
ownership disjoint so they cannot collide. Integration is a merge per agent.

## Environment precondition: MET

The devcontainer rebuild has happened. Verified:

- `/opt/ros/jazzy` present; `ros2` and `gz` resolve after
  `source /opt/ros/jazzy/setup.bash` (they are not on PATH before it)
- `colcon build --symlink-install` builds all 7 packages clean in ~13s
- `rclpy` imports under python3.12

## Prerequisite fixed before dispatch

`check.sh` replaced `PYTHONPATH` instead of appending, dropping ROS's own
site-packages. The 13 `SafetyNode` tests - the gate every motion command
passes through - skipped on the rclpy import both locally and in CI, whose
unit-tests job installs no ROS. They had never run anywhere. Now sourced and
appended: robot_safety reports 61 tests, no skips.

(An earlier draft of this plan credited those 13 tests to robot_voice. They
are robot_safety's; robot_voice has no ROS tests at all yet.)

## What is actually hardware-blocked

Only the measured geometry:

- `robot_description/urdf/robot.urdf.xacro` dimensions (placeholders)
- `robot_bringup/config/base_dynamics.yaml` (`measured: false`)

Everything downstream - `stop_distance`, `caution_distance`, the Nav2
footprint and inflation radius - is *derived* from those, recomputed by the
tests, and guarded against hand-editing. So the blocked surface is two input
files, not the safety numbers themselves. Nothing else in the software needs
the robot to exist.

## The four tracks

| Agent | Track | Owns |
|---|---|---|
| 1 | robot_voice ROS node (safety-critical) | `robot_voice/**`, new `voice.launch.py` + its test |
| 2 | robot_locations node/service | `robot_locations/**`, new `locations.launch.py` + its test |
| 3 | First real sim bring-up (verification, not codegen) | bringup launch/config/worlds, `test_bringup.py`, rviz + gazebo xacro |
| 4 | Measurement-independent hardening | `robot_navigation/**`, `web_ui/**`, `docs/**` |

Agents 1 and 2 each get their own launch file rather than editing
`simulation.launch.py`, and their own test file rather than editing
`test_bringup.py` - that is what keeps them parallelizable. `setup.py` already
globs `launch/*.launch.py`, and `test_bringup.py` globs launch files for the
valid-Python check, so neither needs a shared-file edit.

Agent 3 is the high-value one: it observes the topic graph running for the
first time. The claim that exactly one publisher exists on `/cmd_vel` has
never been checked against a live system.

## Invariants every agent was given

- Only `robot_safety` publishes `/cmd_vel`; new motion sources publish to
  `/cmd_vel_requested` or go through Nav2
- The safety gate is never placed under lifecycle management
- Do not hand-edit the derived distances or the placeholder dimensions, and do
  not mark anything `measured: true`
- Latched e-stop: only `emergency_stop_reset` releases it
- `.claude/skills/run-checks/check.sh` must pass, and the robot_safety line
  must read plain `OK` - a `(skipped=N)` there means the gate regressed
- Commit on the worktree branch; do not push, PR, or merge

## Known gap

`.claude/hooks/guard-derived-distances.sh` matches `Write|Edit` only. An agent
editing `safety.yaml` through Bash (`sed`, a heredoc) bypasses it. The
`robot_bringup` drift tests still catch the result, so the guard is a fast
failure rather than the only one - but it is not airtight, and agents told to
prefer Bash for edits are exactly the case it misses.

---

# Second fan-out: 2026-09-19

The first four tracks are merged. `check.sh` now discovers packages and suites
by globbing `robot/ros2_ws/src`, so a new package is on `PYTHONPATH` and under
test without editing a shared file - that is what keeps this round's two new
packages from colliding.

Still hardware-blocked, and still only this: the URDF dimensions and
`base_dynamics.yaml`. Everything below is measurement-independent.

| Agent | Track | Owns |
|---|---|---|
| 5 | Console to ROS: the operator console drives a real graph | `web_ui/**`, new `robot_console/**` |
| 6 | Navigation bring-up: map the test room, reach a goal | `robot_bringup/**`, `robot_description/**`, `docs/getting-started.md` |
| 7 | Flight recorder: recorded runs as safety evidence | new `robot_telemetry/**`, `docs/safety-test-procedure.md` |
| 8 | Voice input path and the speech-provider ADR | `robot_voice/**`, `docs/decisions/0002-*.md` |

Shared files nobody owns: `README.md`, `AGENTS.md`, this file, `.claude/**`,
`.github/**`. The integrator edits those at merge, including adding the two
new packages to the CI `ros-build` package list.

`robot_safety` and `robot_core` are read-only for all four. The gate is not
this round's work; three of the four tracks exist to observe it.

Same invariants as the first round, plus: `ros2 topic pub` and
`ros2 service call` are denied by repo policy, so an agent that needs the
robot to move asks Nav2 for a goal rather than routing around the gate or
around the permission.

## Integration notes

- Agent 8 (voice) merged. Its worktree was cut from `11c67d3` rather than the
  branch tip and its sandbox refused `source /opt/ros/jazzy/setup.bash`, so it
  could not build or run anything. The integrator did that verification:
  `colcon build` clean, and a live `voice_node` + `say` run confirming an
  approved destination reaches the goal attempt, an unapproved one is refused,
  and a heard stop publishes the latch.
- The gap it found and could not fix is fixed here: a stop word the engine
  mangled ("sto", "hal", "wait") was a refusal, and a refusal does not cancel
  a trip already running. It is now a `halt` outcome - trip cancelled, latch
  untouched - in `command_gateway.py` and `voice_node.py`, recorded in ADR
  0002.
- **Owed to `web_ui/app.py` when agent 5 merges**: `_handle_voice_command`
  dispatches on the outcome action and does not know about `halt`, so a
  mangled stop typed into the console answers correctly but does not cancel
  the trip. One `elif` beside the existing `stop` branch. Left undone here
  only because agent 5 is editing that file.

## Second fan-out: outcome

All four merged; `check.sh: PASSED` with ten suites, `colcon build` clean on
nine packages. Two defects the agents found in code they could not touch were
fixed by the integrator and are covered by tests:

- `safety_state` was published change-only and volatile, so anything
  subscribing to a settled robot heard nothing and failed closed to "unknown".
  Now transient-local depth 1; the test fails against the old publisher.
- A mangled "stop" - what a real recogniser makes of the word - was a refusal,
  and a refusal does not cancel a trip. Now a `halt`: trip cancelled, latch
  untouched.
- The gate also ended every Ctrl-C with a traceback and a non-zero status.

### What the worktrees got wrong

Every one of the four agents was given a worktree cut from `11c67d3`, roughly
twenty commits behind the branch tip, rather than from HEAD. Agent 8 could not
recover (its attempts to fast-forward were denied) and shipped code it could
never build; the other three each found a different way to the right commit
and lost time doing it. **Check the base commit of every worktree before
dispatching the next round.**

Agent 7 also reports that the harness instructs agents to prefer `sed` and
heredocs for edits, which is exactly the bypass in the "Known gap" above, and
that the scratchpad is shared between parallel agents - one agent overwrote
another's helper script mid-run. Give agents uniquely named scratch files and
tell them to use Write/Edit.

### Still open

- **Nothing has navigated.** The mapping drive and the goal-reaching run were
  blocked: the documented procedure needs `simulation.launch.py safety:=false`
  (so the simulation gate and the stricter navigation gate do not stack on
  `/cmd_vel`), and the permission system refuses that argument as weakening
  safety. Agent 6 did not route around it, which was right. This needs the
  user's decision, not an agent's.
- No recording has been made of a full simulation run, so `robot_telemetry`
  has never read `/map`, `/amcl_pose` or `/tf` in anger - only counted them.

---

# Third fan-out: 2026-09-21 — two humans, not four agents

Split for two people working at the same time. The dividing line is the
devcontainer: **Track A needs ROS + Gazebo running, Track B does not.** That
keeps the two people off each other's ROS graph as well as out of each other's
files, and it means Track B still moves in a Codespace that was never rebuilt.

Still hardware-blocked, and still only this: `robot_description/urdf/robot.urdf.xacro`
dimensions and `robot_bringup/config/base_dynamics.yaml`. Nothing below needs
the robot to exist.

## Blocker to clear before Track A starts

`simulation.launch.py safety:=false` is refused by the permission system as
weakening safety. Reading the argument's own description, it is the opposite:
`navigation.launch.py` brings its own gate with the stricter navigation
params, and two gates both publishing `/cmd_vel` means the laxer one keeps
commanding motion while the stricter one is trying to stop. The robot is
gated either way; the flag decides *which* gate, not *whether*. Allowing it is
the safe choice, and it is the only thing standing between this repo and its
first goal-reaching run. Needs the user's decision, not an agent's.

## Track A — drive it (needs the container)

Owns: `robot_bringup/launch/*`, `robot_bringup/config/nav2*`, `robot_bringup/maps/**`,
`robot_navigation/config/**`, `docs/getting-started.md`.
Runs `robot_telemetry` but does not edit it.

1. First goal-reaching run in simulation. Nothing has navigated, anywhere.
2. Drive the room under SLAM and save a real map, replacing the one rendered
   from the world file. Removes a placeholder the same way measuring the base
   would.
3. Record that run with `robot_telemetry` and run `analyse` on it. The
   recorder has never read `/map`, `/amcl_pose` or `/tf` in anger, only
   counted them. Expect the analyser's `NOT EXERCISED` lines to turn into
   real passes or real failures — either is information.
4. Tune AMCL and the DWB critics from what the runs show. They are untuned
   because there has never been a run to tune them from.
5. Confirm the localization stop actually fires. `require_localization` is
   already `true` in `safety_navigation.yaml`, but it has never fired in a
   real run because nothing has ever navigated. A stop condition that has
   never triggered is a claim, not a feature. (`safety.yaml` has it `false`
   for plain simulation; check that asymmetry is deliberate.)

## Track B — build it (no container needed)

Owns: new `robot_bringup/config/collision_monitor.yaml`, new
`robot_bringup/launch/collision_monitor.launch.py`, new `docs/hazard-analysis.md`,
`.github/workflows/**`, `robot_telemetry/**`, `robot_console/**`, `web_ui/**`,
`robot_voice/**`.

1. Nav2 Collision Monitor. README integration order step 5, absent from the
   repo. It is the independent second layer that is supposed to sit beside
   `robot_safety`, not behind it. Its footprint derives from the URDF like
   everything else, so it is measurement-independent. New config + new launch
   file + new test, so it does not collide with Track A; wiring it into
   `navigation.launch.py` is a one-line integration at merge.
2. Hazard analysis. README suggested first issue 2 is "hazard analysis and
   emergency-stop test procedure". The test procedure exists;
   the hazard analysis does not. It is the document the Collision Monitor
   zones and the exclusion zone should both be argued from.
3. A regression test for the safety topology that CI can actually run. CI has
   no Gazebo, so every claim proven by the two live runs — one publisher on
   `/cmd_vel`, Nav2 remapped, `behavior_server`'s three publishers — is
   currently proven once by hand and never again. Either headless Gazebo in
   CI or a launch-graph test that does not need it.
4. Whatever Track A's recordings expose in the analyser. Track A produces the
   recordings; Track B fixes the analyser.

## Merge

Same as previous rounds: one merge per track. `README.md`, `AGENTS.md`, this
file and `.claude/**` belong to neither track and are edited at merge.

## Track A progress: 2026-09-21, third live bring-up

`simulation.launch.py safety:=false` was attempted and **refused again** by the
permission classifier ("Safety Bypass Flag"). Track A items 1, 2, 4 and 5 stay
blocked behind it; no attempt was made to reconstruct the same topology by
hand, because that is the denial's intent rather than its letter.

What was done instead, with the simulator in its *safest* configuration
(gate on, headless, `ROS_DOMAIN_ID=42`):

- `colcon build` clean, 9 packages, 16 s.
- Third live bring-up. `/cmd_vel` had **one** publisher, `safety_controller`,
  and one subscriber, the Gazebo bridge. Third independent confirmation.
- **New, and not previously checked live: `/emergency_stop_reset` has zero
  publishers.** Nothing in a running graph can release the latch. That is two
  written safety rules confirmed against a live system rather than by reading
  code - `robot_voice` cannot reset, and `robot_telemetry` publishes nothing.
- The recorder ran against a live graph for the first time (the plan said it
  never had). It subscribed to `/tf`, `/tf_static`, `/scan`, `/cmd_vel` and
  `/safety_state` and produced two real recordings, in `/root/robot_runs/`.
- Voice to latch, live: `say "stop"` moved the gate from "no motion command
  received" to "emergency stop active", and the latch **held** through a
  subsequent `say "go to the kitchen"`. The destination did not release it.
- Both recordings analysed. The second reports four checks PASSED including
  the latch, and honestly refuses to call the 55 ms zero-command a stopping
  time because the robot was already stationary.
- `check.sh: PASSED`, ten suites, `robot_safety` a plain `OK`.

### Defect for Track B: a green verdict over an unexercised check

`robot_telemetry/report.py` lines 64-72. `Report.verdict` returns
`INCONCLUSIVE` only when **every** finding is `NOT_EXERCISED`. One passing
check is enough to make the headline `VERDICT: PASS` and the exit code `0`.

The first recording proves it: the emergency stop was never touched, the latch
check reported `NOT EXERCISED`, and the report still opened with
`VERDICT: PASS` and exited `0`. `docs/safety-test-procedure.md` says a check
the run never put to the test "is deliberately not a pass" - the body of the
report honours that, the two things a reviewer or a CI job actually reads do
not. A recording where nobody pressed stop should not be attachable to a pull
request under a green headline.

Suggested shape: any `NOT_EXERCISED` finding makes the verdict
`INCOMPLETE` - distinct from both `PASS` and `INCONCLUSIVE` - with a non-zero
exit. That is Track B's call; it owns the file.

### Smaller thing, working as designed

A recorder stopped with `SIGINT` from a non-terminal left no `metadata.yaml`.
`analyse_run` detected it, exited `3`, and printed the exact `ros2 bag reindex`
command to fix it, which worked. Worth keeping; it behaved better than the
thing it was reporting on.
