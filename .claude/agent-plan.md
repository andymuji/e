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
