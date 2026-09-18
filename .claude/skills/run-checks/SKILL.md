---
name: run-checks
description: Run this repository's Python checks - the eight unittest suites across the ROS 2 packages and web_ui, compileall, and ruff. Use whenever asked to run the tests, lint, or verify a change before committing, and after editing anything under robot/ros2_ws/src or web_ui.
---

# Run checks

Every suite needs the five source packages on `PYTHONPATH`; running `python3 -m
unittest` without it fails on imports. The script sets it and runs all eight
suites, so use the script rather than assembling the command by hand.

```bash
.claude/skills/run-checks/check.sh          # tests + compileall + ruff
.claude/skills/run-checks/check.sh tests    # suites only
.claude/skills/run-checks/check.sh lint     # ruff only
.claude/skills/run-checks/check.sh compile  # compileall only
```

Exit status is non-zero if anything fails. Passing suites print one line; a
failing suite prints its full output.

Pass extra `unittest` arguments after the mode: `check.sh tests -v`.

## Notes

- Read the per-suite lines and the exit status, and the final `check.sh:
  PASSED`/`check.sh: FAILED` line. Do **not** read `All checks passed!` as the
  verdict: that line is printed by `ruff`, reports on lint alone, and still
  appears when a suite above it failed.
- The `robot_safety` suite needs `rclpy`. The script sources
  `/opt/ros/jazzy/setup.bash` itself when the import fails and ROS is
  installed, so those 13 tests run inside the devcontainer without any manual
  sourcing. `(skipped=13)` means ROS is genuinely absent - a Codespace that was
  never rebuilt into the container. Skips are not failures, but they mean the
  tests guarding the motion gate did not run.
- The `robot_bringup` and `robot_navigation` suites recompute `stop_distance`,
  `caution_distance`, the Nav2 footprint, and the inflation radius from
  `base_dynamics.yaml` and the URDF, and fail on drift. A failure there means an
  input changed or a derived value was hand-edited - fix the input, per AGENTS.md.
- This runs plain Python. ROS package builds (`colcon build`) still need
  `source /opt/ros/jazzy/setup.bash` first; see AGENTS.md.
