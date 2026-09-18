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

- `robot_safety` skips ~13 tests when `rclpy` is not importable. Skips are not
  failures, but a run under a sourced ROS 2 Jazzy environment executes them.
- The `robot_bringup` and `robot_navigation` suites recompute `stop_distance`,
  `caution_distance`, the Nav2 footprint, and the inflation radius from
  `base_dynamics.yaml` and the URDF, and fail on drift. A failure there means an
  input changed or a derived value was hand-edited - fix the input, per CLAUDE.md.
- This runs plain Python. ROS package builds (`colcon build`) still need
  `source /opt/ros/jazzy/setup.bash` first; see CLAUDE.md.
