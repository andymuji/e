# Robot Project

<!-- This is the one instruction file for this repository. Codex, Claude Code
     and other coding agents all read it. CLAUDE.md is a one-line pointer at
     this file, because Claude Code only looks for that name - do not paste a
     copy of these rules into it, or the two will drift and the copy will be
     the one missing a safety rule. -->

ROS 2-based assistive robot for elder care. See [docs/decisions/0001-ros-baseline.md](docs/decisions/0001-ros-baseline.md) for architecture decisions and [docs/safety-test-procedure.md](docs/safety-test-procedure.md) before running anything on hardware.

## Baseline

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3
- Apache-2.0 license

## Environment

`.devcontainer/devcontainer.json` provides ROS 2 Jazzy, Gazebo Harmonic, colcon,
ruff, the GitHub CLI, and the Claude Code CLI. A GitHub Codespace started
without rebuilding into that container has Python and ruff only - no
`/opt/ros`, no `colcon`, no `gz`. The Python checks below run either way;
anything under "For ROS package work" needs the container (VS Code command
palette: **Dev Containers: Rebuild Container**).

## Packages

- `robot_bringup` - launch files, parameters, simulation worlds
- `robot_core`
- `robot_description` - URDF/xacro, RViz config
- `robot_locations`
- `robot_navigation`
- `robot_safety`
- `robot_voice`

The URDF dimensions are placeholders. Nav2 footprints, inflation radii, and the
`robot_safety` distances all derive from them, so they must be replaced with
measured values before any hardware test.

## Safety Rules

- Every requested motion command must pass through `robot_safety` before reaching the hardware controller.
- The software emergency stop is secondary; the physical robot requires an independently wired emergency stop.
- The software emergency stop is latched. Only an explicit reset on `emergency_stop_reset` releases it; a `false` on `emergency_stop` must never do so.
- Loss of the command stream is a stop condition, not a reason to hold the last velocity. Both `sensor_timeout` and `command_timeout` fail closed.
- Do not run hardware tests without the documented tether, exclusion zone, and second operator.
- Do not connect another node directly to the motor controller's velocity input.
- Motion path: `/cmd_vel_requested` -> `robot_safety` -> `/cmd_vel` -> driver or Gazebo DiffDrive. Nothing else publishes `/cmd_vel`.
- Nav2 is a motion source, not a motion authority. Every Nav2 node that can emit a velocity has `cmd_vel` remapped to `cmd_vel_requested`, including `behavior_server`: its recovery behaviours drive the robot, and they run when something has already gone wrong.
- The safety gate is never put under lifecycle management. The lifecycle manager deactivates its nodes on failure, and the gate has to still be running then.
- `stop_distance` and `caution_distance` are derived from `robot_bringup/config/base_dynamics.yaml`, and the Nav2 footprint and inflation radius from the URDF dimensions. Do not hand-edit them; change the inputs. The tests recompute both and fail on drift, and `.claude/hooks/guard-derived-distances.sh` refuses agent edits that touch either value in `safety.yaml`.

## Development Checks

From the repository root:

```bash
.claude/skills/run-checks/check.sh          # all eight test suites, compileall, ruff
.claude/skills/run-checks/check.sh tests    # suites only (add -v for verbose)
.claude/skills/run-checks/check.sh lint     # ruff only
.claude/skills/run-checks/check.sh compile  # compileall only
```

The script sets the `PYTHONPATH` the suites need; `python3 -m unittest` run
directly without it fails on imports. CI runs the same script, so a green local
run and a green pipeline cannot drift apart.

For ROS package work, source Jazzy first and build from `robot/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
cd robot/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the simulation with `ros2 launch robot_bringup simulation.launch.py`, and
drive it from a second terminal with `ros2 launch robot_bringup teleop.launch.py`.
Map with `slam.launch.py`, then navigate a saved map with
`navigation.launch.py map:=...`. The SLAM and Nav2 configuration has not been
run against the simulator yet; treat the first run as bring-up.
