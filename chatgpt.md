ok ge# Robot Project

ROS 2-based assistive robot for elder care. See [docs/decisions/0001-ros-baseline.md](docs/decisions/0001-ros-baseline.md) for architecture decisions and [docs/safety-test-procedure.md](docs/safety-test-procedure.md) before running anything on hardware.

## Baseline

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3
- Apache-2.0 license

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

## Development Checks

From the repository root:

```bash
# Run the full Python test suite.
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_bringup/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_core/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_description/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_locations/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_navigation/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_safety/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_voice/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s web_ui/tests -v

# Compile Python sources.
python3 -m compileall -q robot/ros2_ws/src web_ui
```

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

