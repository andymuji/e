# Robot Project

ROS 2-based assistive robot for elder care. See [docs/decisions/0001-ros-baseline.md](docs/decisions/0001-ros-baseline.md) for architecture decisions and [docs/safety-test-procedure.md](docs/safety-test-procedure.md) before running anything on hardware.

## Baseline

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3
- Apache-2.0 license

## Packages

- `robot_core`
- `robot_locations`
- `robot_navigation`
- `robot_safety`
- `robot_voice`

## Safety Rules

- Every requested motion command must pass through `robot_safety` before reaching the hardware controller.
- The software emergency stop is secondary; the physical robot requires an independently wired emergency stop.
- Do not run hardware tests without the documented tether, exclusion zone, and second operator.
- Do not connect another node directly to the motor controller's velocity input.

## Development Checks

From the repository root:

```bash
# Run the full Python test suite.
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_core/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_locations/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_navigation/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_safety/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m unittest discover -s robot/ros2_ws/src/robot_voice/tests -v
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_safety python3 -m unittest discover -s web_ui/tests -v

# Compile Python sources.
python3 -m compileall -q robot/ros2_ws/src web_ui
```

For ROS package work, source Jazzy first and build from `robot/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
cd robot/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
