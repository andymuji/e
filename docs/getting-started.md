# Getting started

## Supported baseline

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3

The included development container installs this baseline and the workspace dependencies automatically.

## Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd robot/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the safety command gate

```bash
ros2 run robot_safety safety_node
```

The node accepts requested motion on `cmd_vel_requested`, laser scans on `scan`, and a software stop signal on `emergency_stop`. It publishes gated motion on `cmd_vel` and status changes on `safety_state`.

The controller fails closed: missing, invalid, or stale range data produces a stop command. Do not connect any other node directly to the motor controller's velocity input.

## Run fast tests without ROS

From the repository root:

```bash
for package in robot_core robot_locations robot_navigation robot_safety robot_voice; do
  PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python -m unittest discover -s robot/ros2_ws/src/$package/tests -v || exit 1
done
python -m compileall -q robot/ros2_ws/src
```

CI repeats the Python checks and builds all five ROS packages on Jazzy.
