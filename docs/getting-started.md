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

The `robot_safety` package also includes ROS-aware tests for obstacle speed limiting and emergency-stop precedence. They run during the ROS 2 package test phase; the fast Python test command below skips them when ROS 2 is not installed.

## Run fast tests without ROS

From the repository root:

```bash
for package in robot_core robot_locations robot_navigation robot_safety robot_voice; do
  PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python -m unittest discover -s robot/ros2_ws/src/$package/tests -v || exit 1
done
python -m compileall -q robot/ros2_ws/src
```

CI repeats the Python checks and builds all five ROS packages on Jazzy.

## Run the developer web console

The local web console is in `web_ui/`. It runs repeatable safety scenarios, reports the controller decision and sensor inputs, and includes a navigation smoke test for approved named locations. Scenario controls change simulated inputs only; they do not expose coordinates or direct motor commands.

From the repository root, run:

```bash
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_safety python3 -m web_ui.app
```

Open <http://127.0.0.1:8080>. The current `DemoGoalDispatcher` and `SafetyScenarioAdapter` are in-memory adapters for local testing. Replace the goal adapter with one that submits `NavigationGoal.pose` to `NavigateToPose`; keep the existing safety node between navigation output and the motor controller.
