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

## Run the simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash          # from robot/ros2_ws, after colcon build
ros2 launch robot_bringup simulation.launch.py
```

This starts Gazebo Harmonic with the `test_room` world, spawns the robot,
bridges the sensor and command topics, launches the safety gate, and opens
RViz. Pass `rviz:=false` to skip RViz, or `world:=/path/to/other.sdf` to load
a different world.

Drive it from a second terminal:

```bash
ros2 launch robot_bringup teleop.launch.py
```

Teleop publishes to `/cmd_vel_requested`. Nothing reaches the wheels without
passing the safety gate first:

```text
teleop / Nav2  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel  ->  Gazebo DiffDrive
```

Drive slowly at the table or the cabinet. The robot should limit itself to 35%
speed inside 0.85 m and stop at 0.45 m, and `safety_state` should say why:

```bash
ros2 topic echo /safety_state
```

Engage and release the software stop by hand:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /emergency_stop_reset std_msgs/msg/Bool "{data: true}"
```

Publishing `false` on `/emergency_stop` deliberately does nothing: see the
latch note below.

## Build a map

Mapping is supervised and manual. With the simulation running, start SLAM
Toolbox and drive the robot around the room yourself:

```bash
ros2 launch robot_bringup slam.launch.py
ros2 launch robot_bringup teleop.launch.py     # in another terminal
```

Nothing in `slam.launch.py` commands motion, and it starts no planner: the
robot should not be driving itself through a map that is still half built.
When the room is covered, save the map deliberately and check it in, so the
map Nav2 localizes against is a reviewed artifact:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/test_room
```

## Navigate to a goal

Start the simulation with its own gate turned off, because this launch brings
one:

```bash
ros2 launch robot_bringup simulation.launch.py safety:=false
ros2 launch robot_bringup navigation.launch.py map:=/absolute/path/test_room.yaml
```

This starts AMCL, Nav2, and the safety gate.

`safety:=false` matters. Both launch files start a gate, and run together they
both subscribe to `cmd_vel_requested` and both publish `cmd_vel`. The gate this
launch brings is the strict one - it adds the localization and battery checks -
so the simulation's gate would go on commanding motion while this one is trying
to stop. Leave the simulation's gate on when driving by teleop, and off
whenever Nav2 is coming. Set the initial pose in RViz
before sending a goal: AMCL has to be told roughly where the robot is.

Nav2 is a motion source, not a motion authority. Both `controller_server` and
`behavior_server` have `cmd_vel` remapped to `cmd_vel_requested`, so the path
is unchanged:

```text
Nav2 / teleop  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel
```

`behavior_server` matters as much as `controller_server`. Its recovery
behaviours drive the robot, and they run precisely when something has already
gone wrong, so an unremapped one is a path around the gate at the worst
moment. `NavigationTopologyTests` asserts the remap on every node that can
emit a velocity, and asserts the gate is not in the lifecycle manager's list:
the lifecycle manager deactivates its nodes on failure, and the gate is the
thing that has to still be running then.

Navigation runs the gate with `safety_navigation.yaml`, which additionally
treats a lost or diverged pose as a stop condition.

### What the simulation is not

The geometry in `robot_description` is a placeholder differential-drive base
with plausible-but-invented dimensions. It is enough to exercise frames,
sensor plumbing, and the safety gate. It is not a model of any robot you own,
and its numbers must be replaced with measured ones before the Nav2
footprint, the inflation radius, or the stopping distances derived from them
mean anything.

The SLAM, AMCL, and Nav2 configuration in `robot_bringup/config` has been
written and is checked for internal consistency by the test suite, but it has
not yet been run against the simulator: this repository's CI has no Gazebo.
Treat the first `slam.launch.py` and `navigation.launch.py` run as bring-up
work, not as a regression test. Expect to tune AMCL and the DWB critics.

## View the robot without a simulator

```bash
ros2 launch robot_description display.launch.py
```

## Run the safety command gate

```bash
ros2 run robot_safety safety_node
```

The node accepts requested motion on `cmd_vel_requested`, laser scans on `scan`, a software stop signal on `emergency_stop`, and a deliberate operator reset on `emergency_stop_reset`. It publishes gated motion on `cmd_vel` and status changes on `safety_state`.

The controller fails closed. It publishes a zero velocity when any of the following is true:

| Condition | Reason reported on `safety_state` |
| --- | --- |
| The software stop is latched | `emergency stop active` |
| No scan has arrived yet | `no obstacle sensor received` |
| The last scan is older than `sensor_timeout` | `obstacle sensor timed out` |
| No motion command has arrived yet | `no motion command received` |
| The last command is older than `command_timeout` | `motion command timed out` |
| The scan contains no usable range | `invalid obstacle sensor reading` |
| An obstacle is within `stop_distance` | `obstacle inside stop distance` |
| Localization is required and no pose has arrived | `no localization received` |
| The last pose is older than `localization_timeout` | `localization timed out` |
| Localization is required and has diverged | `localization uncertainty too high` |
| A battery reserve is set and the level is unknown | `battery level unknown` |
| The battery is at or below the reserve | `battery below reserve` |

The command timeout matters as much as the sensor timeout: if the commanding node dies mid-drive while the path ahead happens to be clear, the gate must not keep replaying the last velocity. A command that times out is discarded rather than resumed.

The software stop is latched. Publishing `false` on `emergency_stop` does **not** release it; only `true` on `emergency_stop_reset` does. This keeps a restarting node that publishes a default `false` from silently releasing a stop that somebody engaged on purpose. The software stop is still secondary to the independently wired physical stop.

The localization and battery checks are off unless configured, because a
robot has to actually have those inputs before demanding them means anything.
Teleop has no pose to lose, and the simulated base has no battery; a gate that
insisted on both would simply refuse to move, and operators would learn to
ignore it. `navigation.launch.py` turns the localization check on, because
under Nav2 the pose is what chooses where to drive.

Parameters: `stop_distance`, `caution_distance`, `sensor_timeout`,
`command_timeout`, `control_rate_hz`, `require_localization`,
`localization_timeout`, `max_localization_covariance`, `low_battery_fraction`.
The last two use a negative value to mean "check disabled", because ROS
parameters have no null and `0.0` is a real value.

### Where the distances come from

`stop_distance` and `caution_distance` are derived, not chosen. They are how
far the robot still travels after the lidar first sees something: the reading
becoming a wheel command, the base braking, and whatever sticks out ahead of
the sensor. The inputs are in `robot_bringup/config/base_dynamics.yaml` and
the arithmetic is in `robot_safety/distances.py`.

Do not edit the distances in `safety.yaml` by hand. `BringupSafetyDistanceTests`
recomputes them from the URDF and the dynamics and fails if the two disagree,
which is what stops the numbers from quietly surviving a change of base,
sensor rate, or speed limit. Change `base_dynamics.yaml`, then update
`safety.yaml` and `safety_navigation.yaml` to match.

Every input there is still an assumption (`measured: false`) matched to the
placeholder URDF.

Do not connect any other node directly to the motor controller's velocity input.

The `robot_safety` package also includes ROS-aware tests for obstacle speed limiting and emergency-stop precedence. They run during the ROS 2 package test phase; the fast Python test command below skips them when ROS 2 is not installed.

## Run fast tests without ROS

From the repository root:

```bash
for package in robot_bringup robot_core robot_description robot_locations \
               robot_navigation robot_safety robot_voice; do
  PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python -m unittest discover -s robot/ros2_ws/src/$package/tests -v || exit 1
done
python -m compileall -q robot/ros2_ws/src
ruff check robot/ros2_ws/src web_ui
```

CI repeats the Python checks, lints, and builds all seven ROS packages on
Jazzy.

## Run the developer web console

The local web console is in `web_ui/`. It runs repeatable safety scenarios,
reports the controller decision and sensor inputs, manages the approved
destination list, and provides a latched emergency stop.

Scenario controls change simulated sensor inputs only. They never command
motor power, and they cannot release an engaged emergency stop: the stop is a
separate control with its own latch, matching the `emergency_stop` and
`emergency_stop_reset` topics on the robot.

Engaging the stop abandons the active trip, and no goal can be sent from the
web or the voice gateway until it is reset.

From the repository root, run:

```bash
PYTHONPATH=robot/ros2_ws/src/robot_core:robot/ros2_ws/src/robot_locations:robot/ros2_ws/src/robot_navigation:robot/ros2_ws/src/robot_safety:robot/ros2_ws/src/robot_voice python3 -m web_ui.app
```

Open <http://127.0.0.1:8080>.

Destinations are saved to `web_ui/locations.json` by map coordinate, entered by
hand. Once localization exists this should become "save the robot's current
pose", because a typed coordinate is only as good as the operator's reading of
the map. Saving a name that already exists replaces it, and the voice gateway
shares the same list, so a place saved here is immediately a valid spoken
destination.

### What is still a stand-in

`DemoGoalDispatcher` and `SafetyScenarioAdapter` are in-memory adapters for
local testing. Nothing here talks to a robot:

- Replace the goal adapter with one that submits `NavigationGoal.pose` to
  `NavigateToPose`, and keep the safety node between navigation output and the
  motor controller.
- Replace the safety adapter with one that publishes to `emergency_stop` and
  `emergency_stop_reset` and reads the real `safety_state`.
- The console has no map display and no live robot pose, so "robot state" here
  means the scenario you selected, not what a robot is doing.

The console binds to `127.0.0.1` with no authentication. It is a developer
tool, not something to expose to a household or a care team.
