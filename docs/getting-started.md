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

A map saved without driving the robot around covers only what the lidar saw
from the spawn point, and a goal outside that patch is rejected with "Start
Coordinates ... outside bounds" - so cover the room before saving. This is
worse than it sounds. A map saved from a standing robot was measured: 1,316
free cells against 10,463 unknown ones, and the free cells are a speckled fan
rather than a floor, because at 5 m the beams of a 1-degree lidar are 9 cm
apart and the cells between them are 5 cm. One scan does not make a room.

Note also that SLAM anchors the map at the robot's starting pose: the initial
pose you give AMCL is in map coordinates, which are not the Gazebo world
coordinates the robot was spawned at.

### The committed map is generated, not driven

`maps/test_room.yaml` is rendered from `worlds/test_room.sdf` by

```bash
ros2 run robot_bringup world_to_map \
  robot/ros2_ws/src/robot_bringup/worlds/test_room.sdf -f maps/test_room
```

It exists so that `navigation.launch.py` has something to load, and it is
exact where a driven map is honest: it contains the room's geometry with no
sensor, no drift, and no shadow behind the table. **It is not evidence that
anything has been mapped or navigated.** Because it comes from the world file
its frame is the Gazebo world frame, which is why `nav2.yaml` can start AMCL
at the spawn pose; a driven map would be anchored at the robot's start
instead, where the robot begins at the origin.

Replace it with a driven map when a run can be completed, and change the AMCL
`initial_pose` in `nav2.yaml` to match the new frame when you do. The test
`MapGeneratorTests` checks the committed map against the world file, so
changing the room without rebuilding the map fails the suite; delete that one
test when the map stops being generated.

## Navigate to a goal

Start the simulation with its own gate turned off, because this launch brings
one:

```bash
ros2 launch robot_bringup simulation.launch.py safety:=false
ros2 launch robot_bringup navigation.launch.py
```

This starts AMCL, Nav2, and the safety gate. `map:=` defaults to the committed
map of the test room; pass a path to use another.

`safety:=false` matters, and it is easy to misread as weakening safety. It is
the opposite. Both launch files start a gate, and run together they both
subscribe to `cmd_vel_requested` and both publish `cmd_vel`. The gate this
launch brings is the strict one - it adds the localization and battery checks -
so the simulation's gate would go on commanding motion while this one is trying
to stop, and the laxer of the two would win. **The flag chooses which gate is
in charge, not whether there is one.** Leave the simulation's gate on when
driving by teleop, and off whenever Nav2 is coming - and never turn it off in
any other arrangement, or reconstruct this topology by hand.

Automated tooling has repeatedly refused this argument on the strength of its
name, which is why the first goal-reaching run has still not happened. If you
are running the commands yourself, this is the supported arrangement and the
only one.

AMCL starts at the pose the simulation spawns the robot at, so the stack comes
up localized without RViz. That only works while the committed map shares the
Gazebo world frame; against a driven map, set the initial pose in RViz, or
change `initial_pose` in `nav2.yaml`. Then send a goal - through Nav2, which is
the only sanctioned way to ask for motion:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, \
   orientation: {w: 1.0}}}}"
```

While the map is still being built there is nothing for AMCL to localize
against, so `slam:=true` drops map_server and AMCL and takes `map->odom` from
a running `slam.launch.py` instead. That is how to drive the robot under goals
while mapping, without putting a planner into `slam.launch.py`, which must
launch nothing that can drive:

```bash
ros2 launch robot_bringup slam.launch.py
ros2 launch robot_bringup navigation.launch.py slam:=true
```

Nav2 is a motion source, not a motion authority. Both `controller_server` and
`behavior_server` have `cmd_vel` remapped away from the wheels, onto the
Collision Monitor's input, and the monitor forwards what survives to the gate:

```text
teleop  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel
Nav2  ->  /cmd_vel_raw  ->  collision_monitor  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel
```

Two layers, and they are not the same thing. The monitor is a *constraint*: it
reads the scan directionally and can only reduce a velocity already asked for.
The gate is the *authority*: the only publisher of `/cmd_vel`, applying its own
blunter, omnidirectional check afterwards. The monitor being there is not a
reason to relax the gate, and if the monitor dies nothing reaches
`cmd_vel_requested` at all, the gate's `command_timeout` fires, and the robot
stops.

Measured on a running stack (2026-09-23): `/cmd_vel_raw` had 4 publishers -
`controller_server` once and `behavior_server` three times, one per recovery
behaviour - `/cmd_vel_requested` had exactly one publisher, the monitor, and
`/cmd_vel` exactly one, `safety_controller`.

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

The SLAM, AMCL, and Nav2 configuration in `robot_bringup/config` has now been
run against the simulator once, and that first run found four things the test
suite could not see: Gazebo's GUI aborts where there is no display and takes
the server with it (hence `headless:=true`), `slam_toolbox` is a lifecycle
node that sat in `unconfigured` forever when launched as a plain node, the
Nav2 planner plugin was named in Humble's form and aborted the whole bringup,
and the gate took localization freshness from a topic AMCL stops publishing
when the robot stands still, which deadlocked it. All four are fixed.

A second run went further and checked the claims this project makes about its
own topic graph, rather than checking the files that are supposed to produce
it. Against a live system:

- `/cmd_vel` had exactly one publisher, the safety gate. This is the
  repository's central safety claim and it had never been observed before.
- `controller_server` and `behavior_server` both published to
  `/cmd_vel_requested` and neither published `/cmd_vel`. `behavior_server`
  showed three publishers on the request topic, one per recovery behaviour -
  spin, back up and wait - which is three more paths the remap has to cover
  than the one a reading of the launch file suggests.
- The lifecycle manager's `node_names`, read from the running node rather than
  from the file, did not contain the gate.
- `slam_toolbox` reached `active`, published `map->odom`, and `map_saver_cli`
  wrote a map, so the lifecycle fix from the first run holds.
- `map_server` loaded the committed map and activated, and AMCL with it.

Two things were learned the hard way. The simulator's lidar arrives at about
7.5 Hz rather than the 10 Hz the sensor is configured for, because the
container runs Gazebo at roughly three-quarters of real time; `sensor_timeout`
is 0.5 s and the observed worst gap was 0.23 s, so there is margin, but a
slower machine would start tripping the gate on nothing. And ROS 2 defaults to
domain 0, so a second stack running anywhere on the same machine joins the
same graph: during this run `/cmd_vel` briefly showed two publishers, both
named `safety_controller`, one of which belonged to somebody else's workspace.
Set `ROS_DOMAIN_ID` before believing any count of publishers.

**Nothing has navigated anywhere yet.** No robot has been driven under a goal,
no map has been built by driving, and the DWB critics and AMCL have never been
tuned against a moving robot, because that run has not been completed. Expect
the first one to need tuning, and expect it to find things, as both runs so
far have.

CI still has no Gazebo, so none of this is a regression test - the suite
checks the configuration for internal consistency and nothing more. It now
also checks that the committed map exists, is loadable, is installed, and
still matches the world file, which catches a stale map but says nothing about
whether a robot could follow it. Run the simulator headless where there is no
display:

```bash
ros2 launch robot_bringup simulation.launch.py rviz:=false headless:=true
```

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

Do not edit the distances in `safety.yaml` by hand. `SafetyDistanceDerivationTests`
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
