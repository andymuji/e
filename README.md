# Home Robot Capstone

An assistive mobile robot that can map a house, navigate to named locations, avoid obstacles and people, climb stairs, and carry a small payload.

## Scope decision

Treat stair climbing as a separate mechanical subsystem and risk track. A robot that safely climbs stairs is substantially harder than a wheeled indoor robot, so the project should first prove navigation on one floor in simulation and on a controlled flat test area. Do not put a person or an unsecured payload near a prototype until the emergency-stop and low-level safety behavior has been tested.

## Proposed system

```text
Camera / lidar / wheel encoders / IMU
								|
								v
				Sensor and robot drivers
								|
								v
			 Localization + map (ROS 2)
								|
								v
			Planner + obstacle avoidance
								|
								v
			 Motion controller / motors

Voice command -> intent parser -> named location -> navigation goal
Camera during mapping -> optional human-reviewed location suggestion
Web interface -> map, locations, robot state, goal, emergency stop
```

Use ROS 2 as the robotics backbone. Keep hardware drivers, navigation, voice, mapping UI, and stair logic in separate packages or services so each part can be simulated and tested without the complete robot.

## Recommended capstone scope

### Must demonstrate

- A map created from a controlled indoor environment.
- Named locations such as `kitchen` and `front door` shown in a mapping interface.
- A spoken command such as “go to the kitchen” converted into a navigation goal.
- Obstacle avoidance for furniture and a person crossing the robot's path.
- A hard emergency stop that overrides software commands.
- A small payload carried on level ground without changing the robot's safe behavior.

### Stretch goals

- Camera-assisted suggestions for room or object labels during mapping, always requiring human confirmation.
- Multi-floor map handling.
- A stair-climbing module tested first on a fixture or instrumented test rig, then only on stairs with a physical safety tether and a human operator.

### Explicit non-goals for the first version

- Fully autonomous operation around unsupervised people.
- Carrying heavy, fragile, hot, or hazardous items.
- Letting an AI model directly control motor speeds.
- Treating camera labels as authoritative navigation goals.

## Build in vertical slices

1. **Safety and control:** define `Sensor`, `DriveCommand`, and `EmergencyStop` interfaces. In a mock simulator, verify that a close obstacle produces stop or slow behavior and that emergency stop always wins.
2. **Base robot:** drive the robot manually on flat ground. Add motor feedback, battery monitoring, bumper sensing, and a physical emergency stop.
3. **Simulation:** create a small house world and validate localization, mapping, and navigation before risking hardware. Use recorded sensor data where possible.
4. **Autonomous navigation:** integrate a 2D lidar or depth sensor, wheel encoders, and IMU. Start with one room, then a full floor. Tune inflation and stopping distances for humans and furniture.
5. **Locations and interface:** allow an operator to save map poses as named locations, view robot state, send a goal, cancel a goal, and trigger emergency stop.
6. **Voice:** convert speech to a small allow-listed intent set (`go to`, `stop`, `where are you`, `return`). Confirm ambiguous locations instead of guessing.
7. **Carrying:** add a low center-of-gravity tray and test payload limits on level ground. Revalidate braking, turning, and obstacle clearance.
8. **Stairs:** only after the flat-ground system is reliable, evaluate a dedicated stair mechanism with a written hazard analysis, tether, mechanical braking, and a human-in-the-loop test protocol.

## Core ROS 2 repositories

These projects provide the first navigation stack. Pin versions to the chosen ROS 2 distribution instead of tracking the default branch indefinitely.

| Step | Capability | Repository | Integration result |
| --- | --- | --- | --- |
| 1. Map the floor | Build and save a 2D occupancy map from a manually driven robot | [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | A saved map of one controlled floor, with the map file committed or stored as a documented test artifact |
| 2. Locate the robot | Estimate the robot pose from the saved map and live LiDAR data | [Nav2 AMCL](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl) | The robot pose remains stable while driving through the mapped test area |
| 3. Navigate to destinations | Plan a route, follow it, and replan around newly detected obstacles | [Nav2](https://github.com/ros-navigation/navigation2) | A goal sent in RViz or the operator interface reaches a named location without entering configured keep-out margins |
| 4. Avoid moving humans | Apply speed limits or stop behavior from live sensor observations | [Nav2 Collision Monitor](https://github.com/ros-navigation/navigation2/tree/main/nav2_collision_monitor) | A person or close obstacle causes the robot to slow or stop, then resume or replan only after the path is clear |
| 5. Add simple controls | Send navigation goals from a small Python control layer | [Nav2 Simple Commander](https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander) | Buttons can save and send named goals such as `kitchen` and `sofa`, cancel a goal, and report success or failure |

### Integration order

1. Bring up the robot description, LiDAR, odometry, transforms, and manual teleoperation.
2. Run SLAM Toolbox and save a map while driving slowly around one floor.
3. Load that map into Nav2 AMCL and verify localization before enabling autonomous motion.
4. Configure Nav2's robot footprint, inflation radius, maximum speeds, stopping behavior, and recovery actions from measured robot dimensions.
5. Add Collision Monitor as an independent safety layer. Test sensor timeouts and stop behavior before testing people.
6. Wrap Nav2 Simple Commander in the project's `robot_locations` package and expose only allow-listed actions to voice and web clients.

Do not use the voice service or camera-based suggestions as a substitute for localization or obstacle sensing. They may request a named goal, but Nav2 and the safety controller decide whether and how the robot moves.

## Suggested repository layout

```text
robot/
	ros2_ws/src/
		robot_description/       # URDF, meshes, transforms
		robot_bringup/            # launch files and parameters
		robot_base/               # motors, encoders, battery, e-stop
		robot_safety/             # speed limits, stopping rules, watchdogs
		robot_navigation/         # mapping, localization, planning config
		robot_voice/              # speech-to-intent adapter and destination gating
		robot_locations/          # named poses and map metadata
		robot_interfaces/         # shared messages and service definitions
	simulation/                 # realized as robot_bringup/worlds so colcon installs it
	web_ui/                     # map and operator controls
	hardware/                   # wiring, bills of materials, CAD references
	docs/                       # decisions, hazards, test procedures
	tests/                      # unit, integration, and scenario tests
```

## Hardware selection principles

Choose the platform around the safety problem, not just the motor torque:

- Differential-drive or similarly stable base for the first floor-navigation prototype.
- 2D lidar for reliable planar obstacle detection, plus depth camera if needed for people and payload awareness.
- Wheel encoders and IMU for odometry.
- Physical normally-closed emergency-stop circuit that removes motor power.
- Bumper or contact sensors, motor current monitoring, battery protection, and a watchdog.
- Onboard computer capable of running ROS 2 and navigation locally; do not require cloud connectivity for stopping or driving.

The stair mechanism should be selected only after the team has documented stair dimensions, robot mass, center of gravity, traction, failure modes, and a recovery procedure. A separate tracked or legged stair robot may be a better research prototype than modifying a flat-floor base late in the project.

## Safety invariants

- Loss of command, sensor timeout, localization failure, or low battery causes a controlled stop. All four are implemented in `robot_safety`; the last two are enabled by configuration once the robot has a pose and a battery to report.
- Emergency stop is physical, latched, and independent of the network and AI services.
- Voice and vision may request goals, but never bypass the safety controller or directly set motor power.
- Human detection reduces speed and increases stopping distance; it does not guarantee safe operation by itself.
- Every hardware test has a tether or exclusion zone appropriate to the failure mode.

## GitHub workflow

Track each vertical slice as an issue with acceptance tests. Use pull requests for hardware and software changes, and attach a short test recording or log for behavior changes. Keep a decision record for the ROS 2 distribution, simulator, sensors, base platform, voice provider, and stair strategy.

Suggested first issues:

1. Choose ROS 2 distribution, simulator, language, and license.
2. Write the hazard analysis and emergency-stop test procedure.
3. ~~Implement the mock obstacle-stop controller with unit tests.~~ **Completed:** the first hardware-independent safety slice is in `robot/ros2_ws/src/robot_safety`.
4. Select the flat-floor base, sensors, compute board, and power system.
5. ~~Create the ROS 2 workspace and a minimal simulated house.~~ **Completed:** `robot_description` holds a differential-drive URDF with a 2D lidar and IMU, `robot_bringup` holds the `test_room` Gazebo Harmonic world and the launch files that start the simulator, the topic bridge, and the safety gate. The URDF dimensions are placeholders pending the hardware choice.
6. ~~Add CI that runs formatting, unit tests, and package builds.~~ **Partially completed:** GitHub Actions now runs the Python unit tests and compilation checks; ROS 2 package builds will be added after the ROS 2 baseline is selected.

### Current implementation

The first safety controller is implemented without a ROS 2 dependency so it can be tested in this repository immediately. It converts the nearest obstacle reading into a bounded decision:

- `clear`: full speed allowed when the path is outside the caution distance.
- `caution`: speed limited to 35% when an obstacle is nearby.
- `stop`: zero speed for a close obstacle, a latched emergency stop, invalid or stale sensor data, or a stale motion command.

The emergency stop latches: once engaged it holds until a deliberate reset, so a clear sensor reading or a restarting publisher cannot release it. A command stream that goes silent is treated as a stop condition rather than a reason to keep the last requested velocity.

Run its unit tests with:

```bash
PYTHONPATH=robot/ros2_ws/src/robot_safety python3 -m unittest discover -s robot/ros2_ws/src/robot_safety/tests -v
```

The physical test procedure is documented in [docs/safety-test-procedure.md](docs/safety-test-procedure.md). The next implementation slice is to wrap this controller in a ROS 2 package after the team selects the ROS 2 distribution and robot base.

### Simulation

`ros2 launch robot_bringup simulation.launch.py` starts Gazebo Harmonic with a
6x5 m test room, spawns the robot, bridges sensors and commands, and runs the
safety gate. Teleop drives it through `/cmd_vel_requested`:

```text
teleop / Nav2  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel  ->  Gazebo DiffDrive
```

Only `robot_safety` publishes `/cmd_vel`, and only `/cmd_vel` is bridged into
the simulator, so there is no path from a motion source to the wheels that
skips the gate. The room contains a table and a cabinet to stop for.

This is the first slice where the safety controller runs as a live node rather
than as unit-tested logic.

SLAM Toolbox, AMCL, and Nav2 are now configured and launchable
(`slam.launch.py`, `navigation.launch.py`), with Nav2 wired as a motion
source behind the gate rather than as a motion authority. That configuration
is checked for internal consistency by the test suite but has not been run
against the simulator yet, because CI here has no Gazebo. The first run is
bring-up work, not a regression test.

The URDF describes a plausible indoor base, not a robot anyone owns. Its
dimensions are placeholders, and Nav2 footprints, inflation radii, and the
safety distances all derive from them, so they must be replaced with measured
values before they mean anything.

### Derived safety numbers

The safety distances and the Nav2 costmap geometry are computed from the base,
not typed in. `robot_safety/distances.py` turns the speed, braking, sensor
rate, control rate, and sensor overhang in `robot_bringup/config/base_dynamics.yaml`
into `stop_distance` and `caution_distance`; `robot_navigation/footprint.py`
turns the URDF's `base_length` and `base_width` into the costmap footprint and
inflation radius.

The tests recompute both and fail if the shipped configuration disagrees. That
is the mechanism that stops a margin from quietly surviving a change of base,
sensor, or speed limit: raising Nav2's top speed without re-deriving the stop
distance is a build failure rather than a robot that outruns its own margin.

Every input is still an assumption (`measured: false`) matched to the
placeholder URDF.

### Additional completed software foundations

- `robot_core` defines validated `Pose2D` and `NavigationGoal` data contracts.
- `robot_locations` persists operator-approved named poses as JSON and rejects unknown destinations.
- `robot_voice` parses only a small allow-listed command set and returns `unknown` for ambiguous commands. Its `CommandGateway` then resolves each parsed intent against the approved locations, so an unrecognised destination becomes a spoken refusal rather than a goal, and a stop word anywhere in the transcript wins over everything else.
- `robot_navigation` provides a deterministic grid planner for simulator and integration tests, including obstacle detours, replanning, and no-path errors.
- `robot_description` and `robot_bringup` provide the simulated robot, the test world, and the launch wiring, with tests that assert the safety topology rather than only the geometry.
- `robot_safety` additionally treats localization failure and a flat battery as stop conditions, off until the robot has those inputs to lose.
- `.github/workflows/tests.yml` lints, runs all unit tests, compiles the Python, and builds the ROS packages on Jazzy, on pushes and pull requests.

These modules are deliberately independent of ROS 2 so the behavior can be tested in this repository. They are not a replacement for SLAM Toolbox, AMCL, Nav2, Collision Monitor, a speech-to-text engine, or physical safety hardware.

## Definition of done for the first milestone

In simulation and in a controlled flat-floor test, a robot stops before a configured obstacle, responds to a physical emergency stop, and can be commanded to a single named location. The behavior is reproducible from a documented setup and produces logs that the team can inspect.

## Development baseline and quick start

The first implementation targets **Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic**. The repository now contains seven installable ROS 2 Python packages and a fail-closed `robot_safety` command gate.

Start with [the build and run guide](docs/getting-started.md). The rationale and safety consequences are recorded in [ADR 0001](docs/decisions/0001-ros-baseline.md).
