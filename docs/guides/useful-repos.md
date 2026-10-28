# Useful outside projects

Other people's code and Claude add-ons worth knowing about, grouped by when
this project will need them. Every entry was checked on **2026-10-18**: it
exists, it has been updated recently, and its license is noted. Projects
change; if an entry is much older than that date, check it again before
relying on it.

**License key.** This project is Apache-2.0. Code under Apache, MIT, BSD or
MPL can be used. **GPL / AGPL** code may be *run* as a separate program or read
for ideas, but must not be copied into this repository.

**Safety key.** Anything that can send a driving command must go through
`robot_safety` (see the rules in [AGENTS.md](../../AGENTS.md)). Entries that
could skip the gate say so.

## Now: drive the base and map a room

| Project | What it is for | License | Notes |
|---|---|---|---|
| [ldrobotSensorTeam/ldlidar_ros2](https://github.com/ldrobotSensorTeam/ldlidar_ros2) | The LD14P lidar driver we already use | MIT | Unmaintained since 2023 but the only LD14P driver. `robot/scripts/setup-pi.sh` and `car_mapping.launch.py` already patch its two known problems. |
| [linorobot/linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) | Ready-made Pico motor program: 4 mecanum wheels, encoders, **BTS7960 drivers** built in | Apache-2.0 | `jazzy` branch, `MECANUM` + `USE_BTS7960_MOTOR_DRIVER`. Listens on `/cmd_vel` (which only the gate publishes) and stops the motors after 200 ms without a command. Would replace our `robot_base` driver: a decision to make, not yet made. |
| [linorobot/linorobot2](https://github.com/linorobot/linorobot2) | A complete mecanum robot (bringup, SLAM, Nav2, Gazebo) to compare against | Apache-2.0 | Partner to the firmware above. |
| [micro-ROS/micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) | The bare micro-ROS library for the Pico | Apache-2.0 | Only if writing the motor program ourselves. |
| [Ekumen-OS/andino](https://github.com/Ekumen-OS/andino) | A small Pi 4 robot with simple serial motor firmware | BSD-3 | Two wheels only; useful as a Pi 4 + Jazzy reference. |
| [turtlebot/turtlebot4_setup](https://github.com/turtlebot/turtlebot4_setup) | Pi 4 + Ubuntu 24.04 + Jazzy setup script (`jazzy` branch) | Apache-2.0 | Cross-check for our own `setup-pi.sh` if something breaks. |
| [LihanChen2004/pcd2pgm](https://github.com/LihanChen2004/pcd2pgm) | Turns a point cloud into a Nav2 map | Apache-2.0 | A second opinion on `scan_to_map.py`. Our script is still the shortest path from a phone scan. |
| [isl-org/Open3D](https://github.com/isl-org/Open3D) | Python library that reads phone scans (GLB/PLY) | MIT | The step between a Polycam scan and pcd2pgm. |
| [CloudCompare/CloudCompare](https://github.com/CloudCompare/CloudCompare) | Desktop app to view, crop and convert scans | **GPL-2.0** | Use as an app only. |

## Next: the first goal reached

| Project | What it is for | License | Notes |
|---|---|---|---|
| [ros-navigation/nav2_minimal_turtlebot_simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation) | Nav2's own known-good Gazebo Harmonic robot | Apache-2.0 | **Start here.** Reach a goal in it, then compare our launch files and parameters. |
| [ros-navigation/navigation2_tutorials](https://github.com/ros-navigation/navigation2_tutorials) | Code for the Nav2 "set up your own robot" guide | Apache-2.0 | |
| [ros-controls/gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) | Connects motor controllers to Gazebo Harmonic | Apache-2.0 | |
| [gazebosim/ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template) | Official layout for a ROS + Gazebo project | Apache-2.0 | Pattern for the scanned-room digital twin. |
| [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers) | Includes a mecanum drive controller | Apache-2.0 | Only on the ros2_control route. |
| [ros2/launch](https://github.com/ros2/launch) (`launch_testing`) | Tests that start nodes and check what they publish | Apache-2.0 | E.g. one test: feed the gate a command, stop it, check `/cmd_vel` goes to zero. |
| [ros2/rosbag2](https://github.com/ros2/rosbag2) | Record and replay runs | Apache-2.0 | Replay a recorded run into a test. |
| [ros-tooling/action-ros-ci](https://github.com/ros-tooling/action-ros-ci) | GitHub Action that builds and tests ROS code | Apache-2.0 | |

## Later: voice, seeing people, learning from others

| Project | What it is for | License | Notes |
|---|---|---|---|
| [ggml-org/whisper.cpp](https://github.com/ggml-org/whisper.cpp) | Offline speech-to-text that runs on a Pi | MIT | |
| [mgonzs13/whisper_ros](https://github.com/mgonzs13/whisper_ros) | ROS 2 nodes wrapping whisper.cpp | MIT | |
| [alphacep/vosk-api](https://github.com/alphacep/vosk-api) | Lighter offline speech recognition | Apache-2.0 | The most Pi-friendly option. |
| [SYSTRAN/faster-whisper](https://github.com/SYSTRAN/faster-whisper) | Python Whisper | MIT | Heavier on a Pi. |
| [dscripka/openWakeWord](https://github.com/dscripka/openWakeWord) | Wake word ("hey robot") | Apache-2.0 | Some ready-made wake-word models are non-commercial. |
| [OHF-Voice/piper1-gpl](https://github.com/OHF-Voice/piper1-gpl) | Local text-to-speech | **GPL-3.0** | Run as a separate program, never copied in. |
| [google-ai-edge/mediapipe](https://github.com/google-ai-edge/mediapipe) | Body-pose tracking on a Pi 4 | Apache-2.0 | Fall detection = a simple rule on top (hips drop fast, body goes flat). No ready-made ROS fall detector exists. |
| [PINTO0309/PINTO_model_zoo](https://github.com/PINTO0309/PINTO_model_zoo) | Small vision models converted for edge devices | MIT (models vary) | |
| [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) | ROS 2 person detection | MIT, but uses **AGPL** Ultralytics | Read for design ideas only. |
| [hello-robot/stretch_ros2](https://github.com/hello-robot/stretch_ros2) | ROS 2 stack of a real home-assistance robot | Apache-2.0 | The closest project to this one. |
| [hello-robot/stretch_ai](https://github.com/hello-robot/stretch_ai) | Stretch's AI layer for home tasks | Apache-2.0 | |

## Claude add-ons

Install these for yourself (user scope), not into the project settings.

| Add-on | What it lets Claude do | License | Safety |
|---|---|---|---|
| [dbwls99706/ros2-engineering-skills](https://github.com/dbwls99706/ros2-engineering-skills) | Know ROS 2: robot models, Nav2, launch files, ros2_control | Apache-2.0 | Advice only. Its optional hooks run on every tool call; read them before turning them on. It may suggest standard Nav2 wiring; ours sends every velocity to `cmd_vel_raw`. |
| [binabik-ai/mcp-rosbags](https://github.com/binabik-ai/mcp-rosbags) | Read and plot recorded runs (`~/robot_runs`) | Apache-2.0 | Reads files only. Safest. |
| [wise-vision/ros2_mcp](https://github.com/wise-vision/ros2_mcp) | Look inside the live robot software | MPL-2.0 | **Always run read-only** (`ROS2_MCP_READONLY=1`); otherwise it can publish `/cmd_vel`. Needs the full ROS container. |
| [arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills) | More ROS 2 / bringup / testing skills | Apache-2.0 | Advice only; overlaps the first entry, so pick one. |
| [alexex1993/mcu-skills](https://github.com/alexex1993/mcu-skills) | Pico (RP2040) firmware know-how | MIT | Only if we write our own motor program. |
| [anthropics/skills](https://github.com/anthropics/skills) | `webapp-testing` for the console and tablet page | per skill | |

**Do not install** [robotmcp/ros-mcp-server](https://github.com/robotmcp/ros-mcp-server),
[kvgork/gazebo-mcp](https://github.com/kvgork/gazebo-mcp) or
[Ranch-Hand-Robotics/rde-mcp-ros-2](https://github.com/Ranch-Hand-Robotics/rde-mcp-ros-2)
anywhere that can reach the robot: all three can send driving commands that
skip the safety gate, and none can be locked to watch-only.

## Checked and left out

Abandoned or archived: `rhasspy/piper` (moved to the GPL repo above),
`dusty-nv/ros_deep_learning`, `ros-safety/software_watchdogs`,
`hbrobotics/ros_arduino_bridge`, `joshnewans/articubot_one`,
`joshnewans/diffdrive_arduino`. GPL and stale: `LNSterne/LD14P_RP2040`. No
LD14P support: `Myzhar/ldrobot-lidar-ros2` (LD19/LD06 only).
