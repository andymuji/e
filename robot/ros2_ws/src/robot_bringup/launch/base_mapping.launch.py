r"""Drive the mecanum base by keyboard, through the safety gate, and map a room.

On the Pi, in tmux so a dropped ssh session does not end the run:

    ros2 launch robot_bringup base_mapping.launch.py
    ros2 launch robot_bringup base_mapping.launch.py lidar_yaw:=3.1416 \
        serial_port:=/dev/serial/by-id/usb-MicroPython_...

Starts everything car_mapping.launch.py starts - the LD14P lidar, its
position on the base, Cartographer, the recorder and the watch-only Foxglove
viewer - plus the two things that make the wheels turn:

    robot_safety (the gate):  /cmd_vel_requested -> /cmd_vel
    robot_base (the driver):  /cmd_vel -> USB serial -> Pico -> motors

Drive from a SECOND ssh terminal on the Pi:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=cmd_vel_requested

The remap is not optional: it is what puts the keyboard in front of the gate.
Forget it and the keyboard publishes /cmd_vel itself; the driver sees a
second publisher on its topic and holds the wheels at zero, saying why.
teleop.launch.py does the same remap but opens an xterm, which needs a
desktop session, so it only works on the Pi with a screen attached (or with
X forwarding) - not in a plain ssh terminal, where launch cannot hand the
keyboard to a node.

Stop with Ctrl-C in this terminal. The driver sends a final zero and closes
the port; if it cannot, the Pico stops the motors itself 0.25 s after the
last command.

WHAT THE GATE NEEDS BEFORE ANYTHING MOVES. With safety.yaml (the teleop
settings, localization and battery checks off) the gate allows motion only
while all of these hold:
  - a lidar scan no older than 0.5 s - from the LD14P, truthfully;
  - a keyboard command no older than 0.5 s - teleop_twist_keyboard only
    sends one per key press, so the base moves while a key is held and stops
    about half a second after it is released;
  - the software emergency stop has not been latched;
  - nothing closer than stop_distance (0.45 m) IN ANY DIRECTION, and 35%
    speed inside caution_distance (0.85 m). Mount the lidar as the highest
    thing on the base with a clear view all round: any return from the
    chassis, the Pi or a cable closer than 0.45 m is an obstacle to the gate,
    and the base will never move.
Nothing here fakes any of those inputs.

The gate runs on the REAL clock here. safety.yaml says use_sim_time: true,
for the simulator; on the Pi there is no /clock, a simulated clock never
leaves zero, and every age the gate measures would be zero - a lidar that
died after one scan would still look fresh. The override below is what keeps
the sensor and command timeouts meaning anything.

TF. Cartographer publishes map -> odom -> base_link (provide_odom_frame in
car_cartographer.lua), so the driver publishes /odom but no transform:
base.yaml keeps its publish_tf off, or base_link would have two parents.

SAFETY. There is no physical emergency stop on this base yet. Until one is
wired to cut the 12 V motor supply, run this only with the chassis raised so
no wheel touches anything. On the floor, only with the tether, the marked
exclusion zone and a second person holding the stop, per AGENTS.md and
docs/safety-test-procedure.md. The software stop and the Pico's timeout are
secondary to that button, not a substitute for it.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    config = bringup_share / "config"

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="The Pico's USB serial port.",
        ),
        # car_mapping.launch.py reads `label` for the recording's name.
        DeclareLaunchArgument(
            "label",
            default_value="base-mapping",
            description="Name of the recording, under ~/robot_runs/.",
        ),

        # Lidar, its position, Cartographer, recorder, watch-only Foxglove.
        # Its lidar_x/y/z/yaw, lidar_port and record arguments pass through.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "car_mapping.launch.py")
            ),
        ),

        # The one publisher of /cmd_vel. Not under lifecycle management.
        Node(
            package="robot_safety",
            executable="safety_node",
            name="safety_controller",
            output="screen",
            # The override comes last so it wins over safety.yaml's
            # use_sim_time: true. See "real clock" above.
            parameters=[str(config / "safety.yaml"), {"use_sim_time": False}],
        ),

        # The hardware consumer of /cmd_vel.
        Node(
            package="robot_base",
            executable="base_node",
            name="base_driver",
            output="screen",
            parameters=[
                str(config / "base.yaml"),
                {"serial_port": LaunchConfiguration("serial_port")},
            ],
        ),
    ])
