"""Keyboard teleoperation that respects the safety gate.

Publishes to /cmd_vel_requested, never to /cmd_vel. Run it in its own terminal
because the keyboard driver needs the focused terminal to read keys. Arrow
keys drive, q quits; see config/key_teleop.yaml for why key_teleop.

It opens an xterm, which needs a screen. Where there is none (a Codespace, the
Pi over ssh), run the same driver in the terminal itself, from robot/ros2_ws:

    ros2 run key_teleop key_teleop --ros-args -r key_vel:=cmd_vel_requested \\
      --params-file install/robot_bringup/share/robot_bringup/config/key_teleop.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))

    return LaunchDescription([
        Node(
            package="key_teleop",
            executable="key_teleop",
            output="screen",
            prefix="xterm -e",
            remappings=[("key_vel", "cmd_vel_requested")],
            parameters=[str(bringup_share / "config" / "key_teleop.yaml")],
        ),
    ])
