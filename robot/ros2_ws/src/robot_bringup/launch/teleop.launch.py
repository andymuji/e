"""Keyboard teleoperation that respects the safety gate.

Publishes to /cmd_vel_requested, never to /cmd_vel. Run it in its own terminal
because teleop_twist_keyboard needs the focused terminal to read keys.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output="screen",
            prefix="xterm -e",
            remappings=[("/cmd_vel", "/cmd_vel_requested")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
