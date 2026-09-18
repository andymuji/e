"""Voice commands on top of a running simulation and navigation stack.

    ros2 launch robot_bringup simulation.launch.py
    ros2 launch robot_bringup navigation.launch.py map:=/path/to/test_room.yaml
    ros2 launch robot_bringup voice.launch.py locations:=/path/to/locations.json

SAFETY: nothing started here can drive the robot. The voice node publishes no
velocity at all; an approved destination becomes a Nav2 NavigateToPose goal,
and Nav2's velocity output is already remapped to cmd_vel_requested by
navigation.launch.py, so the gate still decides what reaches the wheels:

    speech -> Nav2 goal -> Nav2 -> /cmd_vel_requested -> robot_safety -> /cmd_vel

That is why this file starts neither the gate nor Nav2: voice is one more
motion source, and it is only allowed to ask. Started on its own it will
answer out loud that navigation is not running, and send nothing.

A heard "stop" engages the latched software emergency stop and cancels the
trip. Releasing the latch is an operator action on `emergency_stop_reset`,
which no node here publishes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "locations",
            description=(
                "JSON file of operator-approved named poses. Destinations "
                "outside it are refused, so it is required rather than "
                "defaulted, exactly like the map."
            ),
        ),
        DeclareLaunchArgument(
            "home_location",
            default_value="",
            description=(
                "Which approved location 'go home' means. Empty leaves "
                "'home' unapproved, and so refused."
            ),
        ),

        Node(
            package="robot_voice",
            executable="voice_node",
            name="voice_commander",
            output="screen",
            parameters=[{
                # The goal stamp has to come from the clock Nav2 is using,
                # or the pose cannot be transformed when it arrives.
                "use_sim_time": True,
                "locations_file": LaunchConfiguration("locations"),
                "home_location": LaunchConfiguration("home_location"),
            }],
        ),
    ])
