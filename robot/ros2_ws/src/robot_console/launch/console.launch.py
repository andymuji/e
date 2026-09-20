"""The operator console, served against a running robot.

    ros2 launch robot_bringup simulation.launch.py
    ros2 launch robot_bringup navigation.launch.py map:=/path/to/test_room.yaml
    ros2 launch robot_console console.launch.py locations:=/path/to/locations.json

SAFETY: nothing started here can drive the robot. The console publishes no
velocity at all; a chosen destination becomes a Nav2 NavigateToPose goal, and
Nav2's velocity output is already remapped to cmd_vel_requested by
navigation.launch.py, so the gate still decides what reaches the wheels:

    console -> Nav2 goal -> Nav2 -> /cmd_vel_requested -> robot_safety -> /cmd_vel

That is why this file starts neither the gate nor Nav2: the console is one
more motion source, and it is only allowed to ask. Started on its own it
reports that the safety gate is not running, refuses to send goals, and shows
the stop latch as unknown rather than as released.

The console's STOP button publishes `true` on `emergency_stop` and its reset
publishes `true` on `emergency_stop_reset`. The physical emergency stop is
wired independently of all of this.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            "host",
            default_value="127.0.0.1",
            description=(
                "Address the console listens on. Local by default: the page "
                "can engage and release the software stop, so putting it on "
                "the network is a deliberate decision."
            ),
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8080",
            description="Port the console listens on.",
        ),

        Node(
            package="robot_console",
            executable="console_node",
            name="operator_console",
            output="screen",
            parameters=[{
                # The goal stamp and every timeout have to come from the clock
                # the rest of the stack is using, or a healthy system trips
                # them whenever the simulator runs slower than real time.
                "use_sim_time": True,
                "locations_file": LaunchConfiguration("locations"),
                "host": LaunchConfiguration("host"),
                # Typed, because a launch argument is text and the node
                # declares an integer: an untyped one is rejected at start-up.
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            }],
        ),
    ])
