"""Let Foxglove watch the robot: the lidar scan, the map as it forms, the model.

    ros2 launch robot_bringup viewer.launch.py

Then open Foxglove, choose "Open connection", then "Foxglove WebSocket", and
enter ws://<this machine>:8765. In a Codespace, forward port 8765 and use the
wss:// address the Ports tab gives it instead.

The same launch runs on the Pi for the proof of concept, with the Mac as the
viewer: add `use_sim_time:=false` there, since there is no simulator clock.

WATCH ONLY. Out of the box foxglove_bridge lets every connected client publish
on any topic, call any service, and set any node's parameters. That means a
browser tab could send /cmd_vel_requested, call emergency_stop_reset, or
rewrite stop_distance on the safety gate. All of that is switched off here.
The viewer gets the topic list and the robot's meshes and nothing else. A
viewer that can act is no longer just a viewer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# What a connected viewer may do. The absent ones - clientPublish, services,
# parameters, parametersSubscribe - are the ones that act.
WATCH_ONLY_CAPABILITIES = ["connectionGraph", "assets"]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="8765", description="Port Foxglove connects to."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="True against Gazebo; false on the real robot.",
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "capabilities": WATCH_ONLY_CAPABILITIES,
                # Belt and braces: even if clientPublish came back, no topic
                # matches this, so a client could publish nothing.
                "client_topic_whitelist": ["^$"],
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }],
        ),
    ])
