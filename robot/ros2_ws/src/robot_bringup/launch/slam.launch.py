"""Build a map of the current world with SLAM Toolbox.

Mapping is supervised and manual. Start the simulation, start this, then
drive with teleop.launch.py and watch the map form in RViz:

    ros2 launch robot_bringup simulation.launch.py
    ros2 launch robot_bringup slam.launch.py
    ros2 launch robot_bringup teleop.launch.py

Nothing here commands motion. When the room is covered, save the map
deliberately:

    ros2 run nav2_map_server map_saver_cli -f maps/test_room

Then check the resulting .pgm and .yaml in, so the map Nav2 localizes
against is a reviewed artifact rather than whatever was in RAM that day.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(bringup_share / "config" / "slam_toolbox.yaml"),
            description="SLAM Toolbox parameter file.",
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": True},
            ],
        ),
    ])
