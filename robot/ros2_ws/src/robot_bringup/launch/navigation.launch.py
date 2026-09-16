"""Autonomous navigation: AMCL on a saved map, then Nav2.

    ros2 launch robot_bringup simulation.launch.py
    ros2 launch robot_bringup navigation.launch.py map:=/path/to/test_room.yaml

SAFETY: Nav2 is a motion source, not a motion authority. Every node below
that can emit a velocity has cmd_vel remapped to cmd_vel_requested, so its
output is a request the safety gate decides on:

    Nav2 / teleop  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel

That includes behavior_server. Its recovery behaviours drive the robot, and
they run precisely when something has already gone wrong, so an unremapped
behavior_server is a path around the gate at the worst possible moment.
BringupNavigationTopologyTests asserts the remap on every one of them.

The gate runs with safety_navigation.yaml rather than safety.yaml, which
additionally treats a lost or diverged pose as a stop condition. Under teleop
a human is watching; under Nav2 the pose is what chooses where to drive.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Every Nav2 node that can put a velocity on the wire. Adding a node that
# drives the robot means adding it here too.
VELOCITY_SOURCES = ("controller_server", "behavior_server")

# Nav2 nodes the lifecycle manager brings up, in start order.
MANAGED_NODES = [
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    nav2_params = str(bringup_share / "config" / "nav2.yaml")
    safety_params = str(bringup_share / "config" / "safety_navigation.yaml")

    map_yaml = LaunchConfiguration("map")

    # The one remap that keeps Nav2 behind the gate.
    gated = [("/cmd_vel", "/cmd_vel_requested")]

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Occupancy map .yaml saved from slam.launch.py.",
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[nav2_params, {"yaml_filename": map_yaml}],
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=gated,
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
            remappings=gated,
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
        ),

        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params],
        ),

        # The gate, with the localization checks that only apply once
        # something other than a human is steering.
        Node(
            package="robot_safety",
            executable="safety_node",
            name="safety_controller",
            output="screen",
            parameters=[safety_params],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": MANAGED_NODES,
            }],
        ),
    ])
