"""Bring up the test room, the robot, and the safety gate in Gazebo Harmonic.

Nothing here commands motion. Drive the robot with:

    ros2 launch robot_bringup teleop.launch.py

which publishes to /cmd_vel_requested. The safety node is the only publisher
on /cmd_vel, which is what the Gazebo DiffDrive plugin subscribes to.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    description_share = Path(get_package_share_directory("robot_description"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))

    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")

    robot_description = ParameterValue(
        Command(["xacro ", str(description_share / "urdf" / "robot.urdf.xacro")]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=str(bringup_share / "worlds" / "test_room.sdf"),
            description="Gazebo world file to load.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start RViz."
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description=(
                "Run Gazebo as a server with no GUI. Required where there is "
                "no display: the GUI aborts on the missing Qt platform plugin "
                "and takes the server down with it, leaving `create` retrying "
                "for a world that will never appear."
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": [
                    "-r -v 3 ",
                    PythonExpression(
                        ["'-s ' if '", headless, "'.lower() == 'true' else ''"]
                    ),
                    world,
                ]
            }.items(),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", "home_robot",
                "-x", "-2.0", "-y", "-1.5", "-z", "0.05",
            ],
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            parameters=[{
                "config_file": str(bringup_share / "config" / "ros_gz_bridge.yaml"),
                "use_sim_time": True,
            }],
        ),

        # The gate between anything that wants motion and the wheels.
        Node(
            package="robot_safety",
            executable="safety_node",
            output="screen",
            parameters=[str(bringup_share / "config" / "safety.yaml")],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", str(description_share / "rviz" / "robot.rviz")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
