"""Map from a physical LD14P and expose the live scene to Foxglove.

This is a mapping-only launch: it does not send velocity commands. Move the
robot only with a separately validated and supervised motion controller.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("robot_description"))
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    serial_port = LaunchConfiguration("serial_port")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")

    robot_description = ParameterValue(
        Command(["xacro ", str(description_share / "urdf" / "robot.urdf.xacro")]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial device for the LD14P.",
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="127.0.0.1",
            description="Bind locally by default; use 0.0.0.0 only on a trusted LAN.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket port.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        Node(
            package="ldlidar_sl_ros2",
            executable="ldlidar_sl_ros2_node",
            name="ldlidar_publisher_ld14p",
            output="screen",
            parameters=[{
                "product_name": "LDLiDAR_LD14P",
                "laser_scan_topic_name": "/scan",
                "point_cloud_2d_topic_name": "/pointcloud2d",
                "frame_id": "lidar_link",
                "port_name": serial_port,
                "serial_baudrate": 230400,
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
                "angle_crop_min": 135.0,
                "angle_crop_max": 225.0,
            }],
        ),
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[{
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": True,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
                "use_sim_time": False,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "slam.launch.py")
            ),
            launch_arguments={"use_sim_time": "false"}.items(),
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            output="screen",
            parameters=[{
                "address": ParameterValue(foxglove_address, value_type=str),
                "port": ParameterValue(foxglove_port, value_type=int),
            }],
        ),
    ])
