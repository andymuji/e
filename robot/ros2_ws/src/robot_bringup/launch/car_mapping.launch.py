"""Map a real room from the proof-of-concept RC car, on the Pi.

    ros2 launch robot_bringup car_mapping.launch.py
    ros2 launch robot_bringup car_mapping.launch.py label:=stage5-map lidar_yaw:=3.1416

Starts the LD14P lidar driver, Cartographer building a map from the lidar
alone, the recorder, and Foxglove (watch-only, via viewer.launch.py). All on
the real clock: there is no simulator, and slam.launch.py, which waits for
one, would wait forever here. See docs/poc-plan.md.

Run it in a foreground terminal (tmux on the Pi, so a dropped ssh session
does not end the run) and stop it with Ctrl-C. Killed any other way, the
recording can be left without its metadata.yaml.

When the map covers the room, save it from a second terminal:

    ros2 run nav2_map_server map_saver_cli -f maps/<room>

SAFETY: the car is driven by a person with its own radio remote, outside
robot_safety. Nothing started here can move it: the Pi only listens to the
lidar, and must never be connected to the car's motor controller or radio.

DEPENDENCY: LDRobot's driver, ldlidar_ros2, is not in rosdep or apt. Clone it
with its SDK submodule into robot/ros2_ws/src, and build it on its own. On
Ubuntu 24.04 its SDK misses an include, so it needs the extra flag:

    git clone --recursive https://github.com/ldrobotSensorTeam/ldlidar_ros2
    colcon build --packages-select ldlidar_ros2 \\
        --cmake-args -DCMAKE_CXX_FLAGS="-include pthread.h"

Its own ld14p.launch.py is deliberately NOT used: it also publishes
base_link -> base_laser at a made-up height, and a second publisher of the
lidar's position would fight the one below.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same frame name as the lidar on the robot description, so Foxglove layouts
# and recordings read the same on the car and in simulation.
LIDAR_FRAME = "lidar_link"


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    telemetry_share = Path(get_package_share_directory("robot_telemetry"))

    # Where the lidar sits on the car. PLACEHOLDERS: measure in Stage 3 and
    # pass them here. Never put them in the URDF or base_dynamics.yaml - those
    # describe the real robot, and its safety distances are derived from them.
    offsets = [
        DeclareLaunchArgument(
            f"lidar_{axis}",
            default_value="0.0",
            description=(
                f"Lidar {axis} from the car's centre ({unit}). "
                "PLACEHOLDER: measure in Stage 3."
            ),
        )
        for axis, unit in [("x", "m"), ("y", "m"), ("z", "m"), ("yaw", "rad")]
    ]

    return LaunchDescription([
        *offsets,
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0",
            description="The LD14P's USB serial port.",
        ),
        DeclareLaunchArgument(
            "record",
            default_value="true",
            description=(
                "Record the run with robot_telemetry, so the map can be "
                "rebuilt from it."
            ),
        ),

        # The ONE publisher of the lidar's position on the car.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_lidar",
            arguments=[
                "--x", LaunchConfiguration("lidar_x"),
                "--y", LaunchConfiguration("lidar_y"),
                "--z", LaunchConfiguration("lidar_z"),
                "--yaw", LaunchConfiguration("lidar_yaw"),
                "--frame-id", "base_link",
                "--child-frame-id", LIDAR_FRAME,
            ],
            parameters=[{"use_sim_time": False}],
        ),

        # Parameters as in LDRobot's ld14p.launch.py, except the frame. The
        # driver leaves any it is not given uninitialised, so all are set.
        Node(
            package="ldlidar_ros2",
            executable="ldlidar_ros2_node",
            name="ldlidar_publisher_ld14p",
            output="screen",
            parameters=[{
                "product_name": "LDLiDAR_LD14P",
                "laser_scan_topic_name": "scan",
                "point_cloud_2d_topic_name": "pointcloud2d",
                "frame_id": LIDAR_FRAME,
                "port_name": LaunchConfiguration("lidar_port"),
                "serial_baudrate": 230400,
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
                "angle_crop_min": 135.0,
                "angle_crop_max": 225.0,
                "range_min": 0.02,
                "range_max": 8.0,
                "use_sim_time": False,
            }],
        ),

        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": False}],
            arguments=[
                "-configuration_directory", str(bringup_share / "config"),
                "-configuration_basename", "car_cartographer.lua",
            ],
        ),
        # Turns Cartographer's submaps into the /map Foxglove shows and
        # map_saver_cli saves.
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": False, "resolution": 0.05}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "viewer.launch.py")
            ),
            launch_arguments={"use_sim_time": "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(telemetry_share / "launch" / "record.launch.py")
            ),
            # The recorder's own `label` argument still wins when given.
            launch_arguments={
                "label": LaunchConfiguration("label", default="car-mapping")
            }.items(),
            condition=IfCondition(LaunchConfiguration("record")),
        ),
    ])
