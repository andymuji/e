"""Rebuild a room's map from a recording, when the live map went wrong.

    ros2 launch robot_bringup map_from_recording.launch.py \\
        recording:=~/robot_runs/car-mapping-20261004-101500 room:=living_room

Plays the lidar scans and the lidar's position from a recording made by
car_mapping.launch.py back through Cartographer, with the same settings as
the live run, then saves the result with `ros2 run robot_bringup save_map`
and stops by itself. It takes as long as the drive did (rate:=2.0 halves
that, on a computer fast enough). Add force:=true to replace a map of the
same name. Another Cartographer file can be given with
configuration_basename:=<file>.lua.

ISOLATED ON PURPOSE. Everything here - the replayed scans, their clock, and
the map being rebuilt - lives under /map_rebuild, never on /scan, /clock,
/tf or /map. A robot on the same network and ROS_DOMAIN_ID would otherwise
take the replayed scans for what its lidar sees now, and its safety gate
would judge a live motion against a room from the past. In Foxglove the map
being rebuilt is /map_rebuild/map.

Only /scan and /tf_static are played. A recording also holds /cmd_vel,
/cmd_vel_requested and /emergency_stop_reset; playing those would drive a
robot, or release its stop, from a file. /tf is left out too: on the car it
is the live Cartographer's own output, which would fight the rebuild's.
"""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_bringup.save_map import (
    RUNS_DIR,
    check_free,
    check_recording,
    check_room_name,
    source_maps_dir,
    targets,
)

NAMESPACE = "map_rebuild"

# The only topics taken from the recording. Never the velocities or the stop
# reset it also holds: see the docstring.
PLAYED_TOPICS = ["/scan", "/tf_static"]

# Every absolute name the rebuild touches, moved under its namespace. Written
# out in full, so test_cmd_vel_topology can read what the nodes are wired to.
ISOLATED = [
    ("/scan", "/map_rebuild/scan"),
    ("/tf", "/map_rebuild/tf"),
    ("/tf_static", "/map_rebuild/tf_static"),
    ("/clock", "/map_rebuild/clock"),
]


def _rebuild(context, *args, **kwargs) -> list:
    """Check everything before a long replay, not at the end of it."""
    recording = Path(LaunchConfiguration("recording").perform(context)).expanduser()
    room = check_room_name(LaunchConfiguration("room").perform(context))
    force = LaunchConfiguration("force").perform(context).lower() == "true"
    check_recording(recording, PLAYED_TOPICS)
    maps = source_maps_dir(Path(get_package_prefix("robot_bringup")))
    check_free(targets(room, maps, RUNS_DIR), force)

    play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(recording),
            "--topics", *PLAYED_TOPICS,
            "--clock",
            # Give Cartographer time to subscribe before the first scan.
            "--delay", "3",
            "--rate", LaunchConfiguration("rate").perform(context),
            "--disable-keyboard-controls",
            "--remap",
            *[f"{name}:={moved}" for name, moved in ISOLATED],
        ],
        output="screen",
    )
    save = ExecuteProcess(
        cmd=[
            "ros2", "run", "robot_bringup", "save_map", room,
            *(["--force"] if force else []),
            "--ros-args", "-r", f"__ns:=/{NAMESPACE}",
        ],
        output="screen",
    )
    return [
        play,
        RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[save])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=save,
                on_exit=[EmitEvent(event=Shutdown(reason="map rebuilt"))],
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "recording",
            description=(
                "The recording's folder, e.g. "
                "~/robot_runs/car-mapping-<date>-<time>."
            ),
        ),
        DeclareLaunchArgument(
            "room", description="Name to save the map under, e.g. living_room."
        ),
        DeclareLaunchArgument(
            "force",
            default_value="false",
            description="true replaces a map (and .pbstream) of the same name.",
        ),
        DeclareLaunchArgument(
            "rate", default_value="1.0", description="Playback speed; 1.0 is as driven."
        ),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="car_cartographer.lua",
            description=(
                "Cartographer settings in robot_bringup/config; match the live run."
            ),
        ),
        # First, so a wrong name or recording stops the launch before
        # anything starts.
        OpaqueFunction(function=_rebuild),

        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            namespace=NAMESPACE,
            output="screen",
            parameters=[{"use_sim_time": True}],
            remappings=ISOLATED,
            arguments=[
                "-configuration_directory", str(bringup_share / "config"),
                "-configuration_basename", LaunchConfiguration("configuration_basename"),
            ],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            namespace=NAMESPACE,
            output="screen",
            parameters=[{"use_sim_time": True, "resolution": 0.05}],
            remappings=ISOLATED,
        ),
    ])
