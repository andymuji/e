"""Named places: save the robot's current pose, recall it as a Nav2 goal.

Run this alongside navigation.launch.py, which is what turns a goal into
motion and what runs the safety gate:

    ros2 launch robot_bringup simulation.launch.py
    ros2 launch robot_bringup navigation.launch.py map:=/path/to/test_room.yaml
    ros2 launch robot_bringup locations.launch.py

Name the place the robot is standing in, then send it back there later:

    ros2 topic pub --once /save_location std_msgs/String "{data: kitchen}"
    ros2 topic pub --once /recall_location std_msgs/String "{data: kitchen}"
    ros2 service call /location_manager/list_locations std_srvs/srv/Trigger

SAFETY: nothing here can drive the robot. The only node started is
robot_locations, which publishes a /goal_pose and nothing else; Nav2 decides
the path and the safety gate decides whether the wheels turn. Starting a
planner or a teleop here would make this launch file a second motion source,
so LocationsLaunchTests asserts robot_locations is the only package in it.

The store file defaults to a per-machine runtime path outside the workspace
rather than web_ui/locations.json: see the note in location_node.py on why
the web console and this node do not share one file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "locations_file",
            default_value="",
            description=(
                "JSON file of saved places. Empty uses the node's default "
                "runtime path under XDG_STATE_HOME."
            ),
        ),

        Node(
            package="robot_locations",
            executable="location_node",
            name="location_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "locations_file": LaunchConfiguration("locations_file"),
            }],
        ),
    ])
