"""Map the simulated room and watch it in Foxglove, in one command.

    ros2 launch robot_bringup sim_mapping.launch.py

The simulated twin of car_mapping.launch.py: the test room and robot in
Gazebo with no window (simulation.launch.py, safety gate included), SLAM
Toolbox building the map (slam.launch.py), and the watch-only Foxglove viewer
(viewer.launch.py). Drive from a second terminal; see teleop.launch.py for
the command that works without a screen.

Nothing here commands motion.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    launch_dir = Path(get_package_share_directory("robot_bringup")) / "launch"

    def include(name: str, **arguments: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / name)),
            launch_arguments=arguments.items(),
        )

    return LaunchDescription([
        include("simulation.launch.py", rviz="false", headless="true"),
        include("viewer.launch.py"),
        # SLAM Toolbox configures as soon as it starts, and gives up if the
        # simulator's clock and the robot's frames are not there yet.
        TimerAction(period=20.0, actions=[include("slam.launch.py")]),
    ])
