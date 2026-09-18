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

LIFECYCLE: slam_toolbox (2.8.x, the Jazzy version) is a lifecycle node. Run as
a plain Node it starts, prints its stack size, and then sits in `unconfigured`
forever - no /map, no scan subscription, no error. It has to be configured and
activated. This launch does that itself, with launch events, rather than
handing it to the Nav2 lifecycle manager: `use_lifecycle_manager` stays false,
so slam_toolbox does not join the manager's bond group. That keeps mapping
independent of the navigation lifecycle, and it keeps the safety gate out of
lifecycle management entirely - see the note in navigation.launch.py.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("robot_bringup"))

    autostart = LaunchConfiguration("autostart")

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_sim_time": True,
                # Mapping is not part of the navigation lifecycle group.
                "use_lifecycle_manager": False,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(bringup_share / "config" / "slam_toolbox.yaml"),
            description="SLAM Toolbox parameter file.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description=(
                "Configure and activate slam_toolbox on start. False leaves it "
                "in `unconfigured` for manual lifecycle transitions."
            ),
        ),

        slam_toolbox,

        # unconfigured -> inactive
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(autostart),
        ),

        # inactive -> active, once configuring has actually finished.
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    LogInfo(msg="slam_toolbox configured; activating."),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    ),
                ],
            ),
            condition=IfCondition(autostart),
        ),
    ])
