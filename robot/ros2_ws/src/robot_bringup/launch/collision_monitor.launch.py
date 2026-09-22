"""Nav2's Collision Monitor: a second obstacle layer in front of the gate.

    ros2 launch robot_bringup collision_monitor.launch.py

NOT WIRED IN YET, AND NOT YET RUN. Two things are still true of this file:

1. `nav2_collision_monitor` is not installed in the development container, so
   this launch file has never been executed and the parameter names in
   config/collision_monitor.yaml have never been accepted by the node they
   are written for. `rosdep install` fetches the package; the first run is
   bring-up work, not a regression test.

2. Nothing feeds it yet. Today Nav2's velocity sources are remapped straight
   onto /cmd_vel_requested, so the monitor sits in the graph with an empty
   input. Connecting it is a one-line change in navigation.launch.py - point
   the `gated` remap at /cmd_vel_raw instead of /cmd_vel_requested - which
   belongs to whoever owns that file, and which must be made for ALL of the
   velocity sources at once. A source left pointing at /cmd_vel_requested
   would simply skip this layer, silently.

SAFETY: this node cannot drive the robot. It has no path to /cmd_vel.

    Nav2  ->  /cmd_vel_raw  ->  collision_monitor  ->  /cmd_vel_requested
                                                            |
                                                  robot_safety (the gate)
                                                            |
                                                        /cmd_vel  ->  wheels

The Collision Monitor only ever reduces a velocity somebody else already
asked for; it cannot originate one. Its output is still a request, and the
gate still rules on it afterwards with its own, blunter, omnidirectional
check. Nav2 ships this node configured to publish /cmd_vel directly, because
upstream expects it to be the last thing before the wheels. Here it is not,
and CollisionMonitorTopologyTests fails if the output topic is ever changed
back.

If this node dies or is deactivated, nothing arrives on /cmd_vel_requested at
all, the gate's command_timeout fires, and the robot stops. Losing this layer
stops the robot rather than freeing it.

ON LIFECYCLE MANAGEMENT: unlike the safety gate, the Collision Monitor *is*
managed, and that is deliberate. The rule is that the gate must still be
running when the lifecycle manager shuts things down on failure - and it is,
because it is not in anybody's managed list. This node is different: it is a
constraint, and a constraint that has been shut down leaves the request path
dead rather than open. Its manager is separate from navigation's so that this
file can be run, and stopped, on its own.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The only node this manager is allowed to control. The safety gate is
# deliberately absent and must stay absent: a lifecycle manager deactivates
# its nodes on failure, and the gate has to still be running then.
MANAGED_NODES = ["collision_monitor"]


def generate_launch_description() -> LaunchDescription:
    config = (
        Path(get_package_share_directory("robot_bringup"))
        / "config"
        / "collision_monitor.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(config),
            description=(
                "Collision Monitor parameters. The zones in the default file "
                "are derived from the URDF and base_dynamics.yaml; a "
                "replacement is not checked for drift."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Follow the simulator's clock rather than the wall clock.",
        ),

        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "node_names": MANAGED_NODES,
            }],
        ),
    ])
