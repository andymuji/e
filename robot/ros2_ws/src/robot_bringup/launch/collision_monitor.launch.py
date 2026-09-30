"""Nav2's Collision Monitor: a second obstacle layer in front of the gate.

    ros2 launch robot_bringup collision_monitor.launch.py

navigation.launch.py includes this file, so an ordinary navigation run brings
the monitor up with Nav2 remapped onto its input. Launching it directly, as
above, is for working on the monitor by itself.

WHAT HAS BEEN OBSERVED, AND WHAT HAS NOT (2026-09-23, first run ever):

1. Run on its own against ROS_DOMAIN_ID=42, the node starts, accepts every
   parameter name in config/collision_monitor.yaml, creates the Scan source
   and both polygons, reaches the active state and bonds to its lifecycle
   manager. Before this the configuration had never been loaded by the node
   it was written for, and the parameter names were checked only against the
   Nav2 Jazzy documentation.

2. The topology was confirmed live rather than from the launch file:
   /cmd_vel_requested had exactly one publisher (this node), /cmd_vel_raw one
   subscriber (this node), and **/cmd_vel did not exist at all** - the node
   opened no publisher on the wheel topic. That is the single most dangerous
   line in this configuration, and it is now checked against a running node
   and not only against two files.

3. The StopZone it published matched the derived geometry exactly: x from
   -0.20 to 0.60, y +/-0.15, in base_footprint.

NOT yet observed: this node slowing or stopping a real velocity. That needs
something driving, and nothing has navigated yet. The zones are derived from
the URDF placeholders, so what it does to a real obstacle remains a claim.
Its wiring is proven; its behaviour is not.

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
