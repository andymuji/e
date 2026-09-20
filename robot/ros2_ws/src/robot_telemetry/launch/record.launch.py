"""Record a run as evidence, alongside whatever else is already running.

    ros2 launch robot_telemetry record.launch.py
    ros2 launch robot_telemetry record.launch.py label:=estop-test

Start it in its own terminal next to simulation.launch.py or the robot's
bringup, drive the test, then stop it with Ctrl-C and analyse what it caught:

    ros2 run robot_telemetry analyse_run ~/robot_runs/<the new directory>

SAFETY: nothing started here can move the robot. `ros2 bag record` only
subscribes, and the graph probe only reads the topic graph and writes a file.
This package deliberately contains no publisher at all - not a velocity, and
above all not an `emergency_stop_reset`, which is an operator action and must
never be something a recording tool can perform. A witness that can also act
is no longer a witness.

Recording does not replace the physical emergency-stop test in
docs/safety-test-procedure.md. It produces evidence about one run; the
physical test is about whether the robot can be stopped at all.
"""

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# What a safety report needs to be able to say anything. /cmd_vel_requested
# and /cmd_vel are the two ends of the gate; /safety_state is its reasoning;
# /scan is what it was looking at; the two stop topics are the operator. The
# rest is the context a human needs to make sense of the rest.
#
# A topic that is not being published yet is not an error: rosbag2 keeps
# discovering, so /map and /amcl_pose are picked up if and when navigation is
# started, and simply absent from the recording otherwise.
RECORDED_TOPICS = [
    "/cmd_vel_requested",
    "/cmd_vel",
    "/safety_state",
    "/scan",
    "/emergency_stop",
    "/emergency_stop_reset",
    "/tf",
    "/tf_static",
    "/amcl_pose",
    "/battery_state",
    "/map",
]


def _recording(context, *args, **kwargs) -> list:
    """Build the two actions once the launch arguments have real values.

    The bag directory is named here, at launch time, because the probe has to
    be told the same path and a timestamp evaluated twice would not match.
    """
    directory = Path(
        LaunchConfiguration("output_dir").perform(context)
    ).expanduser()
    label = LaunchConfiguration("label").perform(context) or "run"
    bag = directory / f"{label}-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    directory.mkdir(parents=True, exist_ok=True)
    # Resolved to a float here rather than handed over as a substitution: a
    # ROS parameter declared as a double refuses a string override, and the
    # probe would fail to start at the moment recording was wanted.
    poll_period = float(LaunchConfiguration("poll_period").perform(context))

    return [
        ExecuteProcess(
            # -o refuses to overwrite an existing directory, which is what
            # keeps one run's evidence from quietly replacing another's.
            cmd=["ros2", "bag", "record", "-o", str(bag), *RECORDED_TOPICS],
            output="screen",
        ),
        Node(
            package="robot_telemetry",
            executable="graph_probe",
            name="telemetry_graph_probe",
            output="screen",
            parameters=[{
                # Deliberately left on the wall clock, unlike every other
                # node in this project. Which nodes exist is a fact about the
                # computer, not about the simulated world, and a probe running
                # on simulation time would stop sampling whenever the simulator
                # is paused - which is exactly when someone is looking.
                "use_sim_time": False,
                # Written inside the bag directory so the evidence about who
                # was publishing travels with the messages.
                "output_file": str(bag / "publishers.json"),
                "poll_period": poll_period,
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="~/robot_runs",
            description="Where recordings are kept, one directory per run.",
        ),
        DeclareLaunchArgument(
            "label",
            default_value="run",
            description=(
                "Prefix for this run's directory. Name it after the test being "
                "performed, so the evidence can be matched to the procedure "
                "step it came from."
            ),
        ),
        DeclareLaunchArgument(
            "poll_period",
            default_value="1.0",
            description=(
                "Seconds between topic-graph samples. A node that publishes a "
                "velocity for less than this may go unseen, so lower it when "
                "hunting for one."
            ),
        ),
        OpaqueFunction(function=_recording),
    ])
