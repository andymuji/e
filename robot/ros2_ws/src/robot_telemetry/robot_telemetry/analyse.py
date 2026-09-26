"""Command line: turn a recording into a report, and fail if the run misbehaved.

    ros2 run robot_telemetry analyse_run ~/robot_runs/run-20260919-101500

Exit status is the point. A recording attached to a pull request should be
pass/fail evidence rather than a claim, so this exits 0 only when the run broke
none of the checked rules and put every one of them to the test, 1 when it
broke one, 2 when the recording cannot answer the questions at all, and 4 when
it answered some and left others untouched.
"""

import argparse
from pathlib import Path
import sys

from robot_safety.safety_controller import SafetyController

from .report import DEFAULT_GATE_NODE, build_report

EXIT_UNREADABLE = 3


def default_stop_distance() -> float:
    """The gate's own fallback stop distance, never a second copy of it.

    `stop_distance` is derived from the base dynamics and must exist in
    exactly one place, so this asks `robot_safety` rather than restating a
    number. A run recorded against a parameter file that overrides it should
    be analysed with --stop-distance; the report prints which value it used.
    """
    return SafetyController().stop_distance


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="analyse_run",
        description=(
            "Check a recorded run against the robot's safety invariants and "
            "print a report."
        ),
    )
    parser.add_argument("bag", type=Path, help="the rosbag2 directory to analyse")
    parser.add_argument(
        "--stop-distance",
        type=float,
        default=None,
        metavar="METRES",
        help=(
            "the stop distance the gate was running with; defaults to the "
            "gate's own default, so pass the value from the safety parameter "
            "file if the run used a different one"
        ),
    )
    parser.add_argument(
        "--gate-node",
        default=DEFAULT_GATE_NODE,
        metavar="NAME",
        help="the node allowed to publish /cmd_vel (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        metavar="FILE",
        help="also write the report here, for attaching to a pull request",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)

    # Imported here so that --help works, and so that the failure to find
    # rosbag2 is reported as a sentence rather than as a traceback on import.
    try:
        from .bag import read_publishers, read_records
    except ImportError as error:
        print(
            f"Cannot read recordings: {error}.\n"
            "Source a ROS 2 installation first: . /opt/ros/jazzy/setup.bash",
            file=sys.stderr,
        )
        return EXIT_UNREADABLE

    # A missing directory, a half-written bag, and an unknown message type are
    # all the same answer to the reader: there is no evidence here yet. None of
    # them is a safety verdict, which is why they get their own exit code.
    try:
        records = read_records(arguments.bag)
    except Exception as error:
        print(f"Cannot read {arguments.bag}: {error}", file=sys.stderr)
        if arguments.bag.is_dir() and not (arguments.bag / "metadata.yaml").exists():
            # The common case by far: the recorder was stopped from outside
            # its own terminal - by a script, or a window that was closed -
            # and never wrote its index. The messages are all still there.
            print(
                "The recording has no metadata.yaml, so it was never finished. "
                f"Try: ros2 bag reindex {arguments.bag}",
                file=sys.stderr,
            )
        return EXIT_UNREADABLE

    stop_distance = (
        default_stop_distance()
        if arguments.stop_distance is None
        else arguments.stop_distance
    )
    report = build_report(
        records,
        source=str(arguments.bag),
        stop_distance=stop_distance,
        publishers=read_publishers(arguments.bag),
        gate_node=arguments.gate_node,
    )

    rendered = report.render()
    print(rendered)
    if arguments.output is not None:
        arguments.output.write_text(rendered + "\n")

    return report.verdict.exit_code


if __name__ == "__main__":
    sys.exit(main())
