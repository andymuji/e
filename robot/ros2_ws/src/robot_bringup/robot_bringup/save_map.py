"""Save the map Cartographer has built, into the repository, in one command.

    ros2 run robot_bringup save_map living_room
    ros2 run robot_bringup save_map living_room --force

Run it while car_mapping.launch.py (or map_from_recording.launch.py, which
runs it for you) is still going. It:

1. finishes Cartographer's mapping, so the map stops changing;
2. saves the map as living_room.pgm (the picture) and living_room.yaml (its
   scale and position) in robot/ros2_ws/src/robot_bringup/maps/ of the source
   checkout this workspace was built from, wherever the terminal happens to
   be - that folder is what gets committed;
3. saves Cartographer's own full state as ~/robot_runs/living_room.pbstream.

The .pbstream is everything Cartographer knew: every scan it kept, not just
the picture. It is what would let the map be extended or refined later,
without driving again. It is NOT committed: it is binary, several megabytes
for one room and larger the longer the drive, and every re-save would stay in
the repository's history for good. Nothing reads it yet, and it can be
rebuilt from the recording - which is why it sits in ~/robot_runs, where the
recordings are.

It refuses to replace a map that is already there unless given --force, and
always refuses the name test_room: that map is generated from the simulated
world, and tests check it byte for byte.

Finishing the mapping is final for that run: to map more, start the launch
again. Nothing here moves anything; it only calls Cartographer's services and
nav2_map_server's map_saver_cli.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
import sys
import time

# Generated from worlds/test_room.sdf by world_to_map, and compared with that
# world byte for byte by MapGeneratorTests. A driven map saved under this name
# would fail those tests, and navigation.launch.py would silently switch to it.
RESERVED_NAMES = {"test_room"}

# A plain file name that survives git, a shell and map_server's YAML, with no
# path in it: "../x" or "/tmp/x" would put the map somewhere nobody commits.
ROOM_NAME = re.compile(r"[a-z0-9][a-z0-9_-]*")

# Where the recorder keeps its runs (record.launch.py's output_dir default).
RUNS_DIR = Path("~/robot_runs")

# Cartographer's occupancy grid is republished about once a second; wait long
# enough after finishing the trajectory for the saved picture to include it.
SETTLE_SECONDS = 3.0
SERVICE_TIMEOUT = 10.0


class Refused(Exception):
    """A reason not to save, worded for the person at the terminal."""


@dataclass(frozen=True)
class Targets:
    """The three files one save writes."""

    pgm: Path
    yaml: Path
    pbstream: Path

    @property
    def map_stem(self) -> Path:
        return self.pgm.with_suffix("")

    def all(self) -> list[Path]:
        return [self.pgm, self.yaml, self.pbstream]


def check_room_name(room: str) -> str:
    if room in RESERVED_NAMES:
        raise Refused(
            f"'{room}' is the map generated from the simulated world, and "
            "tests check it byte for byte. Choose the real room's name, for "
            "example living_room."
        )
    if not ROOM_NAME.fullmatch(room):
        raise Refused(
            f"'{room}' is not a usable map name. Use lower-case letters, "
            "digits, _ and -, starting with a letter or digit, for example "
            "living_room."
        )
    return room


def source_maps_dir(package_prefix: Path) -> Path:
    """The maps/ folder in the source checkout this install was built from.

    colcon puts a package at <workspace>/install/<package>, and the sources at
    <workspace>/src/<package>. Following that, rather than the installed
    share/ folder, is what makes the saved map land where git sees it,
    however the workspace was built.
    """
    workspace = package_prefix.resolve().parents[1]
    package = workspace / "src" / "robot_bringup"
    if package_prefix.resolve().parent.name != "install" or not (
        package / "package.xml"
    ).is_file():
        raise Refused(
            f"cannot find the source folder robot_bringup was built from "
            f"(looked for {package}). Build with colcon from robot/ros2_ws "
            "and source robot/ros2_ws/install/setup.bash."
        )
    return package / "maps"


def targets(room: str, maps_dir: Path, runs_dir: Path) -> Targets:
    return Targets(
        pgm=maps_dir / f"{room}.pgm",
        yaml=maps_dir / f"{room}.yaml",
        pbstream=runs_dir.expanduser() / f"{room}.pbstream",
    )


def check_free(chosen: Targets, force: bool) -> None:
    """Refuse before anything is finished or written, not half-way through."""
    existing = [path for path in chosen.all() if path.exists()]
    if existing and not force:
        listed = "\n  ".join(str(path) for path in existing)
        raise Refused(
            f"these already exist:\n  {listed}\n"
            "Choose another name, or add --force to replace them."
        )


def recorded_topics(recording: Path) -> set[str]:
    """The topics a finished recording holds, from its metadata.yaml."""
    import yaml

    metadata = recording / "metadata.yaml"
    if not metadata.is_file():
        raise Refused(
            f"{recording} is not a finished recording: it has no metadata.yaml. "
            "If the run was stopped other than with Ctrl-C, "
            f"`ros2 bag reindex {recording}` can usually repair it."
        )
    information = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
    return {
        entry["topic_metadata"]["name"]
        for entry in information.get("topics_with_message_count", [])
        if entry.get("message_count", 0) > 0
    }


def check_recording(recording: Path, needed: list[str]) -> None:
    """Refuse a recording that cannot make a map, before replaying all of it."""
    missing = [topic for topic in needed if topic not in recorded_topics(recording)]
    if missing:
        raise Refused(
            f"{recording} has no {' or '.join(missing)}, so no map can be "
            "built from it. Was the lidar running while it recorded?"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="save_map",
        description=(
            "Finish Cartographer's map and save it into the repository's "
            "robot_bringup/maps/, with Cartographer's full state in "
            "~/robot_runs/."
        ),
    )
    parser.add_argument("room", help="name of the room, e.g. living_room")
    parser.add_argument(
        "--force",
        action="store_true",
        help="replace a map (and .pbstream) of the same name",
    )
    return parser


def _call(node, client, request, what: str):
    import rclpy

    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
        raise Refused(
            f"Cartographer did not answer ({client.srv_name}). Is "
            "car_mapping.launch.py still running, on this ROS_DOMAIN_ID?"
        )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT)
    if future.result() is None:
        raise Refused(f"Cartographer did not finish {what} in time.")
    return future.result()


def save(chosen: Targets) -> None:
    """Finish every active trajectory, then write the map and the state."""
    from cartographer_ros_msgs.msg import StatusCode, TrajectoryStates
    from cartographer_ros_msgs.srv import (
        FinishTrajectory,
        GetTrajectoryStates,
        WriteState,
    )
    import rclpy

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("save_map")
    try:
        # Relative names, so `--ros-args -r __ns:=/x` reaches a Cartographer
        # started in a namespace, as map_from_recording.launch.py does.
        states = _call(
            node,
            node.create_client(GetTrajectoryStates, "get_trajectory_states"),
            GetTrajectoryStates.Request(),
            "listing its trajectories",
        ).trajectory_states
        finish = node.create_client(FinishTrajectory, "finish_trajectory")
        for trajectory_id, state in zip(
            states.trajectory_id, states.trajectory_state, strict=True
        ):
            if state != TrajectoryStates.ACTIVE:
                continue
            status = _call(
                node,
                finish,
                FinishTrajectory.Request(trajectory_id=trajectory_id),
                "the map",
            ).status
            if status.code != StatusCode.OK:
                raise Refused(f"Cartographer would not finish: {status.message}")
            # Flushed, or it prints after map_saver_cli's own output.
            print(f"Finished mapping (trajectory {trajectory_id}).", flush=True)
        time.sleep(SETTLE_SECONDS)

        namespace = node.get_namespace().rstrip("/")
        chosen.pgm.parent.mkdir(parents=True, exist_ok=True)
        saver = subprocess.run(
            [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", str(chosen.map_stem),
                "-t", f"{namespace}/map",
                "--ros-args", "-p", "save_map_timeout:=10.0",
            ],
            check=False,
        )
        if saver.returncode != 0 or not (
            chosen.pgm.is_file() and chosen.yaml.is_file()
        ):
            raise Refused(
                "map_saver_cli could not save the map: is "
                f"{namespace}/map being published?"
            )

        chosen.pbstream.parent.mkdir(parents=True, exist_ok=True)
        status = _call(
            node,
            node.create_client(WriteState, "write_state"),
            WriteState.Request(
                filename=str(chosen.pbstream), include_unfinished_submaps=True
            ),
            "writing its state",
        ).status
        if status.code != StatusCode.OK:
            raise Refused(f"Cartographer could not write its state: {status.message}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv: list[str] | None = None) -> int:
    from rclpy.utilities import remove_ros_args

    arguments = build_parser().parse_args(
        remove_ros_args(sys.argv if argv is None else argv)[1:]
    )
    try:
        from ament_index_python.packages import get_package_prefix

        room = check_room_name(arguments.room)
        chosen = targets(
            room, source_maps_dir(Path(get_package_prefix("robot_bringup"))), RUNS_DIR
        )
        check_free(chosen, arguments.force)
        save(chosen)
    except Refused as reason:
        print(f"NOT SAVED: {reason}", file=sys.stderr)
        return 1

    print(
        "\nSAVED. Commit these two with git:\n"
        f"  {chosen.pgm}\n  {chosen.yaml}\n"
        "Cartographer's full state, kept beside the recordings (not for git):\n"
        f"  {chosen.pbstream}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
