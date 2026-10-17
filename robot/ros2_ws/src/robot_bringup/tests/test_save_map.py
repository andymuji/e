"""Checks on saving a map, and on rebuilding one from a recording.

save_map writes into the source tree, so a wrong path or a wrong name lands
a file somewhere nobody commits - or over the generated test_room map that
other tests check byte for byte. map_from_recording.launch.py replays a
recording that also holds velocities and the stop reset, so what it plays,
and where it plays it, is checked here from the file itself.
"""

import ast
from pathlib import Path
import tempfile
import unittest

from robot_bringup.save_map import (
    Refused,
    check_free,
    check_recording,
    check_room_name,
    recorded_topics,
    source_maps_dir,
    targets,
)

SHARE = Path(__file__).resolve().parents[1]
LAUNCH = SHARE / "launch"
MAPS = SHARE / "maps"
REBUILD = LAUNCH / "map_from_recording.launch.py"
RECORDER = SHARE.parent / "robot_telemetry" / "launch" / "record.launch.py"

FORBIDDEN_TOPICS = {
    "cmd_vel", "cmd_vel_requested", "cmd_vel_raw", "emergency_stop",
    "emergency_stop_reset",
}


def literal(path: Path, name: str):
    """A module-level constant from a launch file, read without running it."""
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in node.targets
        ):
            return node.value
    raise AssertionError(f"{path.name} has no {name}")


def metadata(topics: dict[str, int]) -> str:
    entries = "".join(
        f"    - topic_metadata:\n        name: {name}\n"
        f"      message_count: {count}\n"
        for name, count in topics.items()
    )
    return (
        "rosbag2_bagfile_information:\n"
        f"  topics_with_message_count:\n{entries}"
    )


class RoomNameTests(unittest.TestCase):
    def test_the_generated_map_cannot_be_overwritten_by_name(self) -> None:
        with self.assertRaises(Refused):
            check_room_name("test_room")

    def test_a_name_cannot_carry_a_path_or_spaces(self) -> None:
        for name in ["../lounge", "/tmp/lounge", "maps/lounge", "Living Room", "", "-x"]:
            with self.subTest(name=name), self.assertRaises(Refused):
                check_room_name(name)

    def test_plain_room_names_are_accepted(self) -> None:
        for name in ["living_room", "kitchen-2", "lounge"]:
            self.assertEqual(check_room_name(name), name)


class WhereTheFilesGoTests(unittest.TestCase):
    def test_maps_go_to_the_committed_maps_folder_not_the_install(self) -> None:
        # colcon's layout: <workspace>/install/robot_bringup beside src/.
        workspace = SHARE.parents[1]
        prefix = workspace / "install" / "robot_bringup"
        self.assertEqual(source_maps_dir(prefix), MAPS)

    def test_an_install_away_from_its_sources_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(Refused):
                source_maps_dir(Path(folder) / "install" / "robot_bringup")

    def test_the_pbstream_is_kept_out_of_the_repository(self) -> None:
        chosen = targets("lounge", MAPS, Path("~/robot_runs"))
        self.assertEqual(chosen.pgm, MAPS / "lounge.pgm")
        self.assertEqual(chosen.yaml, MAPS / "lounge.yaml")
        self.assertEqual(chosen.pbstream, Path.home() / "robot_runs" / "lounge.pbstream")
        self.assertNotIn(SHARE, chosen.pbstream.parents)


class OverwriteTests(unittest.TestCase):
    def setUp(self) -> None:
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        root = Path(self.folder.name)
        self.chosen = targets("lounge", root / "maps", root / "runs")

    def test_nothing_there_is_fine(self) -> None:
        check_free(self.chosen, force=False)

    def test_any_one_existing_file_refuses_the_whole_save(self) -> None:
        for existing in self.chosen.all():
            with self.subTest(existing=existing.name):
                existing.parent.mkdir(parents=True, exist_ok=True)
                existing.write_text("old")
                with self.assertRaises(Refused) as refused:
                    check_free(self.chosen, force=False)
                self.assertIn(str(existing), str(refused.exception))
                existing.unlink()

    def test_force_allows_replacing(self) -> None:
        self.chosen.pgm.parent.mkdir(parents=True)
        self.chosen.pgm.write_text("old")
        check_free(self.chosen, force=True)


class RecordingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.recording = Path(self.folder.name)

    def write(self, topics: dict[str, int]) -> None:
        (self.recording / "metadata.yaml").write_text(metadata(topics))

    def test_an_unfinished_recording_is_refused(self) -> None:
        with self.assertRaises(Refused) as refused:
            check_recording(self.recording, ["/scan"])
        self.assertIn("reindex", str(refused.exception))

    def test_topics_with_no_messages_do_not_count(self) -> None:
        self.write({"/scan": 0, "/tf_static": 1})
        self.assertEqual(recorded_topics(self.recording), {"/tf_static"})
        with self.assertRaises(Refused):
            check_recording(self.recording, ["/scan", "/tf_static"])

    def test_a_recording_with_the_lidar_is_accepted(self) -> None:
        self.write({"/scan": 600, "/tf_static": 1, "/cmd_vel": 5})
        check_recording(self.recording, ["/scan", "/tf_static"])


class RebuildLaunchTests(unittest.TestCase):
    """map_from_recording.launch.py, read rather than run."""

    def setUp(self) -> None:
        self.tree = ast.parse(REBUILD.read_text())
        self.played = ast.literal_eval(literal(REBUILD, "PLAYED_TOPICS"))
        self.namespace = ast.literal_eval(literal(REBUILD, "NAMESPACE"))
        self.isolated = dict(ast.literal_eval(literal(REBUILD, "ISOLATED")))

    def calls(self, name: str) -> list[ast.Call]:
        return [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == name
        ]

    def test_only_the_lidar_and_its_position_are_played(self) -> None:
        # The recording also holds the velocities and the stop reset. Played
        # back, they would drive a robot or release its stop from a file.
        self.assertEqual(self.played, ["/scan", "/tf_static"])
        (play,) = [
            call for call in self.calls("ExecuteProcess")
            if "play" in ast.dump(call)
        ]
        command = ast.dump(play)
        self.assertIn("'--topics'", command)
        self.assertIn("id='PLAYED_TOPICS'", command)
        for option in ["--exclude-topics", "--regex", "--exclude-regex", "--services"]:
            self.assertNotIn(repr(option), command)

    def test_no_velocity_or_stop_topic_is_named_in_code(self) -> None:
        # The module docstring explains what is NOT played; everything else
        # is code, and code has no business naming these.
        body = self.tree.body[1:]
        strings = {
            node.value.strip("/").split("/")[-1]
            for statement in body
            for node in ast.walk(statement)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertFalse(FORBIDDEN_TOPICS & strings)

    def test_everything_the_rebuild_touches_is_moved_off_the_live_names(self) -> None:
        # A live robot on the same domain must never see replayed scans, a
        # replayed clock, or a second map.
        for name in [*self.played, "/tf", "/clock"]:
            self.assertIn(name, self.isolated)
        for moved in self.isolated.values():
            self.assertTrue(moved.startswith(f"/{self.namespace}/"), moved)

    def test_the_map_builders_run_in_the_namespace_on_the_recordings_clock(self) -> None:
        nodes = self.calls("Node")
        self.assertEqual(len(nodes), 2)
        for node in nodes:
            keywords = {keyword.arg: keyword.value for keyword in node.keywords}
            self.assertEqual(ast.literal_eval(keywords["package"]), "cartographer_ros")
            self.assertEqual(ast.unparse(keywords["namespace"]), "NAMESPACE")
            self.assertEqual(ast.unparse(keywords["remappings"]), "ISOLATED")
            (params,) = keywords["parameters"].elts
            values = dict(zip(
                (ast.literal_eval(key) for key in params.keys),
                params.values,
                strict=True,
            ))
            self.assertIs(ast.literal_eval(values["use_sim_time"]), True)

    def test_the_save_asks_the_namespaced_cartographer(self) -> None:
        (save,) = [
            call for call in self.calls("ExecuteProcess")
            if "save_map" in ast.dump(call)
        ]
        self.assertIn("__ns:=/{NAMESPACE}", ast.unparse(save))

    def test_by_default_it_rebuilds_with_the_live_runs_settings(self) -> None:
        self.assertIn('"car_cartographer.lua"', REBUILD.read_text())
        self.assertIn('"car_cartographer.lua"', (LAUNCH / "car_mapping.launch.py").read_text())

    def test_the_recorder_keeps_what_the_rebuild_plays(self) -> None:
        recorded = ast.literal_eval(literal(RECORDER, "RECORDED_TOPICS"))
        self.assertLessEqual(set(self.played), set(recorded))


class MapFolderTests(unittest.TestCase):
    """Every map in maps/ is installed and loadable, not only test_room.

    save_map adds maps beside the generated one. setup.py installs whatever
    is in the folder, so a half-saved pair or an image named by a path from
    the Pi would be installed broken.
    """

    def test_every_map_is_a_yaml_and_image_pair_with_a_bare_image_name(self) -> None:
        import yaml

        for description in sorted(MAPS.glob("*.yaml")):
            with self.subTest(map=description.name):
                image = yaml.safe_load(description.read_text())["image"]
                self.assertEqual(image, Path(image).name)
                self.assertTrue((MAPS / image).is_file())

    def test_every_image_has_its_yaml(self) -> None:
        for image in sorted(MAPS.glob("*.pgm")):
            with self.subTest(image=image.name):
                self.assertTrue(image.with_suffix(".yaml").is_file())


if __name__ == "__main__":
    unittest.main()
