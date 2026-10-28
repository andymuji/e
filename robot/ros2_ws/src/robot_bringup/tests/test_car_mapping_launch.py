"""Checks on car_mapping.launch.py: the Pi on the RC car only listens.

The car is driven by its own radio remote, outside robot_safety, so nothing
this launch starts may name a velocity or the stop reset, and the Foxglove
viewer it brings along must be the watch-only one. It runs on the real
clock; a node left on simulation time would wait forever for a simulator.
"""

import ast
from pathlib import Path
import unittest

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "car_mapping.launch.py"

FORBIDDEN_TOPICS = {"cmd_vel", "cmd_vel_requested", "cmd_vel_raw", "emergency_stop_reset"}

# Everything this launch may start itself: the lidar's position, the lidar,
# and the map builder. Foxglove and the recorder come in by include.
ALLOWED_PACKAGES = {"tf2_ros", "ldlidar_ros2", "cartographer_ros"}


class CarMappingLaunchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tree = ast.parse(LAUNCH.read_text())

    def calls(self, name: str) -> list[ast.Call]:
        return [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == name
        ]

    def keywords(self, call: ast.Call) -> dict[str, ast.expr]:
        return {keyword.arg: keyword.value for keyword in call.keywords}

    def test_no_velocity_or_stop_reset_is_named(self) -> None:
        strings = {
            node.value.lstrip("/")
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertFalse(FORBIDDEN_TOPICS & strings)

    def test_starts_only_the_lidar_and_the_map_builder(self) -> None:
        nodes = self.calls("Node")
        self.assertEqual(len(nodes), 4)
        for node in nodes:
            keywords = self.keywords(node)
            self.assertIn(ast.literal_eval(keywords["package"]), ALLOWED_PACKAGES)
            self.assertNotIn("remappings", keywords)

    def test_every_node_is_on_the_real_clock(self) -> None:
        for node in self.calls("Node"):
            (params,) = self.keywords(node)["parameters"].elts
            values = {
                ast.literal_eval(key): value
                for key, value in zip(params.keys, params.values, strict=True)
            }
            self.assertIs(ast.literal_eval(values["use_sim_time"]), False)

    def test_foxglove_is_the_watch_only_viewer_on_the_real_clock(self) -> None:
        # Starting foxglove_bridge here would bypass viewer.launch.py's
        # capabilities, which test_viewer_launch.py holds to watch-only.
        includes = {
            node.value: include
            for include in self.calls("IncludeLaunchDescription")
            for node in ast.walk(include.args[0])
            if isinstance(node, ast.Constant) and str(node.value).endswith(".launch.py")
        }
        self.assertEqual(set(includes), {"viewer.launch.py", "record.launch.py"})
        arguments = self.keywords(includes["viewer.launch.py"])["launch_arguments"]
        self.assertEqual(ast.literal_eval(arguments.func.value), {"use_sim_time": "false"})

    def test_the_lidar_position_has_one_publisher(self) -> None:
        # LDRobot's ld14p.launch.py publishes its own base_link -> base_laser,
        # which is why the includes above are an exact set: including it
        # would give the lidar two positions.
        publishers = [
            node
            for node in self.calls("Node")
            if ast.literal_eval(self.keywords(node)["executable"]) == "static_transform_publisher"
        ]
        self.assertEqual(len(publishers), 1)


if __name__ == "__main__":
    unittest.main()
