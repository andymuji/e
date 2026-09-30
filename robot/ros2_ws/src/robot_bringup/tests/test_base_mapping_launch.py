"""Checks on base_mapping.launch.py: the gate, the driver and the map builder.

Three things here would be wrong silently on the robot:

- base_link with two parents. Cartographer and the driver can both publish
  odom -> base_link; if both do, TF flips between two answers and the map
  smears. Which one does is decided in two different files, so the rule is
  checked across both.
- The gate on a simulated clock. safety.yaml is written for the simulator;
  on the Pi no /clock exists, time never leaves zero, and every timeout
  the gate has would stop meaning anything.
- The driver going faster than the gate's stop distance assumes.
"""

import ast
from pathlib import Path
import re
import unittest

import yaml

SHARE = Path(__file__).resolve().parents[1]
CONFIG = SHARE / "config"
LAUNCH = SHARE / "launch" / "base_mapping.launch.py"
DRIVER_SOURCE = SHARE.parent / "robot_base" / "robot_base" / "base_node.py"


def cartographer_options() -> dict[str, object]:
    """The top-level `options` of car_cartographer.lua, as Python values."""
    text = (CONFIG / "car_cartographer.lua").read_text()
    block = re.search(r"options\s*=\s*\{(.*?)\n\}", text, re.S)
    if block is None:
        raise AssertionError("car_cartographer.lua has no options block")
    options: dict[str, object] = {}
    for key, value in re.findall(r"^\s*(\w+)\s*=\s*([^,\n]+),", block.group(1), re.M):
        value = value.strip()
        if value in ("true", "false"):
            options[key] = value == "true"
        elif value.startswith('"'):
            options[key] = value.strip('"')
    return options


def declared_defaults(source: str) -> dict[str, object]:
    """Every declare_parameter("name", <literal>) in a node's source."""
    found = {}
    for node in ast.walk(ast.parse(source)):
        if (
            isinstance(node, ast.Call)
            and getattr(node.func, "attr", None) == "declare_parameter"
            and len(node.args) == 2
        ):
            found[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
    return found


class LaunchNodes:
    """The Node actions in base_mapping.launch.py, keyed by package."""

    def __init__(self) -> None:
        tree = ast.parse(LAUNCH.read_text())
        self.nodes: dict[str, ast.Call] = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "Node":
                keywords = {k.arg: k.value for k in node.keywords}
                self.nodes[ast.literal_eval(keywords["package"])] = node

    def keyword(self, package: str, name: str) -> ast.expr:
        for keyword in self.nodes[package].keywords:
            if keyword.arg == name:
                return keyword.value
        raise KeyError(name)

    def parameter_overrides(self, package: str) -> dict[str, object]:
        """The literal dict entries in a node's parameters list, in order."""
        overrides: dict[str, object] = {}
        for element in self.keyword(package, "parameters").elts:
            if isinstance(element, ast.Dict):
                for key, value in zip(element.keys, element.values, strict=True):
                    try:
                        overrides[ast.literal_eval(key)] = ast.literal_eval(value)
                    except ValueError:
                        overrides[ast.literal_eval(key)] = value  # a substitution
        return overrides

    def parameter_files(self, package: str) -> list[str]:
        files = []
        for element in self.keyword(package, "parameters").elts:
            for part in ast.walk(element):
                if isinstance(part, ast.Constant) and str(part.value).endswith(".yaml"):
                    files.append(part.value)
        return files


class TransformTreeTests(unittest.TestCase):
    """base_link has exactly one parent with the defaults as shipped."""

    def setUp(self) -> None:
        self.options = cartographer_options()
        self.base = yaml.safe_load((CONFIG / "base.yaml").read_text())["base_driver"][
            "ros__parameters"
        ]

    def parents_of(self, frame: str) -> list[str]:
        """Who publishes a transform whose child is `frame`, on the car."""
        edges = []
        published = self.options["published_frame"]
        if self.options["provide_odom_frame"]:
            edges += [
                ("cartographer", self.options["map_frame"], self.options["odom_frame"]),
                ("cartographer", self.options["odom_frame"], published),
            ]
        else:
            edges.append(("cartographer", self.options["map_frame"], published))
        if self.base["publish_tf"]:
            edges.append(("base_driver", self.base["odom_frame"], self.base["base_frame"]))
        # car_mapping.launch.py's static transform is base_link -> lidar_link.
        edges.append(("static", "base_link", "lidar_link"))
        return [who for who, _, child in edges if child == frame]

    def test_the_cartographer_options_were_read(self) -> None:
        for key in ("provide_odom_frame", "published_frame", "odom_frame", "map_frame"):
            self.assertIn(key, self.options)

    def test_base_link_has_exactly_one_parent(self) -> None:
        self.assertEqual(len(self.parents_of("base_link")), 1, self.parents_of("base_link"))

    def test_the_driver_does_not_publish_tf_while_cartographer_provides_odom(
        self,
    ) -> None:
        # The reason for the default, stated directly: Cartographer on the
        # car publishes odom -> base_link itself.
        self.assertTrue(self.options["provide_odom_frame"])
        self.assertEqual(self.options["published_frame"], "base_link")
        self.assertFalse(self.base["publish_tf"])

    def test_the_drivers_own_default_is_off_too(self) -> None:
        # A launch that forgets base.yaml must not bring a second parent.
        self.assertIs(declared_defaults(DRIVER_SOURCE.read_text())["publish_tf"], False)

    def test_the_drivers_frames_match_cartographers(self) -> None:
        self.assertEqual(self.base["base_frame"], self.options["published_frame"])
        self.assertEqual(self.base["odom_frame"], self.options["odom_frame"])


class GateClockTests(unittest.TestCase):
    def setUp(self) -> None:
        self.nodes = LaunchNodes()

    def test_the_gate_is_started_with_the_teleop_settings(self) -> None:
        self.assertEqual(
            [Path(name).name for name in self.nodes.parameter_files("robot_safety")],
            ["safety.yaml"],
        )

    def test_the_gate_runs_on_the_real_clock(self) -> None:
        # safety.yaml says true, for the simulator, which is why the override
        # has to be here and has to come after the file.
        safety = yaml.safe_load((CONFIG / "safety.yaml").read_text())
        self.assertTrue(safety["safety_controller"]["ros__parameters"]["use_sim_time"])
        parameters = self.nodes.keyword("robot_safety", "parameters").elts
        self.assertIsInstance(parameters[-1], ast.Dict)

        self.assertIs(self.nodes.parameter_overrides("robot_safety")["use_sim_time"], False)

    def test_the_gate_keeps_its_node_name(self) -> None:
        # The driver trusts /cmd_vel only from a node of this name.
        name = ast.literal_eval(self.nodes.keyword("robot_safety", "name"))
        base = yaml.safe_load((CONFIG / "base.yaml").read_text())

        self.assertEqual(name, base["base_driver"]["ros__parameters"]["gate_node"])

    def test_the_driver_is_on_the_real_clock(self) -> None:
        base = yaml.safe_load((CONFIG / "base.yaml").read_text())

        self.assertIs(base["base_driver"]["ros__parameters"]["use_sim_time"], False)
        self.assertEqual(
            [Path(name).name for name in self.nodes.parameter_files("robot_base")],
            ["base.yaml"],
        )


class DriverParameterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base = yaml.safe_load((CONFIG / "base.yaml").read_text())["base_driver"][
            "ros__parameters"
        ]
        self.gate = yaml.safe_load((CONFIG / "safety.yaml").read_text())[
            "safety_controller"
        ]["ros__parameters"]
        self.dynamics = yaml.safe_load((CONFIG / "base_dynamics.yaml").read_text())

    def test_every_parameter_in_the_file_is_one_the_driver_declares(self) -> None:
        # A misspelt key is silently ignored and the node runs on its default.
        declared = declared_defaults(DRIVER_SOURCE.read_text())

        self.assertEqual(set(self.base) - {"use_sim_time"} - set(declared), set())
        self.assertIn('super().__init__("base_driver")', DRIVER_SOURCE.read_text())

    def test_the_driver_is_no_faster_than_the_stop_distance_assumes(self) -> None:
        # stop_distance is derived from base_dynamics.yaml's max_speed.
        self.assertLessEqual(self.base["max_linear_speed"], self.dynamics["max_speed"])
        self.assertLessEqual(
            declared_defaults(DRIVER_SOURCE.read_text())["max_linear_speed"],
            self.dynamics["max_speed"],
        )

    def test_a_dead_gate_stops_the_driver_no_later_than_the_gate_would(self) -> None:
        self.assertLessEqual(self.base["command_timeout"], self.gate["command_timeout"])

    def test_the_proof_of_concept_drives_like_a_tank(self) -> None:
        self.assertIs(self.base["allow_sideways"], False)


if __name__ == "__main__":
    unittest.main()
