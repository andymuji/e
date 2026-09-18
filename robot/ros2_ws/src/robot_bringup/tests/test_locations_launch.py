"""Checks on the named-places launch file, without running ROS.

locations.launch.py exists to start one node that turns a remembered name
into a Nav2 goal. The thing a typo could break quietly here is the topology:
a second node in this file, or a remap onto the velocity topics, would make
it a motion source that nothing in the safety review is looking at.
"""

import ast
from pathlib import Path
import unittest

LAUNCH = Path(__file__).resolve().parents[1] / "launch"


def launched_packages(source: str) -> set[str]:
    """The `package=` of every launch_ros Node in a launch file.

    Read from the syntax tree rather than by searching the text, so a package
    named in a comment or a docstring is not mistaken for a running node.
    """
    packages = set()
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) != "Node":
            continue
        for keyword in node.keywords:
            if keyword.arg == "package" and isinstance(keyword.value, ast.Constant):
                packages.add(keyword.value.value)
    return packages


def launched_parameters(source: str) -> str:
    """The text of every `parameters=` argument in a launch file."""
    chunks = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        for keyword in node.keywords:
            if keyword.arg == "parameters":
                chunks.append(ast.unparse(keyword.value))
    return "\n".join(chunks)


class LocationsLaunchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.locations = (LAUNCH / "locations.launch.py").read_text()

    def test_locations_launches_nothing_that_can_drive(self) -> None:
        # Recalling a place is a request for motion, not motion: this file
        # starts the node that asks, and nothing that could answer. A planner
        # or a teleop here would be a motion source outside navigation.launch.py,
        # where the remaps onto the request topic are reviewed.
        packages = launched_packages(self.locations)

        self.assertEqual(packages, {"robot_locations"})
        self.assertNotIn("cmd_vel", self.locations)

    def test_nothing_is_remapped_onto_the_velocity_topics(self) -> None:
        # Belt and braces around the assertion above: a remap is the other
        # way a launch file quietly acquires a path to the wheels.
        self.assertNotIn("remappings", self.locations)

    def test_the_node_is_started_under_the_name_it_declares(self) -> None:
        # A mismatched name means a parameter file or service call aimed at
        # this node lands nowhere and the node runs on its defaults.
        source = (
            LAUNCH.parent.parent / "robot_locations" / "robot_locations"
            / "location_node.py"
        ).read_text()

        self.assertIn('super().__init__("location_manager")', source)
        self.assertIn('name="location_manager"', self.locations)

    def test_the_entry_point_this_file_runs_exists(self) -> None:
        setup = (
            LAUNCH.parent.parent / "robot_locations" / "setup.py"
        ).read_text()

        self.assertIn('executable="location_node"', self.locations)
        self.assertIn("location_node = robot_locations.location_node:main", setup)

    def test_the_simulator_clock_is_used(self) -> None:
        # The node ages its pose reading against the node clock, so a sim run
        # with use_sim_time off would judge fresh poses stale.
        self.assertIn('"use_sim_time": True', self.locations)

    def test_the_store_is_not_pointed_at_the_committed_web_ui_fixture(self) -> None:
        # web_ui/locations.json is a demo fixture that the web console
        # rewrites wholesale from its own cached copy. Pointing this node at
        # it would give the file two writers, and the second to write wins.
        self.assertNotIn("web_ui", launched_parameters(self.locations))


if __name__ == "__main__":
    unittest.main()
