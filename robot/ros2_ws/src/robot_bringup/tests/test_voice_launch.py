"""Checks on voice.launch.py: it may ask for motion, never command it.

Voice is the newest motion source in bringup, so what matters here is what it
cannot do - publish a velocity, remap anything onto the wheels, or start the
robot with nothing in front of it. The launch file cannot be executed without
ROS, so these read its syntax tree instead.
"""

import ast
from pathlib import Path
import unittest

SHARE = Path(__file__).resolve().parents[1]
LAUNCH = SHARE / "launch"
VOICE_NODE = SHARE.parent / "robot_voice" / "robot_voice" / "voice_node.py"


def launch_nodes(source: str) -> list[ast.Call]:
    """Every launch_ros Node call in a launch file.

    Read from the syntax tree rather than by searching the text, so a package
    or a topic named in a comment or a docstring is not mistaken for a
    running node.
    """
    return [
        node
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "Node"
    ]


def launched_packages(source: str) -> set[str]:
    """The `package=` of every launch_ros Node in a launch file."""
    packages = set()
    for call in launch_nodes(source):
        for keyword in call.keywords:
            if keyword.arg == "package" and isinstance(keyword.value, ast.Constant):
                packages.add(keyword.value.value)
    return packages


def code_strings(source: str) -> set[str]:
    """Every string literal except the docstrings.

    The docstrings spell out the motion path, so a plain text search for a
    topic name would match the explanation rather than the wiring.
    """
    tree = ast.parse(source)

    docstrings = set()
    for node in ast.walk(tree):
        body = getattr(node, "body", None)
        if not isinstance(body, list) or not body:
            continue
        first = body[0]
        if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
            docstrings.add(id(first.value))

    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and id(node) not in docstrings
    }


class VoiceLaunchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.path = LAUNCH / "voice.launch.py"
        self.voice = self.path.read_text()
        self.node_source = VOICE_NODE.read_text()

    def test_launch_file_is_valid_python(self) -> None:
        compile(self.voice, str(self.path), "exec")

    def test_voice_launches_nothing_that_can_drive(self) -> None:
        # A voice command is a request. The gate and Nav2 are started by
        # simulation.launch.py and navigation.launch.py, which is what keeps
        # this file from being able to bring up a robot that only listens.
        self.assertEqual(launched_packages(self.voice), {"robot_voice"})

    def test_nothing_here_names_the_wheels(self) -> None:
        for literal in code_strings(self.voice):
            with self.subTest(literal=literal):
                self.assertNotIn("cmd_vel", literal)

    def test_no_node_is_remapped_at_all(self) -> None:
        # A remap is how a node reaches a topic it was not written for, and
        # /cmd_vel is the one topic nothing but the gate may publish.
        for call in launch_nodes(self.voice):
            keywords = {keyword.arg for keyword in call.keywords}

            self.assertNotIn("remappings", keywords)

    def test_the_voice_node_publishes_no_velocity(self) -> None:
        # Not even the gated request topic: an approved destination becomes a
        # Nav2 goal, and Nav2's own output is already behind the gate.
        for literal in code_strings(self.node_source):
            with self.subTest(literal=literal):
                self.assertNotIn("cmd_vel", literal)

        self.assertIn("NavigateToPose", self.node_source)

    def test_the_voice_node_cannot_release_the_emergency_stop_latch(self) -> None:
        # A heard "stop" engages the latch. Only an operator on
        # emergency_stop_reset releases it, so a voice that could publish
        # that topic could talk its own stop away.
        for literal in code_strings(self.node_source):
            with self.subTest(literal=literal):
                self.assertNotEqual(literal, "emergency_stop_reset")

    def test_the_node_name_matches_the_voice_node(self) -> None:
        # A mismatched name means a parameter file keyed by node name, or a
        # later remap, silently misses this node.
        self.assertIn('name="voice_commander"', self.voice)
        self.assertIn('super().__init__("voice_commander"', self.node_source)

    def test_the_approved_destination_file_has_to_be_given(self) -> None:
        arguments = [
            call
            for call in ast.walk(ast.parse(self.voice))
            if isinstance(call, ast.Call)
            and getattr(call.func, "id", None) == "DeclareLaunchArgument"
        ]
        locations = next(
            call for call in arguments if call.args[0].value == "locations"
        )
        defaults = {keyword.arg for keyword in locations.keywords}

        # Defaulting the allow-list would mean a typo in the path silently
        # produces a robot that refuses every trip, or worse, one pointed at
        # somebody else's approved destinations.
        self.assertNotIn("default_value", defaults)


if __name__ == "__main__":
    unittest.main()
