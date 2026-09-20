"""Checks on console.launch.py: it may ask for motion, never command it.

The launch file cannot be executed without ROS, so these read its syntax tree
instead. What matters is what it cannot do - publish a velocity, remap
anything onto the wheels, or start the robot with no approved destinations in
front of it.
"""

import ast
from pathlib import Path
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "console.launch.py"
SETUP = PACKAGE / "setup.py"


def code_strings(source: str) -> set[str]:
    """Every string literal except the docstrings.

    The docstring spells out the motion path, so a plain text search for a
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


class ConsoleLaunchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = LAUNCH.read_text()
        self.tree = ast.parse(self.source)
        self.strings = code_strings(self.source)

    def test_it_is_valid_python(self) -> None:
        compile(self.source, str(LAUNCH), "exec")

    def test_it_starts_only_the_console(self) -> None:
        packages = {
            keyword.value.value
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "Node"
            for keyword in node.keywords
            if keyword.arg == "package" and isinstance(keyword.value, ast.Constant)
        }

        self.assertEqual(packages, {"robot_console"})

    def test_it_names_no_velocity_topic(self) -> None:
        # Not even as a remap: the console is a motion source, and only
        # robot_safety publishes /cmd_vel.
        for name in ("cmd_vel", "/cmd_vel", "cmd_vel_requested"):
            with self.subTest(name=name):
                self.assertNotIn(name, self.strings)

    def test_the_destination_store_is_required(self) -> None:
        # Declared with no default, so the console cannot start offering a
        # list of destinations nobody approved.
        declarations = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and getattr(node.func, "id", None) == "DeclareLaunchArgument"
        ]
        locations = [
            node
            for node in declarations
            if node.args and getattr(node.args[0], "value", None) == "locations"
        ]

        self.assertEqual(len(locations), 1)
        defaults = [
            keyword for keyword in locations[0].keywords if keyword.arg == "default_value"
        ]
        self.assertEqual(defaults, [])

    def test_it_listens_locally_unless_told_otherwise(self) -> None:
        # The page can engage and release the software stop, so reaching it
        # from the network is a decision someone has to make on purpose.
        self.assertIn("127.0.0.1", self.strings)

    def test_setup_py_installs_the_launch_file(self) -> None:
        # A launch file that is not installed is one `ros2 launch` cannot find.
        self.assertIn("launch/*.launch.py", code_strings(SETUP.read_text()))


if __name__ == "__main__":
    unittest.main()
