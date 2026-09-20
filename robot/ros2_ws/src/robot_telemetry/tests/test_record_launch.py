"""The recorder is a witness, and these tests are what keep it one.

A flight recorder that can also act on the robot is worse than none: the run
it records is no longer the run that would have happened. So this package is
asserted to contain no publisher, no service client, and no action client at
all - checked against the syntax tree, so a future edit that adds one fails
here rather than on the hardware.
"""

import ast
from pathlib import Path
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "record.launch.py"
MODULE = PACKAGE / "robot_telemetry"

# Every rclpy call that would let this package put something on a topic, ask
# a service for something, or send a goal.
FORBIDDEN_CALLS = {
    "create_publisher",
    "create_client",
    "ActionClient",
    "send_goal_async",
}

# What a safety report cannot be written without. /map and /amcl_pose are
# recorded too but are only present once navigation runs, so they are not
# required here.
REQUIRED_TOPICS = {
    "/cmd_vel_requested",
    "/cmd_vel",
    "/safety_state",
    "/scan",
    "/emergency_stop",
    "/emergency_stop_reset",
}


def called_names(source: str) -> set[str]:
    """Every name that is called in a file, read from the syntax tree.

    From the tree rather than by searching the text, so the word "publisher"
    in a docstring - and this package's docstrings are full of it - is not
    mistaken for a publisher.
    """
    names = set()
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if isinstance(function, ast.Attribute):
            names.add(function.attr)
        elif isinstance(function, ast.Name):
            names.add(function.id)
    return names


def launch_node_packages(source: str) -> set[str]:
    """The `package=` of every launch_ros node action in a launch file."""
    packages = set()
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) not in {"Node", "LifecycleNode"}:
            continue
        for keyword in node.keywords:
            if keyword.arg == "package" and isinstance(keyword.value, ast.Constant):
                packages.add(keyword.value.value)
    return packages


class RecorderPublishesNothingTests(unittest.TestCase):
    def test_no_module_in_this_package_can_send_anything(self):
        for path in sorted(MODULE.glob("*.py")):
            with self.subTest(module=path.name):
                offending = called_names(path.read_text()) & FORBIDDEN_CALLS
                self.assertEqual(
                    offending,
                    set(),
                    f"{path.name} can act on the robot: {sorted(offending)}. "
                    "robot_telemetry records and does nothing else.",
                )

    def test_the_launch_file_starts_nothing_that_could_drive(self):
        packages = launch_node_packages(LAUNCH.read_text())
        self.assertEqual(packages, {"robot_telemetry"})

    def test_the_launch_file_never_runs_a_publishing_command(self):
        """`ros2 topic pub` is denied by repository policy, and by this test.

        The recorder would be the easiest place to smuggle one in: it already
        has a reason to shell out to `ros2`.
        """
        source = LAUNCH.read_text()
        for node in ast.walk(ast.parse(source)):
            if not isinstance(node, ast.Call):
                continue
            if getattr(node.func, "id", None) != "ExecuteProcess":
                continue
            for keyword in node.keywords:
                if keyword.arg != "cmd":
                    continue
                literals = [
                    element.value
                    for element in ast.walk(keyword.value)
                    if isinstance(element, ast.Constant)
                    and isinstance(element.value, str)
                ]
                self.assertIn("record", literals)
                self.assertNotIn("pub", literals)
                self.assertNotIn("call", literals)


class RecordedTopicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = LAUNCH.read_text()

    def test_every_topic_a_report_needs_is_recorded(self):
        tree = ast.parse(self.source)
        recorded = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and any(
                getattr(target, "id", None) == "RECORDED_TOPICS"
                for target in node.targets
            ):
                recorded = {
                    element.value
                    for element in ast.walk(node.value)
                    if isinstance(element, ast.Constant)
                }
        self.assertTrue(REQUIRED_TOPICS <= recorded, sorted(REQUIRED_TOPICS - recorded))

    def test_the_context_topics_are_recorded_too(self):
        self.assertTrue(
            {"/tf", "/tf_static", "/amcl_pose", "/battery_state", "/map"}
            <= set(_recorded_topics(self.source))
        )

    def test_the_launch_file_is_valid_python(self):
        ast.parse(self.source)


def _recorded_topics(source: str) -> list[str]:
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign) and any(
            getattr(target, "id", None) == "RECORDED_TOPICS" for target in node.targets
        ):
            return [
                element.value
                for element in ast.walk(node.value)
                if isinstance(element, ast.Constant)
            ]
    return []


if __name__ == "__main__":
    unittest.main()
