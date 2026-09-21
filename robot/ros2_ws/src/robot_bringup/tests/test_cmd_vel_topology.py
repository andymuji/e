"""Static regression checks for the one-way motion-command topology.

These tests intentionally read Python syntax trees instead of launching ROS.
The safety property must be checked in CI too: a launch comment describing the
gate is not wiring, and a developer machine is not always available to run a
topic graph probe.
"""

import ast
from pathlib import Path
import unittest

BRINGUP = Path(__file__).resolve().parents[1]
LAUNCH = BRINGUP / "launch"
WORKSPACE_SRC = BRINGUP.parent
NAV2_VELOCITY_EXECUTABLES = {
    "controller_server",
    "behavior_server",
    "velocity_smoother",
}


def node_calls(source: str) -> list[ast.Call]:
    """Return Node and LifecycleNode calls, excluding text and comments."""
    return [
        node
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) in {"Node", "LifecycleNode"}
    ]


def keyword(call: ast.Call, name: str) -> ast.AST | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def constant_strings(node: ast.AST, assignments: dict[str, ast.AST]) -> set[str]:
    """Resolve string literals through simple launch-file assignments."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return {node.value}
    if isinstance(node, ast.Name) and node.id in assignments:
        return constant_strings(assignments[node.id], assignments)
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        return set().union(*(constant_strings(item, assignments) for item in node.elts))
    if isinstance(node, ast.Dict):
        return set().union(*(constant_strings(item, assignments) for item in node.values))
    return set()


def assignments(tree: ast.AST) -> dict[str, ast.AST]:
    """Names used by the concise launch-file wiring, including local aliases."""
    return {
        target.id: node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }


def remapping_pairs(call: ast.Call, known: dict[str, ast.AST]) -> set[tuple[str, str]]:
    """Literal remappings, resolving a name such as navigation's ``gated``."""
    remappings = keyword(call, "remappings")
    if isinstance(remappings, ast.Name):
        remappings = known.get(remappings.id)
    if not isinstance(remappings, (ast.List, ast.Tuple)):
        return set()

    pairs = set()
    for item in remappings.elts:
        if not isinstance(item, (ast.List, ast.Tuple)) or len(item.elts) != 2:
            continue
        source, destination = item.elts
        if all(isinstance(value, ast.Constant) and isinstance(value.value, str)
               for value in (source, destination)):
            pairs.add((source.value, destination.value))
    return pairs


def is_safety_gate(call: ast.Call) -> bool:
    package = keyword(call, "package")
    executable = keyword(call, "executable")
    return (
        isinstance(package, ast.Constant)
        and package.value == "robot_safety"
        and isinstance(executable, ast.Constant)
        and executable.value == "safety_node"
    )


def published_topics(source: str) -> set[str]:
    """Topics passed to create_publisher, resolving module-level constants."""
    tree = ast.parse(source)
    known = assignments(tree)
    topics = set()
    for call in ast.walk(tree):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, "attr", None) == "create_publisher"
            and len(call.args) >= 2
        ):
            topics.update(constant_strings(call.args[1], known))
    return topics


class CmdVelTopologyTests(unittest.TestCase):
    """The gate is the sole authority allowed to reach the wheel topic."""

    def setUp(self) -> None:
        self.launches = {
            path.name: (ast.parse(path.read_text()), path.read_text())
            for path in sorted(LAUNCH.glob("*.launch.py"))
        }

    def test_no_launch_node_is_remapped_onto_cmd_vel(self) -> None:
        for filename, (tree, source) in self.launches.items():
            known = assignments(tree)
            for call in node_calls(source):
                with self.subTest(launch=filename, node=ast.unparse(call)):
                    destinations = {
                        destination
                        for _, destination in remapping_pairs(call, known)
                    }
                    self.assertNotIn("/cmd_vel", destinations)
                    self.assertNotIn("cmd_vel", destinations)

    def test_every_nav2_velocity_source_is_remapped_to_the_gate_request(self) -> None:
        navigation_tree, navigation = self.launches["navigation.launch.py"]
        known = assignments(navigation_tree)
        found = set()

        for call in node_calls(navigation):
            executable = keyword(call, "executable")
            if not isinstance(executable, ast.Constant):
                continue
            if executable.value not in NAV2_VELOCITY_EXECUTABLES:
                continue
            found.add(executable.value)
            with self.subTest(node=executable.value):
                self.assertIn(
                    ("/cmd_vel", "/cmd_vel_requested"),
                    remapping_pairs(call, known),
                )

        # behavior_server contains recovery behaviours that drive while a
        # goal has already failed. It must not silently disappear from review.
        self.assertEqual(found, {"controller_server", "behavior_server"})

    def test_only_the_safety_gate_can_be_the_cmd_vel_authority(self) -> None:
        gates = []
        for filename, (_, source) in self.launches.items():
            for call in node_calls(source):
                if is_safety_gate(call):
                    gates.append((filename, call))

        # The simulation gate is conditional; navigation supplies its own
        # stricter gate. Both may exist in source, but no other launcher may
        # introduce a second command authority.
        self.assertEqual({filename for filename, _ in gates}, {
            "simulation.launch.py", "navigation.launch.py"
        })

    def test_no_lifecycle_manager_can_deactivate_the_gate(self) -> None:
        for filename, (tree, source) in self.launches.items():
            known = assignments(tree)
            for call in node_calls(source):
                package = keyword(call, "package")
                if not (isinstance(package, ast.Constant)
                        and package.value == "nav2_lifecycle_manager"):
                    continue
                for parameters in [keyword(call, "parameters")]:
                    if not isinstance(parameters, ast.List):
                        continue
                    for item in parameters.elts:
                        if not isinstance(item, ast.Dict):
                            continue
                        for key, value in zip(item.keys, item.values, strict=False):
                            if isinstance(key, ast.Constant) and key.value == "node_names":
                                names = constant_strings(value, known)
                                with self.subTest(launch=filename, names=names):
                                    self.assertNotIn("safety_controller", names)
                                    self.assertNotIn("robot_safety", names)

    def test_voice_and_telemetry_cannot_release_an_emergency_stop(self) -> None:
        for package in ("robot_voice", "robot_telemetry"):
            module_root = WORKSPACE_SRC / package / package
            for path in module_root.glob("*.py"):
                with self.subTest(package=package, file=path.name):
                    self.assertNotIn("emergency_stop_reset", published_topics(path.read_text()))


if __name__ == "__main__":
    unittest.main()
