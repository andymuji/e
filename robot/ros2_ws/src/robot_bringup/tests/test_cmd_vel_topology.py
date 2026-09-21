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


class Unresolvable(Exception):
    """A launch value this reader cannot prove anything about.

    Raised rather than returned as an empty set. These tests assert that a
    name is *absent* from a list, and "I could not read the list" is not
    evidence of absence. Returning `set()` for an unreadable expression makes
    every such assertion pass without checking anything, which is the one
    failure mode a safety regression test must not have.
    """


def constant_strings(node: ast.AST, assignments: dict[str, ast.AST]) -> set[str]:
    """Every string a launch expression could evaluate to.

    Over-approximates on purpose. The callers ask "is this name absent?", so
    returning a superset can only cause a false failure, never a false pass.
    A `+` is exact; a comprehension is widened to everything it draws from,
    because a filtered subset of a list cannot contain anything the list did
    not. Anything else raises `Unresolvable`.
    """
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return {node.value}
    if isinstance(node, ast.Name):
        if node.id not in assignments:
            raise Unresolvable(f"name {node.id!r} is not assigned in this file")
        return constant_strings(assignments[node.id], assignments)
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        return set().union(
            set(), *(constant_strings(item, assignments) for item in node.elts)
        )
    if isinstance(node, ast.Dict):
        return set().union(
            set(), *(constant_strings(item, assignments) for item in node.values)
        )
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        # MANAGED_NODES + ["safety_controller"] - exact, and the shape that
        # slipped past the first version of this file.
        return constant_strings(node.left, assignments) | constant_strings(
            node.right, assignments
        )
    if isinstance(node, (ast.ListComp, ast.SetComp, ast.GeneratorExp)):
        # [n for n in MANAGED_NODES if n not in LOCALIZATION_NODES] - the
        # result is a subset of what it iterates, which is enough to prove a
        # name absent.
        return set().union(
            set(),
            *(
                constant_strings(generator.iter, assignments)
                for generator in node.generators
            ),
        )
    raise Unresolvable(f"cannot read a {type(node).__name__} as strings")


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


def published_topics(source: str) -> tuple[set[str], list[str]]:
    """Topics passed to create_publisher, and the ones that could not be read.

    The second element matters as much as the first: a topic name assembled at
    runtime is a publisher this reader cannot vouch for, and the caller is
    expected to fail rather than ignore it.
    """
    tree = ast.parse(source)
    known = assignments(tree)
    topics: set[str] = set()
    unreadable: list[str] = []
    for call in ast.walk(tree):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, "attr", None) == "create_publisher"
            and len(call.args) >= 2
        ):
            try:
                topics.update(constant_strings(call.args[1], known))
            except Unresolvable as reason:
                unreadable.append(f"line {call.lineno}: {reason}")
    return topics, unreadable


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
        """No package but robot_safety may create a /cmd_vel publisher.

        The launch files decide what is *started*; this decides what is even
        capable of reaching the wheels. A node that opens a cmd_vel publisher
        is a second authority whether or not a launch file starts it today.
        """
        offenders = []
        scanned = 0
        for package_root in sorted(WORKSPACE_SRC.iterdir()):
            package = package_root.name
            module_root = package_root / package
            if not module_root.is_dir():
                continue
            for path in sorted(module_root.rglob("*.py")):
                scanned += 1
                topics, unreadable = published_topics(path.read_text())
                relative = path.relative_to(WORKSPACE_SRC)
                self.assertEqual(
                    unreadable,
                    [],
                    f"{relative}: publisher topic cannot be read, so this "
                    "test cannot prove it is not /cmd_vel.",
                )
                if {"cmd_vel", "/cmd_vel"} & topics and package != "robot_safety":
                    offenders.append(str(relative))

        self.assertEqual(
            offenders,
            [],
            "only robot_safety may publish the topic that reaches the wheels",
        )
        # The gate itself must still be in there, or this test is scanning
        # nothing and would pass on an empty workspace.
        gate = WORKSPACE_SRC / "robot_safety" / "robot_safety" / "safety_node.py"
        gate_topics, _ = published_topics(gate.read_text())
        self.assertTrue(
            {"cmd_vel", "/cmd_vel"} & gate_topics,
            "robot_safety/safety_node.py no longer publishes cmd_vel; either "
            "the gate moved or this test is looking in the wrong place.",
        )
        self.assertGreater(scanned, 20, f"only scanned {scanned} modules")

    def test_the_gate_is_declared_only_where_it_is_expected(self) -> None:
        gates = {
            filename
            for filename, (_, source) in self.launches.items()
            for call in node_calls(source)
            if is_safety_gate(call)
        }

        # The simulation gate is conditional; navigation supplies its own
        # stricter gate. Both may exist in source, but no other launcher may
        # introduce a second command authority.
        self.assertEqual(
            gates, {"simulation.launch.py", "navigation.launch.py"}
        )

    def test_no_lifecycle_manager_can_deactivate_the_gate(self) -> None:
        examined = 0
        for filename, (tree, source) in self.launches.items():
            known = assignments(tree)
            for call in node_calls(source):
                package = keyword(call, "package")
                if not (isinstance(package, ast.Constant)
                        and package.value == "nav2_lifecycle_manager"):
                    continue
                parameters = keyword(call, "parameters")
                self.assertIsInstance(
                    parameters,
                    ast.List,
                    f"{filename}: a lifecycle manager whose parameters this "
                    "test cannot read. Rewrite it as a literal list, or teach "
                    "this test the new shape - do not leave it unchecked.",
                )
                for item in parameters.elts:
                    if not isinstance(item, ast.Dict):
                        continue
                    for key, value in zip(item.keys, item.values, strict=False):
                        if not (isinstance(key, ast.Constant)
                                and key.value == "node_names"):
                            continue
                        try:
                            names = constant_strings(value, known)
                        except Unresolvable as unreadable:
                            self.fail(
                                f"{filename}: cannot read the node_names of a "
                                f"lifecycle manager ({unreadable}). An "
                                "unreadable list is not proof the gate is "
                                "absent from it."
                            )
                        examined += 1
                        with self.subTest(launch=filename, names=sorted(names)):
                            self.assertNotIn("safety_controller", names)
                            self.assertNotIn("robot_safety", names)

        # Without this, deleting every lifecycle manager - or renaming the
        # parameter - would turn this test into a silent pass.
        self.assertGreaterEqual(
            examined,
            2,
            "expected to check the node_names of both lifecycle managers in "
            "navigation.launch.py; found "
            f"{examined}. The gate must be absent from a list this test "
            "actually read.",
        )

    def test_voice_and_telemetry_cannot_release_an_emergency_stop(self) -> None:
        for package in ("robot_voice", "robot_telemetry"):
            module_root = WORKSPACE_SRC / package / package
            self.assertTrue(module_root.is_dir(), f"{module_root} is missing")
            # rglob, not glob: a publisher hidden one directory down is still
            # a voice that can undo a stop.
            modules = sorted(module_root.rglob("*.py"))
            self.assertTrue(modules, f"no modules found under {module_root}")
            for path in modules:
                with self.subTest(package=package, file=path.name):
                    topics, unreadable = published_topics(path.read_text())
                    self.assertEqual(
                        unreadable,
                        [],
                        f"{path.name}: publisher topic cannot be read, so "
                        "this test cannot prove it is not emergency_stop_reset.",
                    )
                    self.assertNotIn("emergency_stop_reset", topics)
                    self.assertNotIn("/emergency_stop_reset", topics)


if __name__ == "__main__":
    unittest.main()
