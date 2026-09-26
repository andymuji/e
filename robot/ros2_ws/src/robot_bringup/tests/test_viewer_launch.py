"""Checks on viewer.launch.py: Foxglove may watch the robot, never act on it.

foxglove_bridge's defaults let any connected client publish, call services,
and set parameters. These checks read the launch file's syntax tree and fail
if any of that comes back.
"""

import ast
from pathlib import Path
import unittest

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "viewer.launch.py"

# The capabilities that let a client do something rather than see something.
ACTING_CAPABILITIES = {"clientPublish", "services", "parameters", "parametersSubscribe"}


class ViewerLaunchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tree = ast.parse(LAUNCH.read_text())

    def constant(self, name: str) -> object:
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Assign) and any(
                getattr(target, "id", None) == name for target in node.targets
            ):
                return ast.literal_eval(node.value)
        self.fail(f"{name} is not a literal assignment in viewer.launch.py")

    def bridge_parameters(self) -> dict[str, ast.expr]:
        nodes = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "Node"
        ]
        self.assertEqual(len(nodes), 1, "the viewer launch starts exactly one node")
        keywords = {keyword.arg: keyword.value for keyword in nodes[0].keywords}
        self.assertEqual(ast.literal_eval(keywords["package"]), "foxglove_bridge")
        self.assertNotIn("remappings", keywords)

        (params,) = keywords["parameters"].elts
        return {ast.literal_eval(key): value for key, value in zip(params.keys, params.values, strict=True)}

    def test_capabilities_are_set_and_watch_only(self) -> None:
        # Leaving `capabilities` out gives the bridge's defaults, which act.
        params = self.bridge_parameters()
        self.assertEqual(
            getattr(params["capabilities"], "id", None), "WATCH_ONLY_CAPABILITIES"
        )
        self.assertFalse(ACTING_CAPABILITIES & set(self.constant("WATCH_ONLY_CAPABILITIES")))

    def test_no_client_topic_is_allowed(self) -> None:
        params = self.bridge_parameters()
        self.assertEqual(ast.literal_eval(params["client_topic_whitelist"]), ["^$"])


if __name__ == "__main__":
    unittest.main()
