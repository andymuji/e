"""The controller and the goal checker must agree on when the robot has arrived.

DWB's RotateToGoal critic stops forward motion inside FollowPath's
xy_goal_tolerance (0.25 m when unset) and only lets the robot turn. If that
radius is wider than the goal checker's, the robot can stop between the two,
turning on the spot, and the goal never succeeds.
"""

from pathlib import Path
import unittest

import yaml

NAV2_YAML = Path(__file__).resolve().parents[1] / "config" / "nav2.yaml"


class Nav2GoalToleranceTests(unittest.TestCase):
    def test_controller_and_goal_checker_share_the_xy_tolerance(self) -> None:
        params = yaml.safe_load(NAV2_YAML.read_text())["controller_server"][
            "ros__parameters"
        ]
        self.assertIn(
            "xy_goal_tolerance",
            params["FollowPath"],
            "unset, DWB falls back to 0.25 m",
        )
        self.assertEqual(
            params["FollowPath"]["xy_goal_tolerance"],
            params["goal_checker"]["xy_goal_tolerance"],
        )


if __name__ == "__main__":
    unittest.main()
