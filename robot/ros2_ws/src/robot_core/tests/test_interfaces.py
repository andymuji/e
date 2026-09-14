import unittest

from robot_core import NavigationGoal, Pose2D


class InterfaceTests(unittest.TestCase):
    def test_navigation_goal_requires_a_name(self) -> None:
        with self.assertRaises(ValueError):
            NavigationGoal("", Pose2D(0.0, 0.0))

    def test_pose_rejects_non_finite_values(self) -> None:
        with self.assertRaises(ValueError):
            Pose2D(float("nan"), 0.0)


if __name__ == "__main__":
    unittest.main()
