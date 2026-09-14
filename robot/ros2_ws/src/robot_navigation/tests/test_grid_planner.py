import unittest

from robot_navigation import GridPlanner, NoPathError


class GridPlannerTests(unittest.TestCase):
    def test_plans_around_obstacle_wall(self) -> None:
        planner = GridPlanner(5, 5, {(2, 0), (2, 1), (2, 2), (2, 3)})

        path = planner.plan((0, 0), (4, 0))

        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 0))
        self.assertTrue(all(cell not in planner._obstacles for cell in path))
        self.assertGreater(len(path), 5)

    def test_dynamic_obstacle_requires_replanning(self) -> None:
        planner = GridPlanner(4, 3)
        first_path = planner.plan((0, 1), (3, 1))
        planner.set_obstacles({first_path[1]})

        second_path = planner.plan((0, 1), (3, 1))

        self.assertNotEqual(first_path, second_path)
        self.assertNotIn(first_path[1], second_path)

    def test_reports_no_path(self) -> None:
        planner = GridPlanner(3, 3, {(1, 0), (1, 1), (1, 2)})

        with self.assertRaises(NoPathError):
            planner.plan((0, 1), (2, 1))

    def test_rejects_occupied_goal(self) -> None:
        planner = GridPlanner(2, 2, {(1, 1)})

        with self.assertRaises(ValueError):
            planner.plan((0, 0), (1, 1))


if __name__ == "__main__":
    unittest.main()
