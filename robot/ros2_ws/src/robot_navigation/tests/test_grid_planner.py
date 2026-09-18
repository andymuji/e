from collections import deque
from itertools import pairwise
import random
import unittest

from robot_navigation import GridPlanner, NoPathError


def shortest_length(
    width: int, height: int, obstacles: set, start: tuple, goal: tuple
) -> int | None:
    """Breadth-first oracle: the true 4-connected distance, or None."""
    if start == goal:
        return 0
    queue = deque([(start, 0)])
    seen = {start}
    while queue:
        (x, y), distance = queue.popleft()
        for cell in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if not (0 <= cell[0] < width and 0 <= cell[1] < height):
                continue
            if cell in obstacles or cell in seen:
                continue
            if cell == goal:
                return distance + 1
            seen.add(cell)
            queue.append((cell, distance + 1))
    return None


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

    def test_rejects_occupied_start(self) -> None:
        planner = GridPlanner(2, 2, {(0, 0)})

        with self.assertRaises(ValueError):
            planner.plan((0, 0), (1, 1))

    def test_rejects_cells_off_the_grid(self) -> None:
        planner = GridPlanner(3, 3)

        for start, goal in (
            ((-1, 0), (2, 2)),
            ((0, -1), (2, 2)),
            ((0, 0), (3, 0)),
            ((0, 0), (0, 3)),
        ):
            with self.subTest(start=start, goal=goal):
                with self.assertRaises(ValueError):
                    planner.plan(start, goal)

    def test_rejects_obstacles_off_the_grid(self) -> None:
        with self.assertRaises(ValueError):
            GridPlanner(3, 3, {(3, 0)})

        planner = GridPlanner(3, 3, {(1, 1)})
        with self.assertRaises(ValueError):
            planner.set_obstacles({(0, 9)})

        # The refused update must not have been half-applied.
        self.assertEqual(planner._obstacles, {(1, 1)})

    def test_rejects_non_positive_or_non_integer_dimensions(self) -> None:
        for width, height in ((0, 3), (3, 0), (-1, 3), (3, -1)):
            with self.subTest(width=width, height=height):
                with self.assertRaises(ValueError):
                    GridPlanner(width, height)

        for width, height in ((4.5, 4), (4, 4.5), ("4", 4)):
            with self.subTest(width=width, height=height):
                with self.assertRaises(ValueError):
                    GridPlanner(width, height)

    def test_refuses_obstacles_it_could_not_honour(self) -> None:
        # A cell that is not exactly two integers never compares equal to a
        # neighbour, so an unvalidated one is accepted and then planned
        # through: the planner would report a clear route across an obstacle
        # it was explicitly given.
        for bad in ((1.5, 1), (1, 1, 1), (1,), "ab", 1, None):
            with self.subTest(obstacle=bad):
                with self.assertRaises(ValueError):
                    GridPlanner(4, 4, [bad])

    def test_refuses_endpoints_that_are_not_integer_cells(self) -> None:
        # Reported as bad input, not as "no path": a float start used to walk
        # its own offset lattice and fail with NoPathError, which blames the
        # map for what is a caller mistake.
        planner = GridPlanner(4, 4)

        for start, goal in (((0.5, 0), (2, 0)), ((0, 0), (2.5, 0))):
            with self.subTest(start=start, goal=goal):
                with self.assertRaises(ValueError):
                    planner.plan(start, goal)

    def test_path_cells_are_plain_integers(self) -> None:
        # bools are ints in Python, and used to travel straight through into
        # the returned path as `(True, False)`.
        path = GridPlanner(4, 4).plan((True, False), (2, 0))

        self.assertEqual(path[0], (1, 0))
        for cell in path:
            for coordinate in cell:
                self.assertIs(type(coordinate), int)

    def test_start_equal_to_goal_is_a_single_cell_path(self) -> None:
        self.assertEqual(GridPlanner(4, 4).plan((1, 1), (1, 1)), [(1, 1)])

    def test_plans_on_a_single_cell_grid(self) -> None:
        self.assertEqual(GridPlanner(1, 1).plan((0, 0), (0, 0)), [(0, 0)])

    def test_reports_no_path_when_the_goal_is_walled_off(self) -> None:
        # The goal cell itself is free, so this is a genuine no-path rather
        # than a rejected input.
        planner = GridPlanner(3, 3, {(1, 2), (2, 1)})

        with self.assertRaises(NoPathError):
            planner.plan((0, 0), (2, 2))

    def test_path_never_steps_diagonally(self) -> None:
        # (1, 1) and (2, 2) touch only at a corner. A diagonal step from
        # (2, 1) to (1, 2) would squeeze through that corner gap, which is
        # narrower than the robot and not something the footprint clearance
        # covers. The planner stays 4-connected; this fails if diagonal moves
        # are ever added without revisiting clearance.
        planner = GridPlanner(4, 4, {(1, 1), (2, 2)})

        path = planner.plan((0, 0), (3, 3))

        for first, second in pairwise(path):
            step = abs(first[0] - second[0]) + abs(first[1] - second[1])
            self.assertEqual(step, 1, f"diagonal or jumped step {first} -> {second}")

    def test_paths_are_shortest_and_repeatable(self) -> None:
        # Checked against a breadth-first oracle over many random grids: an
        # A* that loses optimality, wanders off the grid, clips an obstacle,
        # or depends on set iteration order shows up here.
        rng = random.Random(20260918)

        for _ in range(300):
            width = rng.randint(1, 6)
            height = rng.randint(1, 6)
            cells = [(x, y) for x in range(width) for y in range(height)]
            obstacles = set(rng.sample(cells, k=rng.randint(0, len(cells) - 1)))
            free = [cell for cell in cells if cell not in obstacles]
            start = rng.choice(free)
            goal = rng.choice(free)
            planner = GridPlanner(width, height, obstacles)
            expected = shortest_length(width, height, obstacles, start, goal)

            with self.subTest(size=(width, height), start=start, goal=goal):
                if expected is None:
                    with self.assertRaises(NoPathError):
                        planner.plan(start, goal)
                    continue

                path = planner.plan(start, goal)

                self.assertEqual(path[0], start)
                self.assertEqual(path[-1], goal)
                self.assertEqual(len(path) - 1, expected)
                self.assertTrue(obstacles.isdisjoint(path))
                self.assertEqual(len(set(path)), len(path))
                # Same inputs, same route, every time.
                self.assertEqual(
                    GridPlanner(width, height, set(obstacles)).plan(start, goal), path
                )

    def test_clearing_obstacles_restores_the_direct_route(self) -> None:
        planner = GridPlanner(4, 3, {(1, 1), (1, 0), (1, 2)})
        with self.assertRaises(NoPathError):
            planner.plan((0, 1), (3, 1))

        planner.set_obstacles(())

        self.assertEqual(planner.plan((0, 1), (3, 1)), [(0, 1), (1, 1), (2, 1), (3, 1)])


if __name__ == "__main__":
    unittest.main()
