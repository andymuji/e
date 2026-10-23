"""The made-up house: repeatable, fully connected, and routed clear of walls."""

from itertools import pairwise
import math
import unittest

from robot_bringup.house import ROOM_NAMES, generate, robot_radius, routes


def distance_to_wall(point, wall) -> float:
    x, y, sx, sy = wall
    dx = max(abs(point[0] - x) - sx / 2, 0.0)
    dy = max(abs(point[1] - y) - sy / 2, 0.0)
    return math.hypot(dx, dy)


class HouseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.radius = robot_radius()
        cls.house = generate(7)
        cls.routes = routes(cls.house, cls.radius)

    def test_same_seed_same_house(self) -> None:
        self.assertEqual(generate(7), self.house)
        self.assertNotEqual(generate(8)["walls"], self.house["walls"])

    def test_every_room_and_the_front_door_is_a_place(self) -> None:
        self.assertEqual(set(self.house["locations"]), {*ROOM_NAMES, "front door"})

    def test_every_place_reaches_every_other(self) -> None:
        places = len(self.house["locations"])
        self.assertEqual(len(self.routes), places * (places - 1) // 2)
        for route in self.routes:
            start, end = (self.house["locations"][route[k]] for k in ("from", "to"))
            self.assertLess(math.dist(route["path"][0], start), 0.05)
            self.assertLess(math.dist(route["path"][-1], end), 0.05)

    def test_no_route_brushes_a_wall(self) -> None:
        # Sampled every 2 cm along each straight leg. Half a grid cell of
        # slack: the planner works on 5 cm cells, the walls do not.
        for route in self.routes:
            for p, q in pairwise(route["path"]):
                steps = max(1, int(math.dist(p, q) / 0.02))
                for i in range(steps + 1):
                    point = (
                        p[0] + (q[0] - p[0]) * i / steps,
                        p[1] + (q[1] - p[1]) * i / steps,
                    )
                    nearest = min(distance_to_wall(point, w) for w in self.house["walls"])
                    self.assertGreater(
                        nearest,
                        self.radius - 0.025,
                        f"{route['from']} -> {route['to']} at {point}",
                    )


if __name__ == "__main__":
    unittest.main()
