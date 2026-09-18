"""Tests for the costmap geometry derived from the base dimensions."""

import ast
import math
import unittest

from robot_navigation import BaseFootprint


def footprint(**overrides) -> BaseFootprint:
    values = {"length": 0.40, "width": 0.30, "clearance_margin": 0.25}
    values.update(overrides)
    return BaseFootprint(**values)


class BaseFootprintTests(unittest.TestCase):
    def test_rejects_non_positive_and_non_finite_dimensions(self) -> None:
        for field in ("length", "width", "distance_step"):
            for bad in (0.0, -1.0, float("inf"), float("nan")):
                with self.subTest(field=field, value=bad):
                    with self.assertRaises(ValueError):
                        footprint(**{field: bad})

    def test_allows_a_zero_clearance_margin_but_not_a_negative_one(self) -> None:
        self.assertIsNotNone(footprint(clearance_margin=0.0))
        with self.assertRaises(ValueError):
            footprint(clearance_margin=-0.1)

    def test_polygon_encloses_the_whole_base(self) -> None:
        corners = footprint().polygon

        self.assertEqual(len(corners), 4)
        self.assertAlmostEqual(max(x for x, _ in corners), 0.20)
        self.assertAlmostEqual(min(x for x, _ in corners), -0.20)
        self.assertAlmostEqual(max(y for _, y in corners), 0.15)
        self.assertAlmostEqual(min(y for _, y in corners), -0.15)

    def test_inscribed_radius_fits_inside_the_base(self) -> None:
        self.assertAlmostEqual(footprint().inscribed_radius, 0.15)

    def test_circumscribed_radius_reaches_the_corners(self) -> None:
        base = footprint()
        corner = max(math.hypot(x, y) for x, y in base.polygon)

        self.assertAlmostEqual(base.circumscribed_radius, corner)

    def test_circumscribed_radius_is_never_smaller_than_the_inscribed_one(
        self,
    ) -> None:
        for length, width in ((0.4, 0.3), (0.3, 0.4), (0.5, 0.5), (1.0, 0.2)):
            with self.subTest(length=length, width=width):
                base = footprint(length=length, width=width)

                self.assertGreaterEqual(
                    base.circumscribed_radius, base.inscribed_radius
                )

    def test_inflation_never_sits_inside_the_robot(self) -> None:
        # A planner inflating less than this produces paths the robot cannot
        # rotate on the spot to follow.
        for margin in (0.0, 0.05, 0.25, 1.0):
            with self.subTest(margin=margin):
                base = footprint(clearance_margin=margin)

                self.assertGreaterEqual(
                    base.inflation_radius, base.circumscribed_radius
                )

    def test_inflation_rounds_up_never_down(self) -> None:
        base = footprint(clearance_margin=0.21, distance_step=0.05)

        self.assertGreaterEqual(
            base.inflation_radius, base.circumscribed_radius + 0.21
        )

    def test_a_bigger_base_needs_more_inflation(self) -> None:
        small = footprint(length=0.3, width=0.2)
        large = footprint(length=0.8, width=0.6)

        self.assertGreater(large.inflation_radius, small.inflation_radius)

    def test_costmap_footprint_is_parseable_back_into_the_polygon(self) -> None:
        base = footprint()
        parsed = [tuple(corner) for corner in ast.literal_eval(
            base.as_costmap_footprint()
        )]

        for actual, expected in zip(parsed, base.polygon, strict=True):
            self.assertAlmostEqual(actual[0], expected[0], places=6)
            self.assertAlmostEqual(actual[1], expected[1], places=6)

    def test_costmap_footprint_round_trips_measured_dimensions(self) -> None:
        # The bringup drift check compares this string back against the URDF
        # to 6 decimal places. Dimensions only round-trip while they stay
        # round numbers, so this uses the awkward ones a tape measure gives.
        for length, width in ((0.4123, 0.3087), (0.123456, 0.98765), (1.2344449, 0.7)):
            with self.subTest(length=length, width=width):
                base = footprint(length=length, width=width)
                parsed = [tuple(corner) for corner in ast.literal_eval(
                    base.as_costmap_footprint()
                )]

                for actual, expected in zip(parsed, base.polygon, strict=True):
                    self.assertAlmostEqual(actual[0], expected[0], places=6)
                    self.assertAlmostEqual(actual[1], expected[1], places=6)

    def test_costmap_footprint_never_reports_a_base_smaller_than_it_is(self) -> None:
        # Rounding that shrinks the footprint is the dangerous direction: the
        # planner then believes the robot fits through gaps it does not.
        for length, width in ((0.4, 0.3), (0.4123, 0.3087), (0.123456, 0.98765)):
            with self.subTest(length=length, width=width):
                base = footprint(length=length, width=width)
                corners = [
                    tuple(corner)
                    for corner in ast.literal_eval(base.as_costmap_footprint())
                ]

                self.assertGreaterEqual(max(x for x, _ in corners), length / 2.0)
                self.assertGreaterEqual(max(y for _, y in corners), width / 2.0)

    def test_polygon_corner_order_is_the_one_nav2_yaml_restates(self) -> None:
        # Nav2's footprint string lists these corners in this order and the
        # bringup test compares them one by one. Reordering here silently
        # breaks that pairing, so the order is pinned.
        self.assertEqual(
            footprint().polygon,
            [(0.2, 0.15), (0.2, -0.15), (-0.2, -0.15), (-0.2, 0.15)],
        )

    def test_polygon_winds_clockwise_as_documented(self) -> None:
        corners = footprint().polygon
        shoelace = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(
                corners, corners[1:] + corners[:1], strict=True
            )
        )

        # Negative in a right-handed x-forward, y-left frame means clockwise.
        self.assertLess(shoelace, 0.0)

    def test_polygon_area_matches_the_base_dimensions(self) -> None:
        corners = footprint(length=0.4123, width=0.3087).polygon
        area = abs(
            sum(
                x1 * y2 - x2 * y1
                for (x1, y1), (x2, y2) in zip(
                    corners, corners[1:] + corners[:1], strict=True
                )
            )
        ) / 2.0

        self.assertAlmostEqual(area, 0.4123 * 0.3087, places=9)


if __name__ == "__main__":
    unittest.main()
