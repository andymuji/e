import base64
import struct
import unittest
import zlib

from robot_console.map_provider import GridSnapshot, RobotPose, RosMapProvider, build_map
from robot_console.occupancy_image import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    data_url,
    grayscale_png,
    shade,
    stride_for,
)


def png_pixels(url: str) -> tuple[int, int, list[list[int]]]:
    """Decode the grayscale PNG back, so the tests check pixels not bytes."""
    raw = base64.b64decode(url.split(",", 1)[1])
    width, height = struct.unpack(">II", raw[16:24])
    idat = b""
    offset = 8
    while offset < len(raw):
        length = struct.unpack(">I", raw[offset:offset + 4])[0]
        tag = raw[offset + 4:offset + 8]
        if tag == b"IDAT":
            idat += raw[offset + 8:offset + 8 + length]
        offset += length + 12
    flat = zlib.decompress(idat)
    rows = []
    for y in range(height):
        start = y * (width + 1)
        self_filter = flat[start]
        assert self_filter == 0
        rows.append(list(flat[start + 1:start + 1 + width]))
    return width, height, rows


def grid(data=None, width=2, height=2, resolution=1.0, origin=(0.0, 0.0)):
    return GridSnapshot(
        resolution=resolution,
        width=width,
        height=height,
        origin_x=origin[0],
        origin_y=origin[1],
        data=data if data is not None else [0] * (width * height),
    )


class OccupancyImageTests(unittest.TestCase):
    def test_unknown_cells_stay_unknown(self) -> None:
        # Drawing an unseen cell as floor tells an operator the opposite of
        # the truth about where the robot has been.
        self.assertEqual(shade(-1), UNKNOWN)
        self.assertEqual(shade(101), UNKNOWN)

    def test_walls_and_floor_are_black_and_white(self) -> None:
        self.assertEqual(shade(100), OCCUPIED)
        self.assertEqual(shade(0), FREE)

    def test_a_grid_is_drawn_bottom_row_last(self) -> None:
        # Grid data starts at the origin, which is the bottom-left corner in
        # the map frame; PNG rows run top down.
        width, height, rows = png_pixels(data_url([100, 0, 0, 0], 2, 2))

        self.assertEqual((width, height), (2, 2))
        self.assertEqual(rows[1][0], OCCUPIED)
        self.assertEqual(rows[0], [FREE, FREE])

    def test_a_truncated_grid_is_not_drawn(self) -> None:
        self.assertIsNone(data_url([0, 0, 0], 2, 2))
        self.assertIsNone(data_url([], 0, 0))

    def test_a_large_map_is_shrunk(self) -> None:
        self.assertEqual(stride_for(100, 100, 640_000), 1)
        self.assertGreater(stride_for(4000, 4000, 640_000), 1)

    def test_shrinking_keeps_a_thin_wall(self) -> None:
        # Sampling one cell per block loses a wall one cell thick, which is
        # exactly the feature an operator is looking at.
        data = [0] * 16
        data[5] = 100
        _, _, rows = png_pixels(data_url(data, 4, 4, max_pixels=4))

        self.assertIn(OCCUPIED, [value for row in rows for value in row])

    def test_the_png_header_says_grayscale(self) -> None:
        png = grayscale_png(1, 1, [bytes([FREE])])

        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(png[24], 8)  # bit depth
        self.assertEqual(png[25], 0)  # colour type: grayscale


class BuildMapTests(unittest.TestCase):
    def test_no_map_and_no_pose_invents_neither(self) -> None:
        payload = build_map(None, None, [])

        self.assertFalse(payload["available"])
        self.assertEqual(payload["walls"], [])
        self.assertIsNone(payload["robot"])
        self.assertNotIn("image", payload)
        self.assertIn("no map", payload["message"].lower())

    def test_a_map_without_a_pose_draws_no_robot(self) -> None:
        payload = build_map(grid(), None, [])

        self.assertTrue(payload["available"])
        self.assertIsNone(payload["robot"])
        self.assertIn("position is unknown", payload["message"])

    def test_the_pose_is_reported_where_the_transform_put_it(self) -> None:
        payload = build_map(grid(), RobotPose(1.5, 2.5, 0.25), [])

        self.assertEqual(payload["robot"], {"x": 1.5, "y": 2.5, "yaw": 0.25})
        self.assertEqual(payload["message"], "")

    def test_the_image_carries_the_grids_own_corners(self) -> None:
        payload = build_map(grid(origin=(-3.0, -4.0), resolution=0.5), None, [])

        self.assertEqual(payload["image"]["min_x"], -3.0)
        self.assertEqual(payload["image"]["max_x"], -2.0)
        self.assertEqual(payload["image"]["max_y"], -3.0)

    def test_bounds_hold_a_destination_outside_the_map(self) -> None:
        payload = build_map(grid(), None, [{"name": "shed", "x": 40.0, "y": 0.0}])

        self.assertGreater(payload["bounds"]["max_x"], 40.0)

    def test_bounds_are_never_a_zero_sized_box(self) -> None:
        payload = build_map(None, None, [{"name": "here", "x": 1.0, "y": 1.0}])
        bounds = payload["bounds"]

        self.assertGreater(bounds["max_x"] - bounds["min_x"], 0.0)
        self.assertGreater(bounds["max_y"] - bounds["min_y"], 0.0)

    def test_a_truncated_map_is_reported_not_drawn(self) -> None:
        payload = build_map(grid(data=[0], width=4, height=4), None, [])

        self.assertFalse(payload["available"])
        self.assertNotIn("image", payload)


class MapProviderTests(unittest.TestCase):
    def test_the_pose_is_read_on_every_draw(self) -> None:
        # Not cached: a marker left on screen after the transform tree went
        # quiet is a robot the operator thinks they can see.
        poses = [RobotPose(0.0, 0.0, 0.0), None]
        provider = RosMapProvider(lambda: poses.pop(0))
        provider.on_grid(grid())

        self.assertIsNotNone(provider.map_data([])["robot"])
        self.assertIsNone(provider.map_data([])["robot"])


if __name__ == "__main__":
    unittest.main()
