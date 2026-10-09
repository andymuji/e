"""Checks on turning a phone scan into a map.

The scan is a made-up 3 m x 2 m room written as glTF, the way Polycam
exports one: y-up, metres, under a node that turns and moves it. A wall
must come out occupied, scanned floor free, a table top above the lidar's
height must not block the floor under it, and outside the walls stays
unknown.
"""

import base64
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest

from robot_bringup.scan_to_map import LIDAR_HEIGHT, NothingAtHeight, render, report
from robot_bringup.world_to_map import FREE, OCCUPIED, UNKNOWN


def box(x0, y0, z0, x1, y1, z1):
    """A box's faces as glTF-frame triangles - all but the bottom, which a
    phone cannot see, so the inside of a wall is never mistaken for floor."""
    c = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [tri for a, b, d, e in faces for tri in ((c[a], c[b], c[d]), (c[a], c[d], c[e]))]


ROOM = (
    [((0, 0, 0), (3, 0, 0), (3, 0, 2)), ((0, 0, 0), (3, 0, 2), (0, 0, 2))]  # floor
    + box(0, 0, -0.1, 3, 2.4, 0)  # four walls
    + box(0, 0, 2, 3, 2.4, 2.1)
    + box(-0.1, 0, 0, 0, 2.4, 2)
    + box(3, 0, 0, 3.1, 2.4, 2)
    + box(1.0, 0, 0.8, 1.4, 0.45, 1.2)  # a stool, under the lidar's height
    + box(2.0, 0.7, 0.5, 2.6, 0.75, 1.0)  # a table top, over it
)


def write_scan(path: Path, triangles, binary: bool) -> None:
    data = b"".join(struct.pack("<3f", *p) for tri in triangles for p in tri)
    gltf = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        # Turned a quarter about the vertical and moved: map must not care.
        "nodes": [{"children": [1], "translation": [5, 0.3, -2]},
                  {"mesh": 0, "rotation": [0, math.sin(math.pi / 4), 0,
                                           math.cos(math.pi / 4)]}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "accessors": [{"bufferView": 0, "componentType": 5126,
                       "count": len(triangles) * 3, "type": "VEC3"}],
        "bufferViews": [{"buffer": 0, "byteLength": len(data)}],
        "buffers": [{"byteLength": len(data)}],
    }
    if binary:
        text = json.dumps(gltf).encode()
        text += b" " * (-len(text) % 4)
        body = (struct.pack("<II", len(text), 0x4E4F534A) + text
                + struct.pack("<II", len(data), 0x004E4942) + data)
        path.write_bytes(b"glTF" + struct.pack("<II", 2, 12 + len(body)) + body)
    else:
        gltf["buffers"][0]["uri"] = (
            "data:application/octet-stream;base64," + base64.b64encode(data).decode())
        path.write_text(json.dumps(gltf))


class ScanToMapTests(unittest.TestCase):
    def render(self, binary=True):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / ("room.glb" if binary else "room.gltf")
            write_scan(path, ROOM, binary)
            return render(path)

    def counts(self, grid):
        flat = [c for row in grid.cells for c in row]
        return flat.count(FREE), flat.count(OCCUPIED), flat.count(UNKNOWN)

    def test_room_size_survives_the_turn(self):
        grid = self.render()
        # 3.2 x 2.2 m of walls plus 0.3 m margin each side, turned a quarter.
        self.assertAlmostEqual(grid.width * grid.resolution, 2.2 + 0.6, delta=0.1)
        self.assertAlmostEqual(grid.height * grid.resolution, 3.2 + 0.6, delta=0.1)

    def test_walls_stool_and_floor(self):
        grid = self.render()
        free, occupied, unknown = self.counts(grid)
        # Open floor: 3 x 2 m less the stool and the walls' edge cells; the
        # table top is above the slice and takes nothing.
        self.assertGreater(free * grid.resolution ** 2, 5.0)
        self.assertLess(free * grid.resolution ** 2, 6.0)
        # The four walls' perimeter, a cell or two thick, plus the stool.
        self.assertGreater(occupied, 2 * (60 + 40))
        self.assertGreater(unknown, 0)

    def test_outside_corner_is_unknown_and_centre_is_free(self):
        grid = self.render()
        self.assertEqual(grid.cells[0][0], UNKNOWN)
        self.assertEqual(grid.cells[grid.height // 2][grid.width // 2 - 8], FREE)

    def test_gltf_with_embedded_buffer_matches_glb(self):
        self.assertEqual(self.render(binary=True).cells, self.render(binary=False).cells)



REAL_SCAN = Path(__file__).resolve().parents[5] / "docs" / "runs" / "10_5_2026.glb"


def floor(x0, z0, x1, z1, step=0.1, tilt=0.0):
    """A floor of small triangles, as a phone scan makes it, rising `tilt`
    metres per metre along glTF x - a floor or a phone not quite level."""
    tris = []
    nx, nz = round((x1 - x0) / step), round((z1 - z0) / step)
    for i in range(nx):
        for j in range(nz):
            a, b = x0 + i * step, x0 + (i + 1) * step
            c, d = z0 + j * step, z0 + (j + 1) * step
            tris += [((a, a * tilt, c), (b, b * tilt, c), (b, b * tilt, d)),
                     ((a, a * tilt, c), (b, b * tilt, d), (a, a * tilt, d))]
    return tris


class ObstacleReportTests(unittest.TestCase):
    """A patch of floor with things on it, and no walls: what a phone scan of
    a corner of a room looks like before the room is finished."""

    def report(self, scene):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "patch.glb"
            write_scan(path, scene, binary=True)
            return report(path)

    def test_low_box_is_a_trip_hazard_the_lidar_misses(self):
        found = self.report(floor(0, 0, 2, 2, tilt=0.01) + box(0.8, 0.01, 0.8, 1.1, 0.11, 1.0))
        self.assertEqual(len(found["obstacles"]), 1)
        stool = found["obstacles"][0]
        # 10 cm tall, sitting on a floor that rises 1 cm per metre.
        self.assertAlmostEqual(stool["height"], 0.1, delta=0.01)
        self.assertTrue(stool["trip_hazard"])
        self.assertFalse(stool["lidar_sees"])
        self.assertAlmostEqual(found["floor"]["tilt_deg"], math.degrees(math.atan(0.01)),
                               delta=0.1)
        # 0.3 x 0.2 m, a cell or so either way.
        self.assertAlmostEqual(stool["size"][0] * stool["size"][1], 0.06, delta=0.04)

    def test_box_above_the_lidar_is_seen(self):
        found = self.report(floor(0, 0, 2, 2) + box(0.8, 0, 0.8, 1.1, 0.4, 1.0))
        (tall,) = found["obstacles"]
        self.assertGreater(tall["height"], LIDAR_HEIGHT)
        self.assertTrue(tall["lidar_sees"])
        self.assertFalse(tall["trip_hazard"])

    def test_rim_where_the_scan_runs_out_is_not_an_obstacle(self):
        # The last strip of the scan curls up 10 cm, as a Polycam edge does.
        rim = [tuple((x, 0.1 if x == 2.2 else y, z) for x, y, z in tri)
               for tri in floor(2.0, 0, 2.2, 2, step=0.2)]
        found = self.report(floor(0, 0, 2, 2) + rim)
        self.assertEqual(found["obstacles"], [])
        # Not measured, but not thrown away either.
        self.assertEqual(len(found["unmeasured"]), 1)
        self.assertLess(found["floor"]["tilt_deg"], 0.5)

    def test_box_with_unscanned_floor_behind_it_is_unmeasured_not_lost(self):
        # The phone never saw the strip behind the box: its shadow.
        scene = (floor(0, 0, 2, 0.8) + floor(0, 1.1, 2, 2)
                 + box(0.8, 0, 0.5, 1.1, 0.1, 0.8))
        found = self.report(scene)
        self.assertEqual(found["obstacles"], [])
        (hidden,) = found["unmeasured"]
        self.assertAlmostEqual(hidden["height"], 0.1, delta=0.01)

    def test_big_tilted_floor_is_not_an_obstacle(self):
        # 1 degree over 6 m is 10 cm of rise: more than the floor band.
        found = self.report(floor(0, 0, 6, 6, step=0.2, tilt=math.tan(math.radians(1))))
        self.assertEqual(found["obstacles"], [])
        self.assertEqual(found["unmeasured"], [])
        self.assertAlmostEqual(found["floor"]["tilt_deg"], 1.0, delta=0.05)

    def test_too_little_floor_says_so(self):
        with self.assertRaisesRegex(ValueError, "too little floor"):
            self.report([((0, 0, 0), (0.01, 0, 0), (0, 0, 0.01))])

    def test_slice_with_nothing_at_lidar_height_says_so(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "patch.glb"
            write_scan(path, floor(0, 0, 2, 2) + box(0.8, 0, 0.8, 1.1, 0.1, 1.0), True)
            with self.assertRaises(NothingAtHeight):
                render(path)

    @unittest.skipUnless(REAL_SCAN.exists(), "the 10_5_2026 phone scan is not checked out")
    def test_real_scan_has_five_balls_the_lidar_cannot_see(self):
        found = report(REAL_SCAN)
        balls = [(-0.47, 0.19), (0.15, 0.40), (-0.20, -0.22), (0.29, -0.27), (0.05, -0.58)]
        self.assertEqual(len(found["obstacles"]), 5)
        for x, y in balls:
            nearest = min(found["obstacles"], key=lambda o: math.dist(o["center"], (x, y)))
            self.assertLess(math.dist(nearest["center"], (x, y)), 0.08)
            self.assertTrue(nearest["trip_hazard"])
            self.assertFalse(nearest["lidar_sees"])


if __name__ == "__main__":
    unittest.main()
