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

from robot_bringup.scan_to_map import render
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


if __name__ == "__main__":
    unittest.main()
