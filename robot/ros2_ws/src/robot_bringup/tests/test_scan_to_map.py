"""Checks on turning a phone scan into a map.

The scan is a made-up 3 m x 2 m room written as glTF, the way Polycam
exports one: y-up, metres, under a node that turns and moves it. A wall
must come out occupied, scanned floor free, a table top above the lidar's
height must not block the floor under it, and outside the walls stays
unknown.

The obstacle report's scenes are the ones an adversarial review broke the
first version with: a furnished room under a ceiling, things that are low at
the bottom and high at the top, a tilted floor, a step, a ghost below the
floor, and glTF files written in the ways Polycam does not but others do.
They are built here from boxes and floor tiles, small enough to run fast.
"""

import base64
import contextlib
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

from robot_bringup import scan_to_map
from robot_bringup.scan_to_map import (
    LIDAR_HEIGHT,
    NothingAtHeight,
    Scan,
    lidar_view,
    render,
    report,
    triangles,
)
from robot_bringup.world_to_map import FREE, OCCUPIED, UNKNOWN


def box(x0, y0, z0, x1, y1, z1, bottom=False):
    """A box's faces as glTF-frame triangles - all but the bottom, which a
    phone cannot see, so the inside of a wall is never mistaken for floor.
    A thing hanging off the floor shows its bottom too: `bottom`."""
    c = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    # Each wound to face out, as a real scan's are: the floor fit counts only
    # surfaces facing up.
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    if bottom:
        faces.append((0, 4, 5, 1))  # Wound to face down, as a real underside is.
    return [tri for a, b, d, e in faces for tri in ((c[a], c[b], c[d]), (c[a], c[d], c[e]))]


ROOM = (
    [((0, 0, 0), (3, 0, 2), (3, 0, 0)), ((0, 0, 0), (0, 0, 2), (3, 0, 2))]  # floor
    + box(0, 0, -0.1, 3, 2.4, 0)  # four walls
    + box(0, 0, 2, 3, 2.4, 2.1)
    + box(-0.1, 0, 0, 0, 2.4, 2)
    + box(3, 0, 0, 3.1, 2.4, 2)
    + box(1.0, 0, 0.8, 1.4, 0.45, 1.2)  # a stool, under the lidar's height
    + box(2.0, 0.7, 0.5, 2.6, 0.75, 1.0)  # a table top, over it
)

# Turned a quarter about the vertical and moved: the map must not care.
TURNED = [{"children": [1], "translation": [5, 0.3, -2]},
          {"mesh": 0, "rotation": [0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4)]}]
# Where the scan is, is where the map is: positions can be checked directly.
IN_PLACE = [{"mesh": 0}]


def write_gltf(path: Path, gltf: dict, data: bytes, binary: bool = True) -> Path:
    """A glTF with one buffer, as a .glb or with the buffer inline."""
    gltf = {"asset": {"version": "2.0"}, **gltf}
    if binary:
        data += b"\0" * (-len(data) % 4)
        gltf["buffers"] = [{"byteLength": len(data)}]
        text = json.dumps(gltf).encode()
        text += b" " * (-len(text) % 4)
        body = (struct.pack("<II", len(text), 0x4E4F534A) + text
                + struct.pack("<II", len(data), 0x004E4942) + data)
        path.write_bytes(b"glTF" + struct.pack("<II", 2, 12 + len(body)) + body)
    else:
        gltf["buffers"] = [{"byteLength": len(data), "uri": (
            "data:application/octet-stream;base64," + base64.b64encode(data).decode())}]
        path.write_text(json.dumps(gltf))
    return path


def write_scan(path: Path, triangles, binary: bool, nodes=TURNED) -> Path:
    data = b"".join(struct.pack("<3f", *p) for tri in triangles for p in tri)
    return write_gltf(path, {
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": nodes,
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "accessors": [{"bufferView": 0, "componentType": 5126,
                       "count": len(triangles) * 3, "type": "VEC3"}],
        "bufferViews": [{"buffer": 0, "byteLength": len(data)}],
    }, data, binary)


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


def floor(x0, z0, x1, z1, step=0.1, tilt=0.0, y=0.0):
    """A floor of small triangles, as a phone scan makes it, `y` up and
    rising `tilt` metres per metre along glTF x - a floor or a phone not
    quite level. Wound to face up, as a scanned floor is."""
    tris = []
    nx, nz = round((x1 - x0) / step), round((z1 - z0) / step)
    for i in range(nx):
        for j in range(nz):
            a, b = x0 + i * step, x0 + (i + 1) * step
            c, d = z0 + j * step, z0 + (j + 1) * step
            ya, yb = y + a * tilt, y + b * tilt
            tris += [((a, ya, c), (b, yb, d), (b, yb, c)),
                     ((a, ya, c), (a, ya, d), (b, yb, d))]
    return tris


def ball(x, z, radius, around=12, up=8):
    """A ball resting on the floor at glTF (x, z), as a mesh of quads."""
    def at(i, j):
        down, round_ = math.pi * j / up, 2 * math.pi * i / around
        return (x + radius * math.sin(down) * math.cos(round_),
                radius - radius * math.cos(down),
                z + radius * math.sin(down) * math.sin(round_))
    return [tri for i in range(around) for j in range(up)
            for tri in ((at(i, j), at(i + 1, j + 1), at(i + 1, j)),
                        (at(i, j), at(i, j + 1), at(i + 1, j + 1)))]


def walls(x0, z0, x1, z1, height=2.4):
    """Four 10 cm walls round a room, glTF x0..x1 by z0..z1."""
    return (box(x0 - 0.1, 0, z0, x0, height, z1) + box(x1, 0, z0, x1 + 0.1, height, z1)
            + box(x0, 0, z0 - 0.1, x1, height, z0) + box(x0, 0, z1, x1, height, z1 + 0.1))


def ceiling(x0, z0, x1, z1, height=2.4):
    return [tuple((x, height, z) for x, _, z in reversed(tri))  # Facing down.
            for tri in floor(x0, z0, x1, z1, step=0.5)]


def legs(x0, z0, x1, z1, top, width=0.04):
    """Four legs at the corners of x0..x1 by z0..z1, up to `top`."""
    return [tri for x in (x0, x1 - width) for z in (z0, z1 - width)
            for tri in box(x, 0, z, x + width, top, z + width)]


# A 4 x 3 m room, glTF x 0..4 and z -3..0, so map x 0..4 and y 0..3.
FURNITURE = {
    # name: (where its middle is on the map, its triangles)
    "sofa": ((1.0, 2.55), box(0.0, 0, -3, 2.0, 0.45, -2.1)),  # against the far wall
    "cabinet": ((3.75, 1.0), box(3.5, 0, -1.5, 4.0, 0.9, -0.5)),  # against the east wall
    "table": ((2.0, 1.2), box(1.5, 0.72, -1.6, 2.5, 0.75, -0.8)
              + legs(1.5, -1.6, 2.5, -0.8, 0.72)),
    "chair": ((0.8, 1.0), box(0.6, 0.43, -1.2, 1.0, 0.46, -0.8)
              + legs(0.6, -1.2, 1.0, -0.8, 0.43, width=0.02)),
    "threshold": ((2.0, 0.025), box(1.6, 0, -0.05, 2.4, 0.025, 0)),  # in the doorway
    "shoe": ((3.14, 0.55), box(3.0, 0, -0.6, 3.28, 0.10, -0.5)),
    "shoe under the table": ((2.035, 1.2), box(1.9, 0, -1.25, 2.17, 0.10, -1.15)),
}
# Too thin to count: under MIN_OBSTACLE.
RUG_AND_CABLE = box(2.8, 0, -2.0, 3.3, 0.01, -1.6) + box(0.3, 0, -0.4, 1.3, 0.012, -0.39)


class ObstacleReportTests(unittest.TestCase):
    """A patch of floor with things on it, and no walls: what a phone scan of
    a corner of a room looks like before the room is finished."""

    def report(self, scene, nodes=TURNED, **options):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "patch.glb"
            write_scan(path, scene, binary=True, nodes=nodes)
            return report(path, **options)

    def test_low_box_is_a_trip_hazard_the_lidar_misses(self):
        found = self.report(floor(0, 0, 2, 2, tilt=0.01) + box(0.8, 0.01, 0.8, 1.1, 0.11, 1.0))
        self.assertEqual(len(found["obstacles"]), 1)
        stool = found["obstacles"][0]
        # 10 cm tall, sitting on a floor that rises 1 cm per metre.
        self.assertAlmostEqual(stool["height"], 0.1, delta=0.01)
        self.assertTrue(stool["trip_hazard"])
        self.assertEqual(stool["lidar"], "misses")
        self.assertEqual(stool["kind"], "object")
        self.assertAlmostEqual(found["floor"]["tilt_deg"], math.degrees(math.atan(0.01)),
                               delta=0.1)
        # 0.3 x 0.2 m, a cell or so either way.
        self.assertAlmostEqual(stool["size"][0] * stool["size"][1], 0.06, delta=0.04)

    def test_box_above_the_lidar_is_seen(self):
        found = self.report(floor(0, 0, 2, 2) + box(0.8, 0, 0.8, 1.1, 0.4, 1.0))
        (tall,) = found["obstacles"]
        self.assertGreater(tall["height"], LIDAR_HEIGHT)
        self.assertEqual(tall["beam_range"], [0.0, 0.4])
        self.assertEqual(tall["lidar"], "sees")
        self.assertFalse(tall["trip_hazard"])

    def test_rim_where_the_scan_runs_out_is_not_an_obstacle(self):
        # The last strip of the scan curls up 10 cm, as a Polycam edge does.
        rim = [tuple((x, 0.1 if x == 2.2 else y, z) for x, y, z in tri)
               for tri in floor(2.0, 0, 2.2, 2, step=0.2)]
        found = self.report(floor(0, 0, 2, 2) + rim)
        self.assertEqual(found["obstacles"], [])
        # Not measured, and not a wall, but not thrown away either.
        self.assertEqual(found["walls"], [])
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
        self.assertTrue(hidden["outline"])

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

    def test_numbers_are_to_the_centimetre(self):
        found = self.report(floor(0, 0, 2, 2, tilt=0.013) + box(0.81, 0, 0.83, 1.17, 0.137, 1.01)
                            + ball(1.5, 1.5, 0.071))
        self.assertEqual(found["accuracy_m"], 0.03)
        self.assertEqual(found["marginal_band_m"], 0.05)
        self.assertEqual(found["min_obstacle"], 0.02)

        def check(value):
            if isinstance(value, float):
                self.assertEqual(value, round(value, 2))
            for inner in value if isinstance(value, list) else ():
                check(inner)
        for item in found["obstacles"] + found["walls"] + found["unmeasured"]:
            for key in ("center", "size", "height", "outline", "beam_range"):
                check(item.get(key, []))
        check([found["floor"]["area_m2"], found["floor"]["outline"],
               found["bounds"]["min"], found["bounds"]["max"]])

    def test_floor_plane_gives_the_floor_height_in_gltf(self):
        # Rises 2 cm per metre along glTF x, from 0.3 m at x = 0, in place.
        found = self.report(floor(0, -2, 2, 0, tilt=0.02, y=0.3), nodes=IN_PLACE)
        a, b, c = found["floor"]["plane"]
        for x, y in ((0.2, 0.3), (1.8, 1.7)):
            self.assertAlmostEqual(a * x + b * y + c, 0.3 + 0.02 * x, delta=0.002)

    def test_floor_area_is_floor_only(self):
        # 2 x 2 m, but the 1 x 0.6 m under a bed was never seen: 3.4 m2 of floor.
        hidden = [t for t in floor(0, 0, 2, 2)
                  if not (0.5 <= t[0][0] < 1.5 and 0.5 <= t[0][2] < 1.1)]
        found = self.report(hidden + box(0.5, 0, 0.5, 1.5, 0.5, 1.1))
        self.assertAlmostEqual(found["floor"]["area_m2"], 3.4, delta=0.1)
        self.assertEqual(len(found["floor"]["outline"]), 4)

    @unittest.skipUnless(REAL_SCAN.exists(), "the 10_5_2026 phone scan is not checked out")
    def test_real_scan_has_five_balls_the_lidar_cannot_see(self):
        found = report(REAL_SCAN)
        balls = [(-0.47, 0.19), (0.15, 0.40), (-0.20, -0.22), (0.29, -0.27), (0.05, -0.58)]
        self.assertEqual(len(found["obstacles"]), 5)
        for x, y in balls:
            nearest = min(found["obstacles"], key=lambda o: math.dist(o["center"], (x, y)))
            self.assertLess(math.dist(nearest["center"], (x, y)), 0.08)
            self.assertTrue(nearest["trip_hazard"])
            self.assertEqual(nearest["lidar"], "misses")
            self.assertEqual(nearest["beam_range"][0], 0.0)
            self.assertGreaterEqual(nearest["height"], 0.12)
            self.assertLessEqual(nearest["height"], 0.15)
        # The rim curling up where the scan ran out is not a wall.
        self.assertEqual(found["walls"], [])
        self.assertTrue(found["unmeasured"])


class FurnishedRoomTests(unittest.TestCase):
    """A whole room under a ceiling, with furniture against its walls."""

    @classmethod
    def setUpClass(cls):
        scene = (floor(0, -3, 4, 0) + walls(0, -3, 4, 0) + ceiling(0, -3, 4, 0)
                 + RUG_AND_CABLE + [t for _, tris in FURNITURE.values() for t in tris])
        with tempfile.TemporaryDirectory() as tmp:
            cls.found = report(write_scan(Path(tmp) / "room.glb", scene, True, IN_PLACE))

    def thing(self, name):
        where = FURNITURE[name][0]
        nearest = min(self.found["obstacles"], key=lambda o: math.dist(o["center"], where))
        self.assertLess(math.dist(nearest["center"], where), 0.1, name)
        return nearest

    def test_each_piece_of_furniture_is_its_own_obstacle(self):
        found = [self.thing(name)["id"] for name in FURNITURE]
        self.assertEqual(len(set(found)), len(FURNITURE))
        self.assertEqual(len(self.found["obstacles"]), len(FURNITURE))

    def test_walls_are_walls_and_the_ceiling_is_nothing(self):
        self.assertEqual(self.found["unmeasured"], [])
        self.assertGreater(len(self.found["walls"]), 4)
        for wall in self.found["walls"]:
            self.assertEqual(wall["height"], 2.4)
            self.assertLessEqual(max(wall["size"]), 1.1)  # In pieces, not one ring.
            self.assertTrue(wall["outline"])

    def test_heights_and_hazards(self):
        cases = {  # name: (height, trip hazard, what the lidar does at 0.195)
            "sofa": (0.45, False, "sees"),
            "cabinet": (0.9, False, "sees"),
            "table": (0.75, False, "sees"),  # Its legs.
            "threshold": (0.025, True, "misses"),
            "shoe": (0.1, True, "misses"),
            "shoe under the table": (0.1, True, "misses"),
        }
        for name, (height, trip, lidar) in cases.items():
            thing = self.thing(name)
            self.assertAlmostEqual(thing["height"], height, delta=0.011, msg=name)
            self.assertEqual(thing["trip_hazard"], trip, name)
            self.assertEqual(thing["lidar"], lidar, name)
            self.assertEqual(thing["beam_range"][0], 0.0, name)
        self.assertEqual(self.thing("table")["beam_range"], [0.0, 0.75])

    def test_floor_area_leaves_out_what_stands_on_it(self):
        # 12 m2 of room; the sofa and cabinet cover theirs, the table does not.
        self.assertGreater(self.found["floor"]["area_m2"], 10.0)
        self.assertLess(self.found["floor"]["area_m2"], 12.5)


class OverhangTests(unittest.TestCase):
    """Things whose tops are high but whose bottoms trip people up - and the
    other way round."""

    def only(self, scene):
        with tempfile.TemporaryDirectory() as tmp:
            found = report(write_scan(Path(tmp) / "s.glb", floor(0, -3, 3, 0) + scene,
                                      True, IN_PLACE))
        self.assertEqual(len(found["obstacles"]), 1, found["obstacles"])
        self.assertEqual(found["unmeasured"], [])
        return found["obstacles"][0]

    def test_office_chair_trips_on_its_base(self):
        # A 60 cm star base 8 cm high, a 5 cm column, a seat at 0.42-0.47.
        chair = self.only(box(1.2, 0, -1.53, 1.8, 0.08, -1.47)
                          + box(1.47, 0, -1.8, 1.53, 0.08, -1.2)
                          + box(1.475, 0.08, -1.525, 1.525, 0.42, -1.475)
                          + box(1.25, 0.42, -1.75, 1.75, 0.47, -1.25))
        self.assertTrue(chair["trip_hazard"])
        self.assertAlmostEqual(chair["height"], 0.47, delta=0.01)
        self.assertAlmostEqual(chair["size"][0], 0.6, delta=0.07)

    def test_floor_lamp_trips_on_its_base(self):
        lamp = self.only(box(1.3, 0, -1.7, 1.7, 0.03, -1.3)
                         + box(1.49, 0.03, -1.51, 1.51, 1.6, -1.49))
        self.assertTrue(lamp["trip_hazard"])
        self.assertEqual(lamp["beam_range"], [0.0, 1.6])
        self.assertEqual(lamp["lidar"], "sees")

    def test_plain_table_is_not_a_trip_hazard(self):
        table = self.only(box(1, 0.72, -2, 2, 0.75, -1) + legs(1, -2, 2, -1, 0.72))
        self.assertFalse(table["trip_hazard"])
        self.assertEqual(table["beam_range"], [0.0, 0.75])

    def test_wall_cabinet_is_over_the_beam(self):
        # Hung from 0.5 to 1.0 m, touching nothing.
        cabinet = self.only(box(1, 0.5, -2, 1.6, 1.0, -1.6, bottom=True))
        self.assertEqual(cabinet["beam_range"], [0.5, 1.0])
        self.assertEqual(cabinet["lidar"], "misses")
        self.assertFalse(cabinet["trip_hazard"])

    def test_ball(self):
        found = self.only(ball(1.5, -1.5, 0.07))
        self.assertEqual(found["beam_range"][0], 0.0)
        self.assertAlmostEqual(found["beam_range"][1], 0.14, delta=0.01)
        self.assertTrue(found["trip_hazard"])
        self.assertEqual(found["lidar"], "misses")

    def test_lidar_view_has_a_marginal_band(self):
        self.assertEqual(lidar_view([0.0, 0.14], 0.195), "misses")
        self.assertEqual(lidar_view([0.0, 0.15], 0.195), "marginal")
        self.assertEqual(lidar_view([0.0, 0.3], 0.195), "sees")
        self.assertEqual(lidar_view([0.0, 0.245], 0.195), "sees")
        self.assertEqual(lidar_view([0.5, 1.0], 0.195), "misses")
        self.assertEqual(lidar_view([0.2, 1.0], 0.195), "marginal")


def riser(x, z0, z1, low, high):
    """The upright face of a step along glTF x = `x`."""
    return [((x, low, z0), (x, high, z0), (x, high, z1)),
            ((x, low, z0), (x, high, z1), (x, low, z1))]


class FloorFitTests(unittest.TestCase):
    """One floor, fitted to most of the floor, shared by the map and report."""

    def scan(self, scene, name="s.glb"):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        return write_scan(Path(self.tmp.name) / name, scene, True, IN_PLACE)

    def test_two_degree_floor_is_free_with_no_phantom_wall(self):
        tilt = math.tan(math.radians(2))  # 21 cm of rise over 6 m.
        grid = render(self.scan(floor(0, -2, 6, 0, tilt=tilt)
                                + box(0.2, 0, -1.2, 0.5, 0.4, -0.9)))
        free = sum(row.count(FREE) for row in grid.cells) * grid.resolution ** 2
        self.assertGreater(free, 11.5)  # 12 m2 less the box.
        occupied = [grid.origin[0] + (c + 0.5) * grid.resolution
                    for row in grid.cells for c, v in enumerate(row) if v == OCCUPIED]
        self.assertTrue(occupied)
        self.assertLess(max(occupied), 0.6)  # The box, and nothing at the far end.

    def test_step_up_into_the_next_room(self):
        # 2.5 m of this room, then 5 cm up into 1.5 m of the next.
        scene = (floor(0, -2, 2.5, 0) + riser(2.5, -2, 0, 0, 0.05)
                 + floor(2.5, -2, 4, 0, y=0.05) + box(0.5, 0, -1.2, 0.8, 0.4, -0.9))
        found = report(self.scan(scene))
        self.assertLess(found["floor"]["tilt_deg"], 0.3)
        (step,) = [o for o in found["obstacles"] if o["kind"] == "step"]
        self.assertAlmostEqual(step["height"], 0.05, delta=0.01)
        self.assertTrue(step["trip_hazard"])
        self.assertAlmostEqual(step["center"][0], 3.25, delta=0.1)
        self.assertEqual(found["unmeasured"], [])

    def test_step_down_into_a_smaller_room_keeps_the_main_floor(self):
        scene = (floor(0, -2, 2.5, 0) + riser(2.5, -2, 0, -0.08, 0)
                 + floor(2.5, -2, 4, 0, y=-0.08))
        plane = Scan(self.scan(scene)).plane
        self.assertLess(math.degrees(math.atan(math.hypot(plane[0], plane[1]))), 0.3)
        self.assertAlmostEqual(plane[2], 0.0, delta=0.005)

    def test_ghost_below_the_floor_changes_nothing(self):
        # 1 m2 the phone read 15 cm under the floor, through a mirror.
        room = (floor(0, -3, 3, 0) + box(1.4, 0, -1.6, 1.7, 0.1, -1.4)
                + box(-0.1, 0, -3, 0, 2.4, 0))
        ghost = floor(2.0, -3, 3, -2, y=-0.15)
        clean, haunted = self.scan(room, "a.glb"), self.scan(room + ghost, "b.glb")
        self.assertEqual(render(clean).cells, render(haunted).cells)
        without, with_ghost = report(clean), report(haunted)
        without.pop("source"), with_ghost.pop("source")
        self.assertEqual(without, with_ghost)

    def test_floor_slab_is_measured_from_its_top(self):
        # Polycam's room-plan export: a floor 10 cm thick, walls to its foot.
        scene = (box(0, -0.1, -2, 3, 0, 0, bottom=True)
                 + box(-0.1, -0.1, -2, 0, 2.4, 0) + box(3, -0.1, -2, 3.1, 2.4, 0))
        found = report(self.scan(scene))
        self.assertAlmostEqual(found["floor"]["gltf_floor_y"], 0.0, delta=0.005)
        self.assertEqual(found["obstacles"], [])


class GltfTests(unittest.TestCase):
    """What other exporters write, read the same - or refused by name."""

    SCENE = floor(0, -1, 1, 0, step=0.5) + box(0.3, 0, -0.7, 0.6, 0.3, -0.4)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.positions = b"".join(struct.pack("<3f", *p) for t in self.SCENE for p in t)
        self.expected = sorted(triangles(write_scan(self.path(), self.SCENE, True, IN_PLACE)))

    def path(self):
        return Path(self.tmp.name) / "s.glb"

    def write(self, primitive, accessors, data=None, **extra):
        data = self.positions if data is None else data
        return write_gltf(self.path(), {
            "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [primitive]}], "accessors": accessors,
            "bufferViews": [{"buffer": 0, "byteLength": len(data)}], **extra}, data)

    def positions_accessor(self, **change):
        return {"bufferView": 0, "componentType": 5126,
                "count": len(self.SCENE) * 3, "type": "VEC3", **change}

    def test_strip_and_fan_are_triangles(self):
        square = [(0, 0, 0), (1, 0, 0), (0, 0, -1), (1, 0, -1)]
        data = b"".join(struct.pack("<3f", *p) for p in square)
        accessor = [{"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3"}]
        strip = triangles(self.write({"attributes": {"POSITION": 0}, "mode": 5},
                                     accessor, data))
        self.assertEqual(len(strip), 2)
        fan_order = [square[0], square[1], square[3], square[2]]
        data = b"".join(struct.pack("<3f", *p) for p in fan_order)
        fan = triangles(self.write({"attributes": {"POSITION": 0}, "mode": 6},
                                   accessor, data))
        self.assertEqual(len(fan), 2)
        area = sum(abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
                   for a, b, c in strip + fan)
        self.assertAlmostEqual(area, 2.0)  # Both cover the whole square.

    def test_points_are_refused(self):
        with self.assertRaisesRegex(ValueError, "points"):
            triangles(self.write({"attributes": {"POSITION": 0}, "mode": 0},
                                 [self.positions_accessor()]))

    def test_no_scenes_reads_every_root_node(self):
        path = write_gltf(self.path(), {
            "nodes": [{"mesh": 0}], "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
            "accessors": [self.positions_accessor()],
            "bufferViews": [{"buffer": 0, "byteLength": len(self.positions)}]},
            self.positions)
        self.assertEqual(sorted(triangles(path)), self.expected)

    def test_compressed_mesh_is_refused_by_name(self):
        with self.assertRaisesRegex(ValueError, "Draco"):
            triangles(self.write({"attributes": {"POSITION": 0}},
                                 [{"componentType": 5126, "count": 3, "type": "VEC3"}]))
        with self.assertRaisesRegex(ValueError, "KHR_draco_mesh_compression"):
            triangles(self.write({"attributes": {"POSITION": 0}}, [self.positions_accessor()],
                                 extensionsUsed=["KHR_draco_mesh_compression"]))
        with self.assertRaisesRegex(ValueError, "sparse"):
            triangles(self.write({"attributes": {"POSITION": 0}},
                                 [self.positions_accessor(sparse={"count": 1})]))

    def test_quantized_positions_are_refused_by_name(self):
        data = struct.pack("<9h", *range(9))
        with self.assertRaisesRegex(ValueError, "int16"):
            triangles(self.write({"attributes": {"POSITION": 0}},
                                 [{"bufferView": 0, "componentType": 5122, "count": 3,
                                   "type": "VEC3"}], data))
        with self.assertRaisesRegex(ValueError, "normalized"):
            triangles(self.write({"attributes": {"POSITION": 0}},
                                 [{"bufferView": 0, "componentType": 5122, "count": 3,
                                   "type": "VEC3", "normalized": True}], data))

    def test_materials_and_textures_are_no_bar(self):
        path = self.write({"attributes": {"POSITION": 0}}, [self.positions_accessor()],
                          extensionsUsed=["KHR_materials_unlit"])
        self.assertEqual(sorted(triangles(path)), self.expected)


class CommandLineTests(unittest.TestCase):
    def run_main(self, scene, *options):
        with tempfile.TemporaryDirectory() as tmp:
            scan = write_scan(Path(tmp) / "s.glb", scene, True, IN_PLACE)
            stem = Path(tmp) / "map"
            out = io.StringIO()
            reads = mock.Mock(wraps=scan_to_map.triangles)
            argv = ["scan_to_map", str(scan), "-f", str(stem), "--report",
                    str(Path(tmp) / "room.json"), *options]
            with (mock.patch.object(sys, "argv", argv),
                  mock.patch.object(scan_to_map, "triangles", reads),
                  contextlib.redirect_stdout(out)):
                scan_to_map.main()
            return (reads.call_count, stem.with_suffix(".pgm").exists(),
                    json.loads((Path(tmp) / "room.json").read_text()), out.getvalue())

    def test_map_and_report_read_the_scan_once(self):
        reads, wrote_map, found, _ = self.run_main(
            floor(0, -2, 2, 0) + box(0.5, 0, -1.5, 0.8, 0.5, -1.2))
        self.assertEqual(reads, 1)
        self.assertTrue(wrote_map)
        self.assertEqual(len(found["obstacles"]), 1)

    def test_nothing_at_lidar_height_writes_no_map(self):
        reads, wrote_map, found, said = self.run_main(
            floor(0, -2, 2, 0) + box(0.5, 0, -1.5, 0.8, 0.1, -1.2))
        self.assertFalse(wrote_map)
        self.assertIn("no map written", said)
        self.assertEqual(found["obstacles"][0]["lidar"], "misses")


if __name__ == "__main__":
    unittest.main()
