"""Turn a phone scan of a room (Polycam glTF export) into an occupancy map.

THIS IS NOT A SLAM RESULT either. A phone carried or driven round the room
measures it well, but the robot's own lidar did not see it: nothing here says
the robot can localize against this map, and the lidar height it is sliced
at is a guess until the real lidar is mounted and measured.

What it does, in the same shape as world_to_map:

* the scan's floor is found once, as the plane most of the floor lies on
  (see floor_plane), and every height below is measured from it - so a
  floor, or a phone, not quite level does not read as a slope of obstacles;
* the mesh is cut by a surface `--height` above that floor and parallel to
  it - what a lidar mounted there would strike - and every cut is occupied;
* every cell with scanned floor under it and no cut in it is free;
* everything else is unknown, including floor the phone never saw.

glTF is y-up and in metres. The map is z-up, so a glTF point (x, y, z) lands
at map (x, -z) at height y. The map's origin is the export's own origin, not
where the scan started or where the floor is: Polycam centres the export on
the scan's bounding box (10_5_2026.glb runs from -1.20 to +1.20 m in x and
-0.09 to +0.09 m in height), so map (0, 0) is the middle of what was scanned.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
import random
import struct

from robot_bringup.world_to_map import FREE, OCCUPIED, UNKNOWN, OccupancyGrid

# A triangle whose corners all lie this close to the floor is floor. Below
# the floor by more than this is not a thing at all but a reflection the
# phone read as a surface behind a mirror or a window: it is ignored.
FLOOR_BAND = 0.05

# The floor is fitted to the lowest surface in cells this wide...
FLOOR_CELL = 0.1
# ...as the plane that most of those lowest surfaces lie close to, a cell
# counting less the further off it is and not at all past FLOOR_SCORE_BAND.
# Counted all-or-nothing, a plane tilted to cut both levels of a step
# collects a wide strip of each and can beat the level floor of either.
FLOOR_SCORE_BAND = 0.01
# The winner is then refitted to the cells this close to it.
FLOOR_FIT_BAND = 0.02
# A phone held off level tilts the whole scan; more than this is not a floor.
MAX_FLOOR_TILT = math.radians(10)
# Planes tried. Seeded, so the same scan always gives the same floor.
FLOOR_TRIES = 200
# Fewer FLOOR_CELLs of floor than this - a tenth of a square metre - is too
# little to say which way the floor runs.
MIN_FLOOR_CELLS = 10

# The obstacle report's thresholds, all in metres above the floor.
# The lidar's height - a guess until the real lidar is mounted and measured.
LIDAR_HEIGHT = 0.195
# Lower than this is the floor's own texture and the scan's noise.
MIN_OBSTACLE = 0.02
# Up to this, a thing is low enough to trip over rather than walk round.
TRIP_HAZARD_MAX = 0.30
# iPhone LiDAR is good to about 1-3 cm close up and worse across a room, so
# the report says no more than this about any size or height it gives.
ACCURACY = 0.03
# A beam this close to an object's top or bottom may hit it or skim past:
# the lidar's own mounting, the floor's fit and the scan are all that rough.
MARGINAL_BAND = 0.05

# Two surfaces over one cell closer together than this are one surface.
RUN_GAP = 0.02
# Surfaces this close, one over the other, are one thing: the phone rarely
# sees the underside of a seat, so the seat's top seems to float a few
# centimetres over the column it stands on. A shoe under a table is far
# further below the top than this, and stays a thing of its own.
JOIN_GAP = 0.10
# A surface rising from WALL_FOOT or lower past WALL_HEIGHT, over a stretch
# at least WALL_MIN_LENGTH long, is structure - a wall, or something as good
# as one - and not furniture. The foot is not the floor itself, because a
# sofa pushed against a wall hides the bottom of it from the phone; it is
# low enough that a cabinet hung on the wall is still a cabinet.
WALL_HEIGHT = 1.0
WALL_FOOT = 0.3
WALL_MIN_LENGTH = 0.5
# Walls are listed in pieces at most this long, so each piece's convex
# outline follows its wall instead of spanning the room the walls enclose.
WALL_PIECE = 1.0
# Wholly above this is ceiling or a ceiling light: overhead of a floor robot
# and of anyone's feet. Left out, or a ceiling would cover every cell.
OVERHEAD = 1.8
# A raised patch at least this big whose top is level to within ACCURACY
# is a step to another floor level, not a thing standing on this one.
STEP_MIN_AREA = 0.25

_COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
_FORMATS = {5121: "B", 5123: "H", 5125: "I", 5126: "f"}
_COMPONENT_NAMES = {5120: "int8", 5121: "uint8", 5122: "int16", 5123: "uint16",
                    5125: "uint32", 5126: "float32"}
_MODES = {0: "points", 1: "lines", 2: "line loop", 3: "line strip",
          4: "triangles", 5: "triangle strip", 6: "triangle fan"}
# Extensions that change where the vertices are kept or what they mean. The
# rest (materials, textures) do not touch the geometry and are ignored.
_GEOMETRY_EXTENSIONS = {"KHR_draco_mesh_compression", "EXT_meshopt_compression",
                        "KHR_mesh_quantization"}
_IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def _load(path: Path) -> tuple[dict, list[bytes]]:
    """The glTF JSON and its binary buffers, from a .glb or a .gltf."""
    data = path.read_bytes()
    if data[:4] == b"glTF":
        chunks, offset = [], 12
        while offset < len(data):
            length, _kind = struct.unpack_from("<II", data, offset)
            chunks.append(data[offset + 8:offset + 8 + length])
            offset += 8 + length
        return json.loads(chunks[0]), chunks[1:]
    gltf = json.loads(data)
    buffers = []
    for buffer in gltf.get("buffers", []):
        uri = buffer["uri"]
        if uri.startswith("data:"):
            buffers.append(base64.b64decode(uri.split(",", 1)[1]))
        else:
            buffers.append((path.parent / uri).read_bytes())
    return gltf, buffers


def _accessor(gltf: dict, buffers: list[bytes], index: int,
              allowed: tuple[int, ...]) -> list[tuple]:
    """An accessor's values, or a ValueError naming what this cannot read.

    Compressed and quantized exports keep their numbers somewhere this does
    not look, or in a form it does not decode. Reading them anyway would
    lose geometry or misplace it without a word, so they are refused.
    """
    accessor = gltf["accessors"][index]
    if "sparse" in accessor:
        raise ValueError(f"accessor {index} is sparse, which is not supported: "
                         "export without sparse accessors")
    if "bufferView" not in accessor:
        raise ValueError(f"accessor {index} has no bufferView - the mesh is "
                         "compressed (Draco or meshopt), which is not supported: "
                         "export uncompressed")
    kind = accessor["componentType"]
    if kind not in allowed or accessor.get("normalized"):
        name = _COMPONENT_NAMES.get(kind, str(kind))
        if accessor.get("normalized"):
            name = "normalized " + name
        raise ValueError(f"accessor {index} holds {name} values ({kind}), which are not "
                         "supported here - the export is quantized: export without "
                         "mesh quantization")
    view = gltf["bufferViews"][accessor["bufferView"]]
    fmt = "<" + _FORMATS[kind] * _COMPONENTS[accessor["type"]]
    stride = view.get("byteStride") or struct.calcsize(fmt)
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    buffer = buffers[view.get("buffer", 0)]
    size, count = struct.calcsize(fmt), accessor["count"]
    if stride == size:  # Packed tight, as Polycam writes it: read in one go.
        return list(struct.iter_unpack(fmt, buffer[start:start + count * size]))
    return [struct.unpack_from(fmt, buffer, start + i * stride) for i in range(count)]


def _matmul(a: list[float], b: list[float]) -> list[float]:
    """Product of two column-major 4x4 matrices, as glTF stores them."""
    return [
        sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4))
        for col in range(4)
        for row in range(4)
    ]


def _local(node: dict) -> list[float]:
    if "matrix" in node:
        return list(node["matrix"])
    tx, ty, tz = node.get("translation", (0, 0, 0))
    qx, qy, qz, qw = node.get("rotation", (0, 0, 0, 1))
    sx, sy, sz = node.get("scale", (1, 1, 1))
    return [
        (1 - 2 * (qy * qy + qz * qz)) * sx, 2 * (qx * qy + qz * qw) * sx,
        2 * (qx * qz - qy * qw) * sx, 0,
        2 * (qx * qy - qz * qw) * sy, (1 - 2 * (qx * qx + qz * qz)) * sy,
        2 * (qy * qz + qx * qw) * sy, 0,
        2 * (qx * qz + qy * qw) * sz, 2 * (qy * qz - qx * qw) * sz,
        (1 - 2 * (qx * qx + qy * qy)) * sz, 0,
        tx, ty, tz, 1,
    ]


def _corners(mode: int, order: list[int]) -> list[tuple[int, int, int]]:
    """Index triples for each triangle, whichever way the primitive lists them.

    Strips and fans are only a shorter way of writing triangles. Points and
    lines are not a surface at all: a point-cloud export is refused rather
    than turned into an empty map.
    """
    if mode == 4:
        return [tuple(order[i:i + 3]) for i in range(0, len(order) - 2, 3)]
    if mode == 5:  # Every other triangle turned over, to keep one winding.
        return [(order[i], order[i + 1], order[i + 2]) if i % 2 == 0
                else (order[i + 1], order[i], order[i + 2])
                for i in range(len(order) - 2)]
    if mode == 6:
        return [(order[0], order[i], order[i + 1]) for i in range(1, len(order) - 1)]
    raise ValueError(f"primitive mode {mode} ({_MODES.get(mode, 'unknown')}) is not a "
                     "surface: export a mesh, not a point cloud or lines")


def triangles(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    """Every triangle in the scene, in z-up metres: (map x, map y, height)."""
    gltf, buffers = _load(path)
    unread = sorted(_GEOMETRY_EXTENSIONS & set(gltf.get("extensionsRequired", [])
                                               + gltf.get("extensionsUsed", [])))
    if unread:
        raise ValueError(f"{path} uses the glTF extension {', '.join(unread)}, which is "
                         "not supported: export without compression or quantization")
    if gltf.get("scenes"):
        roots = gltf["scenes"][gltf.get("scene", 0)]["nodes"]
    else:
        # No scene says what to show, so show every node nothing else holds.
        children = {c for node in gltf.get("nodes", []) for c in node.get("children", [])}
        roots = [i for i in range(len(gltf.get("nodes", []))) if i not in children]
    found = []
    pending = [(index, _IDENTITY) for index in roots]
    while pending:
        index, parent = pending.pop()
        node = gltf["nodes"][index]
        world = _matmul(parent, _local(node))
        pending.extend((child, world) for child in node.get("children", []))
        if "mesh" not in node:
            continue
        for primitive in gltf["meshes"][node["mesh"]]["primitives"]:
            mode = primitive.get("mode", 4)
            m0, m1, m2, _, m4, m5, m6, _, m8, m9, m10, _, m12, m13, m14, _ = world
            points = [
                (m0 * x + m4 * y + m8 * z + m12,
                 -(m2 * x + m6 * y + m10 * z + m14),
                 m1 * x + m5 * y + m9 * z + m13)
                for x, y, z in _accessor(gltf, buffers,
                                         primitive["attributes"]["POSITION"], (5126,))
            ]
            if "indices" in primitive:
                order = [i[0] for i in _accessor(gltf, buffers, primitive["indices"],
                                                 (5121, 5123, 5125))]
            else:
                order = list(range(len(points)))
            found.extend((points[a], points[b], points[c])
                         for a, b, c in _corners(mode, order))
    if not found:
        raise ValueError(f"no triangles in {path}")
    return found


class NothingAtHeight(ValueError):
    """Nothing in the scan reaches the slice: the lidar would see no obstacle."""


def _plane_through(p, q, s):
    """z = a x + b y + c through three points, or None if they are in a line."""
    ux, uy, uz = q[0] - p[0], q[1] - p[1], q[2] - p[2]
    vx, vy, vz = s[0] - p[0], s[1] - p[1], s[2] - p[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    if abs(nz) < 1e-9:
        return None
    a, b = -nx / nz, -ny / nz
    return a, b, p[2] - a * p[0] - b * p[1]


def floor_plane(tris) -> tuple[float, float, float]:
    """The floor as z = a x + b y + c: the plane most of the floor lies on.

    Each FLOOR_CELL keeps the lowest surface over it, which is floor wherever
    the phone saw floor - under a table as well as in the open - and never
    the ceiling. Planes through three of those, chosen at random, are tried
    and the one the most cells lie on, and lie closest to, wins; it is then
    refitted to just those cells, and last to the scan's corners near it.
    The most, not the lowest and not the average: a step into the next
    room, a sunken doorway, or a patch the phone read behind a mirror is a
    minority of cells and drops out, where a least-squares fit to all of
    them would lean towards it and tilt the whole floor.

    Only surfaces facing up count: a floor is seen from above. Polycam's
    room-plan export draws the floor as a slab 10 cm thick, and its
    underside is the lowest surface everywhere - fitted to, every wall foot
    and the slab's own edge would read as a 10 cm obstacle.

    render() and report() both measure from this one plane, so the map and
    the report never disagree about where the floor is.
    """
    upward = [t for t in tris if (t[1][0] - t[0][0]) * (t[2][1] - t[0][1])
              > (t[1][1] - t[0][1]) * (t[2][0] - t[0][0])]
    tris = upward or tris  # A mesh with no winding to go by: take it all.
    lowest = {}

    def mark(x, y, z):
        key = (x // FLOOR_CELL, y // FLOOR_CELL)
        if key not in lowest or z < lowest[key][2]:
            lowest[key] = (x, y, z)

    corners = {p for tri in tris for p in tri}  # A mesh shares each about six ways.
    for point in corners:
        key = (point[0] // FLOOR_CELL, point[1] // FLOOR_CELL)
        held = lowest.get(key)
        if held is None or point[2] < held[2]:
            lowest[key] = point
    for (ax, ay, az), (bx, by, bz), (cx, cy, cz) in tris:
        if (abs(bx - ax) + abs(cx - ax) <= FLOOR_CELL
                and abs(by - ay) + abs(cy - ay) <= FLOOR_CELL):
            continue  # Small: its corners have said it all.
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if area == 0:
            continue  # Edge-on: so have they.
        x0 = math.floor(min(ax, bx, cx) / FLOOR_CELL)
        y0 = math.floor(min(ay, by, cy) / FLOOR_CELL)
        for row in range(y0, math.floor(max(ay, by, cy) / FLOOR_CELL) + 1):
            for column in range(x0, math.floor(max(ax, bx, cx) / FLOOR_CELL) + 1):
                px, py = (column + 0.5) * FLOOR_CELL, (row + 0.5) * FLOOR_CELL
                u = ((bx - px) * (cy - py) - (cx - px) * (by - py)) / area
                v = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) / area
                if u >= 0 and v >= 0 and u + v <= 1:
                    mark(px, py, u * az + v * bz + (1 - u - v) * cz)

    points = sorted(lowest.values())
    if len(points) < 3:
        raise ValueError("too little floor in the scan to fit a plane")
    sample = points[::max(1, len(points) // 1000)]
    chance = random.Random(0)
    best, most = None, 0
    for _ in range(FLOOR_TRIES):
        plane = _plane_through(*chance.sample(points, 3))
        if plane is None or math.hypot(plane[0], plane[1]) > math.tan(MAX_FLOOR_TILT):
            continue
        a, b, c = plane
        on = 0.0
        for x, y, z in sample:
            off = (z - a * x - b * y - c) / FLOOR_SCORE_BAND
            if -1 < off < 1:
                on += 1 - off * off
        if on > most:
            best, most = plane, on
    if best is None:
        raise ValueError("too little floor in the scan to fit a plane")
    a, b, c = best
    for _ in range(3):
        on = [(x, y, z) for x, y, z in points
              if abs(z - a * x - b * y - c) <= FLOOR_FIT_BAND]
        if len(on) < MIN_FLOOR_CELLS:
            raise ValueError("too little floor in the scan to fit a plane")
        a, b, c = _fit_plane(on)
    # The lowest surface in each cell sits under the floor's middle by about
    # the scan's noise, so the plane found from them is set by the floor's
    # own corners: every one near it, not just the lowest.
    corners = list(corners)[::max(1, len(corners) // 20000)]
    for _ in range(2):
        near = [(x, y, z) for x, y, z in corners
                if abs(z - a * x - b * y - c) <= FLOOR_FIT_BAND]
        if len(near) < MIN_FLOOR_CELLS:
            break  # A floor of a few big triangles: the cells have it already.
        a, b, c = _fit_plane(near)
    return a, b, c


class Scan:
    """A scan read once: its triangles, and its floor fitted once.

    The command line can write the map and the report from one scan without
    reading and fitting it twice, and the two cannot use different floors.
    """

    def __init__(self, path: Path):
        self.name = path.name
        self.triangles = triangles(path)
        self.plane = floor_plane(self.triangles)
        a, b, c = self.plane
        # Every corner's height measured up from the floor beneath it, not
        # from a level: the floor itself is then at 0 everywhere, tilted or
        # not, and a cut at `height` runs parallel to it.
        self.above_floor = [tuple((x, y, z - a * x - b * y - c) for x, y, z in tri)
                            for tri in self.triangles]


def _scan(scan) -> Scan:
    return scan if isinstance(scan, Scan) else Scan(Path(scan))


def _cut(triangle, height: float):
    """The segment where a triangle crosses the plane at `height`, if any."""
    ends = []
    for (ax, ay, az), (bx, by, bz) in (
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ):
        if (az - height) * (bz - height) < 0 or (az == height != bz):
            t = (height - az) / (bz - az)
            ends.append((ax + t * (bx - ax), ay + t * (by - ay)))
    return ends[:2] if len(ends) >= 2 else None


def render(scan, resolution: float = 0.05, height: float = LIDAR_HEIGHT,
           margin: float = 0.3) -> OccupancyGrid:
    """Turn a glTF scan into an occupancy grid sliced `height` above the floor.

    `scan` is a path or a Scan. The slice and the floor both follow the
    fitted floor plane, so on a tilted floor the slice stays `height` above
    it everywhere: a level slice would run into the high side of the floor
    and draw a wall across the room, and a level floor band would leave the
    far side unknown.
    """
    scan = _scan(scan)
    tris = scan.above_floor
    walls = [cut for tri in tris if (cut := _cut(tri, height))]
    floors = [tri for tri in tris if -FLOOR_BAND <= tri[0][2] <= FLOOR_BAND
              and -FLOOR_BAND <= tri[1][2] <= FLOOR_BAND
              and -FLOOR_BAND <= tri[2][2] <= FLOOR_BAND]
    if not walls:
        raise NothingAtHeight(
            f"nothing in {scan.name} crosses {height} m above the floor")
    if not floors:
        raise ValueError(f"no floor found in {scan.name}")

    xs = [x for seg in walls for x, _ in seg] + [p[0] for t in floors for p in t]
    ys = [y for seg in walls for _, y in seg] + [p[1] for t in floors for p in t]
    origin = (min(xs) - margin, min(ys) - margin)
    width = int(math.ceil((max(xs) + margin - origin[0]) / resolution))
    rows = int(math.ceil((max(ys) + margin - origin[1]) / resolution))
    cells = [[UNKNOWN] * width for _ in range(rows)]

    def cell(x: float, y: float) -> tuple[int, int]:
        return int((x - origin[0]) / resolution), int((y - origin[1]) / resolution)

    for (ax, ay, _), (bx, by, _), (cx, cy, _) in floors:
        corners = (cell(ax, ay), cell(bx, by), cell(cx, cy))
        for column, row in corners:
            cells[row][column] = FREE  # Covers a triangle smaller than a cell.
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if area == 0 or corners[0] == corners[1] == corners[2]:
            continue
        left, right = min(c[0] for c in corners), max(c[0] for c in corners)
        bottom, top = min(c[1] for c in corners), max(c[1] for c in corners)
        for row in range(bottom, top + 1):
            for column in range(left, right + 1):
                px = origin[0] + (column + 0.5) * resolution
                py = origin[1] + (row + 0.5) * resolution
                u = ((bx - px) * (cy - py) - (cx - px) * (by - py)) / area
                v = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) / area
                if u >= 0 and v >= 0 and u + v <= 1:
                    cells[row][column] = FREE

    # Obstacles last, so they win over the floor beneath them.
    for (ax, ay), (bx, by) in walls:
        steps = max(1, int(math.hypot(bx - ax, by - ay) / (resolution / 2)))
        for i in range(steps + 1):
            column, row = cell(ax + (bx - ax) * i / steps, ay + (by - ay) * i / steps)
            cells[row][column] = OCCUPIED

    return OccupancyGrid(cells, resolution, origin)


def _solve3(m: list[list[float]], v: list[float]) -> list[float]:
    """Cramer's rule for a 3x3 system: all the plane fit needs."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    if abs(d) < 1e-12:  # All the points on one line: no single plane.
        raise ValueError("too little floor in the scan to fit a plane")
    return [det([[v[r] if c == k else m[r][c] for c in range(3)] for r in range(3)]) / d
            for k in range(3)]


def _fit_plane(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """Least-squares z = a x + b y + c through the points."""
    if len(points) < 3:
        raise ValueError("too little floor in the scan to fit a plane")
    n = len(points)
    # About their middle, or a scan far from its origin loses the tilt to
    # rounding in the sums.
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sx = sy = sz = sxx = sxy = syy = sxz = syz = 0.0
    for x, y, z in points:
        x, y = x - mx, y - my
        sx, sy, sz = sx + x, sy + y, sz + z
        sxx, sxy, syy = sxx + x * x, sxy + x * y, syy + y * y
        sxz, syz = sxz + x * z, syz + y * z
    a, b, c = _solve3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]], [sxz, syz, sz])
    return a, b, c - a * mx - b * my


def _hull(points) -> list[list[float]]:
    """Convex hull, anticlockwise, rounded to the centimetre."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return [[_cm(x), _cm(y)] for x, y in pts]

    def chain(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                (ax, ay), (bx, by) = out[-2], out[-1]
                if (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax) > 0:
                    break
                out.pop()
            out.append(p)
        return out[:-1]
    return [[_cm(x), _cm(y)] for x, y in chain(pts) + chain(pts[::-1])]


def _cm(value: float) -> float:
    """To the whole centimetre: the scan is not good to better (ACCURACY)."""
    return round(value, 2) + 0.0  # + 0.0 turns -0.0 into 0.0.


def _clip(poly, axis: int, lo: float, hi: float):
    """The part of a polygon between lo and hi along one axis."""
    for bound, sign in ((lo, 1), (hi, -1)):
        src, poly = poly, []
        last = src[-1]
        d_last = (last[axis] - bound) * sign
        for p in src:
            d = (p[axis] - bound) * sign
            if (d >= 0) != (d_last >= 0):
                t = d_last / (d_last - d)
                poly.append(tuple(q + t * (r - q) for q, r in zip(last, p)))  # noqa: B905 - no strict=, the demo Mac runs Python 3.9
            if d >= 0:
                poly.append(p)
            last, d_last = p, d
        if not poly:
            return poly
    return poly


def lidar_view(beam_range, height: float) -> str:
    """Whether a level beam at `height` would strike something spanning beam_range.

    "marginal" within MARGINAL_BAND of either end: the lidar might catch the
    top of a thing or skim over it, and the report will not pretend to know.
    """
    lo, hi = beam_range
    if lo + MARGINAL_BAND <= height <= hi - MARGINAL_BAND:
        return "sees"
    if height < lo - MARGINAL_BAND or height > hi + MARGINAL_BAND:
        return "misses"
    return "marginal"


def report(scan, resolution: float = 0.03,
           lidar_height: float = LIDAR_HEIGHT) -> dict:
    """What stands on the scanned floor, and whether the lidar would see it.

    The lidar slice above only shows what crosses one plane. An older person
    trips over what is under it - a shoe, a cable, a ball - so this looks at
    everything that rises off the floor, however low. `scan` is a path or a
    Scan. Heights are above floor_plane.

    * Each cell keeps every height at which some surface stands over it, as
      runs: a table's leg is one run from the floor to the top; the top over
      a shoe is two - the shoe low down and the table top high up.
    * Runs entirely above OVERHEAD are ceiling, and dropped.
    * Cells with a run rising from WALL_FOOT past WALL_HEIGHT, over a stretch
      at least WALL_MIN_LENGTH long, are structure and go in `walls`. They
      are taken out first, so a sofa or a cabinet pushed against a wall is
      still a thing of its own.
    * Runs that touch, cell to cell and height to height (to within
      JOIN_GAP), are one object. So the shoe under the table is not the
      table: nothing joins them.
    * An object is a trip hazard if any part of it is low: a cell where its
      lowest run starts at the floor and stops below TRIP_HAZARD_MAX. That
      is the base of an office chair and of a floor lamp, though their tops
      are high, and not a table's legs, which run on up to the top.
    * `beam_range` is the heights at which the object stands at all: where a
      level beam could cut it. `lidar` compares that with lidar_height.
    * An object touching unknown cells or the scan's edge goes in
      `unmeasured`, not `obstacles`. Where a phone scan runs out the mesh
      curls up into a ragged rim, which is an artifact and not a thing - but
      so does a real object half out of the scan, or one hiding a shadow the
      phone never saw behind it. Its height and extent cannot be trusted,
      and it is never dropped silently: a report of no trip hazards must not
      mean they were all thrown away. The exception is a step: a big, level
      raised floor is a step whether or not the scan saw all of it.
    """
    scan = _scan(scan)
    tris = scan.above_floor
    xs = [p[0] for tri in tris for p in tri]
    ys = [p[1] for tri in tris for p in tri]
    origin = (min(xs), min(ys))
    columns = int((max(xs) - origin[0]) / resolution) + 1
    rows = int((max(ys) - origin[1]) / resolution) + 1
    known = bytearray(columns * rows)  # A surface over the cell, at any height.
    floor = bytearray(columns * rows)  # The floor itself over the cell.
    spans: dict[int, list[tuple[float, float]]] = {}

    def where(x, y):
        return (min(columns - 1, max(0, int((x - origin[0]) / resolution))),
                min(rows - 1, max(0, int((y - origin[1]) / resolution))))

    ox, oy = origin
    for tri in tris:
        (ax, ay, ah), (bx, by, bh), (cx, cy, ch) = tri
        top = ah if ah > bh else bh
        top = ch if ch > top else top
        if top < -FLOOR_BAND:
            continue  # Seen in a mirror: there is nothing there.
        ca, cb, cc = (int((ax - ox) / resolution), int((bx - ox) / resolution),
                      int((cx - ox) / resolution))
        ra, rb, rc = (int((ay - oy) / resolution), int((by - oy) / resolution),
                      int((cy - oy) / resolution))
        if ca == cb == cc and ra == rb == rc:
            # Inside one cell, as most of a fine scan's triangles are.
            i = ra * columns + ca
            known[i] = 1
            if (-FLOOR_BAND <= ah <= FLOOR_BAND and -FLOOR_BAND <= bh <= FLOOR_BAND
                    and -FLOOR_BAND <= ch <= FLOOR_BAND):
                floor[i] = 1
            if top > MIN_OBSTACLE:
                spans.setdefault(i, []).append((max(min(ah, bh, ch), MIN_OBSTACLE), top))
            continue
        flat = (-FLOOR_BAND <= ah <= FLOOR_BAND and -FLOOR_BAND <= bh <= FLOOR_BAND
                and -FLOOR_BAND <= ch <= FLOOR_BAND)
        left, right = min(ca, cb, cc), max(ca, cb, cc)
        low_row, high_row = min(ra, rb, rc), max(ra, rb, rc)
        if top > MIN_OBSTACLE:
            # Exactly which heights it covers over each cell, edge-on walls
            # included: cut it into columns, then the columns into cells.
            for column in range(left, right + 1):
                x0 = origin[0] + column * resolution
                strip = _clip(tri, 0, x0, x0 + resolution)
                if not strip:
                    continue
                first = where(0, min(p[1] for p in strip))[1]
                last = where(0, max(p[1] for p in strip))[1]
                for row in range(first, last + 1):
                    y0 = origin[1] + row * resolution
                    piece = _clip(strip, 1, y0, y0 + resolution)
                    if not piece:
                        continue
                    i = row * columns + column
                    known[i] = 1
                    floor[i] |= flat
                    high = max(p[2] for p in piece)
                    if high > MIN_OBSTACLE:
                        spans.setdefault(i, []).append(
                            (max(min(p[2] for p in piece), MIN_OBSTACLE), high))
            continue
        # Floor, or near it: which cells it covers is all that matters.
        for i in (ra * columns + ca, rb * columns + cb, rc * columns + cc):
            known[i] = 1
            floor[i] |= flat
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if area == 0:
            continue
        for row in range(low_row, high_row + 1):
            for column in range(left, right + 1):
                px = origin[0] + (column + 0.5) * resolution
                py = origin[1] + (row + 0.5) * resolution
                u = ((bx - px) * (cy - py) - (cx - px) * (by - py)) / area
                v = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) / area
                if u >= 0 and v >= 0 and u + v <= 1:
                    known[row * columns + column] = 1
                    floor[row * columns + column] |= flat

    # Each cell's heights as runs, lowest first, ceiling dropped.
    runs: dict[tuple[int, int], list[tuple[float, float]]] = {}
    for i, found in spans.items():
        merged = []
        for lo, hi in sorted(found):
            if merged and lo <= merged[-1][1] + RUN_GAP:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        merged = [run for run in merged if run[0] < OVERHEAD]
        if merged:
            runs[(i % columns, i // columns)] = merged

    def neighbours(c, r):
        return [(c + dc, r + dr) for dc in (-1, 0, 1) for dr in (-1, 0, 1) if dc or dr]

    def groups(cells):
        """Cells that touch, diagonals too: a corner can land kitty-corner."""
        cells, found = set(cells), []
        while cells:
            group, todo = [], [cells.pop()]
            while todo:
                cell = todo.pop()
                group.append(cell)
                for n in neighbours(*cell):
                    if n in cells:
                        cells.remove(n)
                        todo.append(n)
            found.append(group)
        return found

    def corners(cells):
        return [(origin[0] + (c + dc) * resolution, origin[1] + (r + dr) * resolution)
                for c, r in cells for dc in (0, 1) for dr in (0, 1)]

    def outline(cells):
        """The convex hull of the cells: only each row's end cells can be on it."""
        ends = {}
        for c, r in cells:
            low, high = ends.get(r, (c, c))
            ends[r] = (min(low, c), max(high, c))
        return _hull(corners([(c, r) for r, pair in ends.items() for c in pair]))

    def extent(cells):
        points = corners(cells)
        x0, x1 = min(p[0] for p in points), max(p[0] for p in points)
        y0, y1 = min(p[1] for p in points), max(p[1] for p in points)
        return {"center": [_cm((x0 + x1) / 2), _cm((y0 + y1) / 2)],
                "size": [_cm(x1 - x0), _cm(y1 - y0)]}

    # Structure first.
    tall = [cell for cell, found in runs.items()
            if any(hi >= WALL_HEIGHT and lo <= WALL_FOOT for lo, hi in found)]
    structure, walls = set(), []
    piece = max(1, round(WALL_PIECE / resolution))
    for group in groups(tall):
        points = corners(group)
        if max(max(p[0] for p in points) - min(p[0] for p in points),
               max(p[1] for p in points) - min(p[1] for p in points)) < WALL_MIN_LENGTH:
            continue  # Tall but slight: a lamp's pole, a person. Furniture.
        structure.update(group)
        by_piece = {}
        for c, r in group:
            by_piece.setdefault((c // piece, r // piece), []).append((c, r))
        for cells in by_piece.values():
            for part in groups(cells):
                top = max(run[1] for cell in part for run in runs[cell])
                walls.append({**extent(part), "height": _cm(top),
                              "outline": outline(part)})

    # Then everything else, as objects of runs that touch.
    left_over = {(cell, k) for cell, found in runs.items() if cell not in structure
                 for k in range(len(found))}
    obstacles, unmeasured = [], []
    while left_over:
        todo, parts = [left_over.pop()], []
        while todo:
            cell, k = todo.pop()
            parts.append((cell, k))
            lo, hi = runs[cell][k]
            for n in [cell, *neighbours(*cell)]:
                for j, (nlo, nhi) in enumerate(runs.get(n, ())):
                    if ((n, j) in left_over and nlo <= hi + JOIN_GAP
                            and lo <= nhi + JOIN_GAP):
                        left_over.remove((n, j))
                        todo.append((n, j))
        lowest, tops = {}, {}
        for cell, k in parts:
            run = runs[cell][k]
            if cell not in lowest or run[0] < lowest[cell][0]:
                lowest[cell] = run
            tops[cell] = max(tops.get(cell, 0.0), run[1])
        cells = list(lowest)
        height = max(tops.values())
        bottom = min(run[0] for run in lowest.values())
        on_floor = MIN_OBSTACLE + RUN_GAP
        low_part = any(lo <= on_floor and hi <= TRIP_HAZARD_MAX
                       for lo, hi in lowest.values())
        edge = any(not (0 <= c < columns and 0 <= r < rows) or not known[r * columns + c]
                   for cell in cells for c, r in neighbours(*cell))
        levels = sorted(tops.values())
        step = (len(cells) * resolution ** 2 >= STEP_MIN_AREA
                and height <= TRIP_HAZARD_MAX
                and levels[len(levels) * 9 // 10] - levels[len(levels) // 10] <= ACCURACY
                and levels[len(levels) // 2] > MIN_OBSTACLE + RUN_GAP)
        found = {**extent(cells), "height": _cm(height), "outline": outline(cells)}
        if edge and not step:
            unmeasured.append(found)
            continue
        # A step rises from the floor even where the scan missed its riser.
        beam = [0.0 if step or bottom <= on_floor else _cm(bottom), _cm(height)]
        obstacles.append({
            "kind": "step" if step else "object",
            **found,
            "beam_range": beam,
            "trip_hazard": step or low_part,
            "lidar": lidar_view(beam, lidar_height),
        })

    # Numbered as a page is read: top row first, left to right.
    def reading(o):
        return (-o["center"][1], o["center"][0])
    obstacles.sort(key=reading)
    walls.sort(key=reading)
    unmeasured.sort(key=reading)
    obstacles = [{"id": number, **o} for number, o in enumerate(obstacles, 1)]
    walls = [{"id": number, **o} for number, o in enumerate(walls, 1)]

    a, b, c = scan.plane
    middle = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
    floor_cells = [(i % columns, i // columns) for i in range(len(floor)) if floor[i]]
    return {
        "source": scan.name,
        "units": "metres; map frame x = glTF x, y = -glTF z; "
                 "heights above the fitted floor",
        "lidar_height": lidar_height,
        "trip_hazard_max": TRIP_HAZARD_MAX,
        "min_obstacle": MIN_OBSTACLE,
        "marginal_band_m": MARGINAL_BAND,
        "accuracy_m": ACCURACY,
        "floor": {
            "area_m2": round(len(floor_cells) * resolution ** 2, 2),
            "outline": outline(floor_cells),
            "tilt_deg": round(math.degrees(math.atan(math.hypot(a, b))), 2),
            # glTF y is height, so this is where the viewer's floor sits -
            # to the millimetre, as it places the scan rather than measures it.
            "gltf_floor_y": round(a * middle[0] + b * middle[1] + c, 3),
            # The floor's glTF y at map (x, y) is a x + b y + c.
            "plane": [round(a, 5), round(b, 5), round(c, 4)],
        },
        "bounds": {"min": [_cm(min(xs)), _cm(min(ys))],
                   "max": [_cm(max(xs)), _cm(max(ys))]},
        "obstacles": obstacles,
        "walls": walls,
        "unmeasured": unmeasured,
    }


def main() -> None:
    """Write the lidar-slice map, the obstacle report, or both.

    A scan with nothing tall enough to cross the lidar's plane - a patch of
    floor with low things on it - has no map to give: every cell would be
    free, which says the lidar sees a clear floor. That is the finding, not
    a failure. So no .pgm is written; the note says why, and the report still
    lists what is there.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scan", type=Path, help="Polycam export, .glb or .gltf.")
    parser.add_argument(
        "-f",
        "--output",
        type=Path,
        help="Output stem: writes <stem>.pgm and <stem>.yaml, like map_saver_cli.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="Also write a JSON report of everything standing on the floor.",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument(
        "--height",
        type=float,
        default=LIDAR_HEIGHT,
        help="Metres above the floor to slice at: the lidar's height "
             f"(default {LIDAR_HEIGHT}).",
    )
    arguments = parser.parse_args()
    if not arguments.output and not arguments.report:
        parser.error("give -f/--output, --report, or both")

    scan = Scan(arguments.scan)  # Read and fitted once, for both outputs.
    if arguments.report:
        found = report(scan, lidar_height=arguments.height)
        arguments.report.write_text(json.dumps(found, indent=1) + "\n")
        views = [o["lidar"] for o in found["obstacles"]]
        print(f"wrote {arguments.report}: {len(found['obstacles'])} obstacles - "
              f"{views.count('misses')} too low for the lidar to see, "
              f"{views.count('marginal')} at the edge of what it might see; "
              f"{len(found['walls'])} pieces of wall; "
              f"{len(found['unmeasured'])} more raised patches at the scan's edge "
              "or partly hidden were not measured - check them by eye")

    if arguments.output:
        try:
            grid = render(scan, arguments.resolution, arguments.height)
        except NothingAtHeight:
            print(f"no map written: nothing in the scan reaches {arguments.height} m "
                  "(the lidar's height) above the floor, so the lidar "
                  "would see an empty floor here. "
                  "Anything on it is too low for the lidar - see --report.")
            return
        pgm = arguments.output.with_suffix(".pgm")
        pgm.write_bytes(grid.pgm())
        arguments.output.with_suffix(".yaml").write_text(grid.yaml(pgm.name))
        print(f"wrote {pgm} ({grid.width}x{grid.height} cells) and its .yaml")


if __name__ == "__main__":
    main()
