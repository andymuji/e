"""Turn a phone scan of a room (Polycam glTF export) into an occupancy map.

THIS IS NOT A SLAM RESULT either. A phone carried or driven round the room
measures it well, but the robot's own lidar did not see it: nothing here says
the robot can localize against this map, and the lidar height it is sliced
at is a guess until the real lidar is mounted and measured.

What it does, in the same shape as world_to_map:

* the scan's floor is found from the mesh (the lowest heights, ignoring a few
  stray points below it);
* the mesh is cut by a horizontal plane at `--height` above that floor -
  what a lidar mounted there would strike - and every cut is occupied;
* every cell with scanned floor under it and no cut in it is free;
* everything else is unknown, including floor the phone never saw.

glTF is y-up and in metres. The map is z-up, so a glTF point (x, y, z) lands
at map (x, -z) at height y. The map frame is wherever the scan started.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
import struct

from robot_bringup.world_to_map import FREE, OCCUPIED, UNKNOWN, OccupancyGrid

# A triangle whose corners all lie this close to the floor is floor.
FLOOR_BAND = 0.05
# Heights below this fraction of all vertices are noise under the floor.
FLOOR_PERCENTILE = 0.01

# The obstacle report seeds its floor from the lowest cell in tiles this wide.
FLOOR_TILE = 0.5

# The obstacle report's thresholds, all in metres above the floor.
# The lidar's height - a guess until the real lidar is mounted and measured.
LIDAR_HEIGHT = 0.195
# Lower than this is the floor's own texture and the scan's noise.
MIN_OBSTACLE = 0.02
# Up to this, a thing is low enough to trip over rather than walk round.
TRIP_HAZARD_MAX = 0.30

_COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
_FORMATS = {5121: "B", 5123: "H", 5125: "I", 5126: "f"}
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


def _accessor(gltf: dict, buffers: list[bytes], index: int) -> list[tuple]:
    accessor = gltf["accessors"][index]
    view = gltf["bufferViews"][accessor["bufferView"]]
    fmt = "<" + _FORMATS[accessor["componentType"]] * _COMPONENTS[accessor["type"]]
    stride = view.get("byteStride") or struct.calcsize(fmt)
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    buffer = buffers[view.get("buffer", 0)]
    return [
        struct.unpack_from(fmt, buffer, start + i * stride)
        for i in range(accessor["count"])
    ]


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


def triangles(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    """Every triangle in the scene, in z-up metres: (map x, map y, height)."""
    gltf, buffers = _load(path)
    scene = gltf["scenes"][gltf.get("scene", 0)]
    found = []
    pending = [(index, _IDENTITY) for index in scene["nodes"]]
    while pending:
        index, parent = pending.pop()
        node = gltf["nodes"][index]
        world = _matmul(parent, _local(node))
        pending.extend((child, world) for child in node.get("children", []))
        if "mesh" not in node:
            continue
        for primitive in gltf["meshes"][node["mesh"]]["primitives"]:
            if primitive.get("mode", 4) != 4:
                continue  # Points or lines: nothing to slice.
            points = []
            for x, y, z in _accessor(gltf, buffers, primitive["attributes"]["POSITION"]):
                wx = world[0] * x + world[4] * y + world[8] * z + world[12]
                wy = world[1] * x + world[5] * y + world[9] * z + world[13]
                wz = world[2] * x + world[6] * y + world[10] * z + world[14]
                points.append((wx, -wz, wy))
            if "indices" in primitive:
                order = [i[0] for i in _accessor(gltf, buffers, primitive["indices"])]
            else:
                order = list(range(len(points)))
            found.extend(
                (points[order[i]], points[order[i + 1]], points[order[i + 2]])
                for i in range(0, len(order) - 2, 3)
            )
    if not found:
        raise ValueError(f"no triangles in {path}")
    return found


class NothingAtHeight(ValueError):
    """Nothing in the scan reaches the slice: the lidar would see no obstacle."""


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


def render(scan: Path, resolution: float = 0.05, height: float = LIDAR_HEIGHT,
           margin: float = 0.3) -> OccupancyGrid:
    """Turn a glTF scan into an occupancy grid sliced `height` above the floor.

    The slice is level, `height` above the lowest of the floor. On a tilted
    floor that is less than `height` above the high side, so it can only
    catch more than report(), which measures from the fitted, tilted floor -
    never less. The two can disagree on a thing at about the lidar's height.
    """
    tris = triangles(scan)
    heights = sorted(p[2] for tri in tris for p in tri)
    floor = heights[int(len(heights) * FLOOR_PERCENTILE)]
    plane = floor + height

    walls = [cut for tri in tris if (cut := _cut(tri, plane))]
    floors = [tri for tri in tris if all(abs(p[2] - floor) <= FLOOR_BAND for p in tri)]
    if not walls:
        raise NothingAtHeight(f"nothing in {scan} crosses {height} m above the floor")
    if not floors:
        raise ValueError(f"no floor found in {scan}")

    xs = [x for seg in walls for x, _ in seg] + [p[0] for t in floors for p in t]
    ys = [y for seg in walls for _, y in seg] + [p[1] for t in floors for p in t]
    origin = (min(xs) - margin, min(ys) - margin)
    width = int(math.ceil((max(xs) + margin - origin[0]) / resolution))
    rows = int(math.ceil((max(ys) + margin - origin[1]) / resolution))
    cells = [[UNKNOWN] * width for _ in range(rows)]

    def cell(x: float, y: float) -> tuple[int, int]:
        return int((x - origin[0]) / resolution), int((y - origin[1]) / resolution)

    for tri in floors:
        (ax, ay), (bx, by), (cx, cy) = ((p[0], p[1]) for p in tri)
        for x, y in ((ax, ay), (bx, by), (cx, cy)):
            column, row = cell(x, y)
            cells[row][column] = FREE  # Covers a triangle smaller than a cell.
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if area == 0:
            continue
        left, bottom = cell(min(ax, bx, cx), min(ay, by, cy))
        right, top = cell(max(ax, bx, cx), max(ay, by, cy))
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
    sx = sy = sz = sxx = sxy = syy = sxz = syz = 0.0
    for x, y, z in points:
        sx, sy, sz = sx + x, sy + y, sz + z
        sxx, sxy, syy = sxx + x * x, sxy + x * y, syy + y * y
        sxz, syz = sxz + x * z, syz + y * z
    return tuple(_solve3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, len(points)]],
                         [sxz, syz, sz]))


def _hull(points: list[tuple[float, float]]) -> list[list[float]]:
    """Convex hull, anticlockwise, rounded to the millimetre."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return [[round(x, 3), round(y, 3)] for x, y in pts]

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
    return [[round(x, 3), round(y, 3)] for x, y in chain(pts) + chain(pts[::-1])]


def report(scan: Path, resolution: float = 0.03,
           lidar_height: float = LIDAR_HEIGHT) -> dict:
    """What stands on the scanned floor, and whether the lidar would see it.

    The lidar slice above only shows what crosses one plane. An older person
    trips over what is under it - a shoe, a cable, a ball - so this looks at
    everything that rises off the floor, however low.

    * The mesh is rasterized into cells: each keeps the lowest and highest
      surface over it. A cell no triangle covers is unknown.
    * The floor is a plane fitted to the cells' lowest surfaces, refitted
      twice without the cells that sit well off it, so a slight tilt in the
      floor (or in how the phone held level) does not read as an obstacle.
    * Every cell whose highest surface is more than MIN_OBSTACLE above that
      plane is raised; touching raised cells are one obstacle.
    * A raised patch touching unknown cells or the scan's edge goes in
      `unmeasured`, not `obstacles`. Where a phone scan runs out the mesh
      curls up into a ragged rim, which is an artifact and not a thing - but
      so does a real object half out of the scan, one hiding a shadow the
      phone never saw behind it, or furniture merged with a wall. Its height
      and extent cannot be trusted, and it is never dropped silently: a
      report of no trip hazards must not mean they were all thrown away.
    """
    tris = triangles(scan)
    xs = [p[0] for tri in tris for p in tri]
    ys = [p[1] for tri in tris for p in tri]
    origin = (min(xs), min(ys))
    columns = int((max(xs) - origin[0]) / resolution) + 1
    rows = int((max(ys) - origin[1]) / resolution) + 1
    low = [[None] * columns for _ in range(rows)]
    high = [[None] * columns for _ in range(rows)]

    def mark(column, row, z):
        if 0 <= row < rows and 0 <= column < columns:
            if low[row][column] is None or z < low[row][column]:
                low[row][column] = z
            if high[row][column] is None or z > high[row][column]:
                high[row][column] = z

    for (ax, ay, az), (bx, by, bz), (cx, cy, cz) in tris:
        for x, y, z in ((ax, ay, az), (bx, by, bz), (cx, cy, cz)):
            mark(int((x - origin[0]) / resolution), int((y - origin[1]) / resolution), z)
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if area == 0:
            continue  # Seen edge-on: a vertical face. Its corners are marked.
        for row in range(int((min(ay, by, cy) - origin[1]) / resolution),
                         int((max(ay, by, cy) - origin[1]) / resolution) + 1):
            for column in range(int((min(ax, bx, cx) - origin[0]) / resolution),
                                int((max(ax, bx, cx) - origin[0]) / resolution) + 1):
                px = origin[0] + (column + 0.5) * resolution
                py = origin[1] + (row + 0.5) * resolution
                u = ((bx - px) * (cy - py) - (cx - px) * (by - py)) / area
                v = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) / area
                if u >= 0 and v >= 0 and u + v <= 1:
                    mark(column, row, u * az + v * bz + (1 - u - v) * cz)

    def centre(column, row):
        return (origin[0] + (column + 0.5) * resolution,
                origin[1] + (row + 0.5) * resolution)

    known = [(c, r) for r in range(rows) for c in range(columns) if low[r][c] is not None]
    # Seed with the lowest cell in each tile: floor wherever a tile has any,
    # so a big room's slope is in the seed and not mistaken for obstacles.
    lowest = {}
    tile = max(1, round(FLOOR_TILE / resolution))
    for c, r in known:
        key = (c // tile, r // tile)
        if key not in lowest or low[r][c] < low[lowest[key][1]][lowest[key][0]]:
            lowest[key] = (c, r)
    a, b, c0 = _fit_plane([(*centre(c, r), low[r][c]) for c, r in lowest.values()])

    def floor_at(c, r):
        x, y = centre(c, r)
        return a * x + b * y + c0

    # Then refit to the cells near it, closer each time, so a tile that is
    # all bed or all scan rim drops out.
    for band in (FLOOR_BAND, MIN_OBSTACLE, MIN_OBSTACLE):
        a, b, c0 = _fit_plane([(*centre(c, r), low[r][c]) for c, r in known
                               if abs(low[r][c] - floor_at(c, r)) <= band])

    def above(c, r):
        return high[r][c] - floor_at(c, r)

    raised = {(c, r) for c, r in known if above(c, r) > MIN_OBSTACLE}
    obstacles, unmeasured = [], []
    while raised:
        blob, todo, edge = [], [raised.pop()], False
        while todo:
            c, r = todo.pop()
            blob.append((c, r))
            # Diagonals too: a box's corner can land in the cell kitty-corner.
            for nc, nr in ((c + dc, r + dr) for dc in (-1, 0, 1) for dr in (-1, 0, 1)):
                if not (0 <= nr < rows and 0 <= nc < columns) or low[nr][nc] is None:
                    edge = True
                elif (nc, nr) in raised:
                    raised.remove((nc, nr))
                    todo.append((nc, nr))
        corners = [(origin[0] + (c + dc) * resolution, origin[1] + (r + dr) * resolution)
                   for c, r in blob for dc in (0, 1) for dr in (0, 1)]
        x0, x1 = min(p[0] for p in corners), max(p[0] for p in corners)
        y0, y1 = min(p[1] for p in corners), max(p[1] for p in corners)
        height = round(max(above(c, r) for c, r in blob), 3)
        found = {
            "center": [round((x0 + x1) / 2, 3), round((y0 + y1) / 2, 3)],
            "size": [round(x1 - x0, 3), round(y1 - y0, 3)],
            "height": height,
        }
        if edge:
            unmeasured.append(found)
            continue
        obstacles.append({
            **found,
            "outline": _hull(corners),
            "trip_hazard": MIN_OBSTACLE < height <= TRIP_HAZARD_MAX,
            "lidar_sees": height >= lidar_height,
        })
    # Numbered as a page is read: top row first, left to right.
    obstacles.sort(key=lambda o: (-o["center"][1], o["center"][0]))
    unmeasured.sort(key=lambda o: (-o["center"][1], o["center"][0]))
    obstacles = [{"id": number, **o} for number, o in enumerate(obstacles, 1)]

    middle = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
    return {
        "source": scan.name,
        "units": "metres; x,y in the map frame (x = glTF x, y = -glTF z); "
                 "heights above the fitted floor",
        "lidar_height": lidar_height,
        "trip_hazard_max": TRIP_HAZARD_MAX,
        "floor": {
            "area_m2": round(len(known) * resolution ** 2, 2),
            "outline": _hull(list(zip(xs, ys, strict=True))),
            "tilt_deg": round(math.degrees(math.atan(math.hypot(a, b))), 2),
            # glTF y is height, so this is where the viewer's floor sits.
            "gltf_floor_y": round(a * middle[0] + b * middle[1] + c0, 3),
        },
        "bounds": {"min": [round(min(xs), 3), round(min(ys), 3)],
                   "max": [round(max(xs), 3), round(max(ys), 3)]},
        "obstacles": obstacles,
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

    if arguments.report:
        found = report(arguments.scan, lidar_height=arguments.height)
        arguments.report.write_text(json.dumps(found, indent=1) + "\n")
        blind = sum(not o["lidar_sees"] for o in found["obstacles"])
        print(f"wrote {arguments.report}: {len(found['obstacles'])} obstacles, "
              f"{blind} of them too low for the lidar to see; "
              f"{len(found['unmeasured'])} more raised patches at the scan's edge "
              "or partly hidden were not measured - check them by eye")

    if arguments.output:
        try:
            grid = render(arguments.scan, arguments.resolution, arguments.height)
        except NothingAtHeight:
            print(f"no map written: nothing in the scan reaches {arguments.height} m "
                  "(the lidar's height) above the lowest of the floor, so the lidar "
                  "would see an empty floor here. "
                  "Anything on it is too low for the lidar - see --report.")
            return
        pgm = arguments.output.with_suffix(".pgm")
        pgm.write_bytes(grid.pgm())
        arguments.output.with_suffix(".yaml").write_text(grid.yaml(pgm.name))
        print(f"wrote {pgm} ({grid.width}x{grid.height} cells) and its .yaml")


if __name__ == "__main__":
    main()
