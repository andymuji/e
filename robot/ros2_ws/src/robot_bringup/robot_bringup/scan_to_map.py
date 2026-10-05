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


def render(scan: Path, resolution: float = 0.05, height: float = 0.195,
           margin: float = 0.3) -> OccupancyGrid:
    """Turn a glTF scan into an occupancy grid sliced `height` above the floor."""
    tris = triangles(scan)
    heights = sorted(p[2] for tri in tris for p in tri)
    floor = heights[int(len(heights) * FLOOR_PERCENTILE)]
    plane = floor + height

    walls = [cut for tri in tris if (cut := _cut(tri, plane))]
    floors = [tri for tri in tris if all(abs(p[2] - floor) <= FLOOR_BAND for p in tri)]
    if not walls:
        raise ValueError(f"nothing in {scan} crosses {height} m above the floor")
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scan", type=Path, help="Polycam export, .glb or .gltf.")
    parser.add_argument(
        "-f",
        "--output",
        type=Path,
        required=True,
        help="Output stem: writes <stem>.pgm and <stem>.yaml, like map_saver_cli.",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument(
        "--height",
        type=float,
        default=0.195,
        help="Metres above the floor to slice at: the lidar's height (default 0.195).",
    )
    arguments = parser.parse_args()

    grid = render(arguments.scan, arguments.resolution, arguments.height)
    pgm = arguments.output.with_suffix(".pgm")
    pgm.write_bytes(grid.pgm())
    arguments.output.with_suffix(".yaml").write_text(grid.yaml(pgm.name))
    print(f"wrote {pgm} ({grid.width}x{grid.height} cells) and its .yaml")


if __name__ == "__main__":
    main()
