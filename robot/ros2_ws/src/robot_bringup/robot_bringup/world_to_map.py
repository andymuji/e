"""Render a Gazebo world into the occupancy map Nav2 localizes against.

THIS IS NOT A SLAM RESULT. A map built by driving the robot is the real
article: it contains what the lidar actually saw, including the places it
could not see, and it is what the robot will meet on hardware. This module
exists because the map has to come from somewhere before anything can be
navigated, and a map computed from the world file is at least honest about
what it is - exact geometry, no sensor, no drift, no unseen corners.

Replace the committed map with a driven one as soon as a simulator run can
be completed; see docs/getting-started.md.

Two differences from a SLAM map matter in use:

* The map frame is the Gazebo world frame, because that is the frame the
  world file is written in. A SLAM map is anchored at wherever the robot
  happened to start, so the two are not interchangeable and the initial pose
  given to AMCL differs between them.
* Everything beyond the room's outer wall is unknown, including whatever lies
  past the doorway. A lidar would have seen a little way through it.

Only box collisions are rendered, and only those that span the height the
lidar sits at - the same readings the robot would have to plan around.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

# Greyscale values map_server reads back as the three occupancy states.
OCCUPIED = 0
FREE = 254
UNKNOWN = 205

# The room is what gets mapped. Anything outside the walls is unknown, so a
# gap in a wall does not flood the space around the building with free cells
# that nothing ever observed.
ROOM_MODEL = "walls"


@dataclass(frozen=True)
class Footprint:
    """One box collision, flattened to the rectangle it occupies on the floor."""

    x: float
    y: float
    length: float
    width: float
    yaw: float

    def contains(self, px: float, py: float) -> bool:
        dx, dy = px - self.x, py - self.y
        cos, sin = math.cos(-self.yaw), math.sin(-self.yaw)
        local_x = dx * cos - dy * sin
        local_y = dx * sin + dy * cos
        return abs(local_x) <= self.length / 2.0 and abs(local_y) <= self.width / 2.0


def _pose(element: ET.Element | None) -> tuple[float, float, float, float]:
    """The x, y, z, yaw of an SDF <pose>, which may be absent."""
    if element is None:
        return 0.0, 0.0, 0.0, 0.0
    found = element.find("pose")
    if found is None or not (found.text or "").strip():
        return 0.0, 0.0, 0.0, 0.0
    values = [float(part) for part in found.text.split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[2], values[5]


def footprints(world: ET.Element, lidar_height: float) -> list[Footprint]:
    """Every box collision in the world that the lidar beam would strike."""
    found = []
    for model in world.iter("model"):
        model_x, model_y, model_z, model_yaw = _pose(model)
        for link in model.iter("link"):
            link_x, link_y, link_z, link_yaw = _pose(link)
            for collision in link.iter("collision"):
                box = collision.find("geometry/box/size")
                if box is None:
                    continue  # A plane or a mesh: the ground, not an obstacle.
                length, width, height = (float(part) for part in box.text.split())
                col_x, col_y, col_z, col_yaw = _pose(collision)

                yaw = model_yaw + link_yaw + col_yaw
                cos, sin = math.cos(model_yaw), math.sin(model_yaw)
                offset_x = link_x + col_x
                offset_y = link_y + col_y
                x = model_x + offset_x * cos - offset_y * sin
                y = model_y + offset_x * sin + offset_y * cos
                centre_z = model_z + link_z + col_z

                if not centre_z - height / 2.0 <= lidar_height <= centre_z + height / 2.0:
                    continue  # Under the beam or over it: invisible to the scan.
                found.append(Footprint(x, y, length, width, yaw))
    return found


def _bounds(shapes: list[Footprint]) -> tuple[float, float, float, float]:
    """The axis-aligned extent of some footprints, corners included."""
    xs, ys = [], []
    for shape in shapes:
        for sx in (-shape.length / 2.0, shape.length / 2.0):
            for sy in (-shape.width / 2.0, shape.width / 2.0):
                cos, sin = math.cos(shape.yaw), math.sin(shape.yaw)
                xs.append(shape.x + sx * cos - sy * sin)
                ys.append(shape.y + sx * sin + sy * cos)
    return min(xs), min(ys), max(xs), max(ys)


@dataclass(frozen=True)
class OccupancyGrid:
    cells: list[list[int]]  # Row 0 is the lowest y; the PGM is written flipped.
    resolution: float
    origin: tuple[float, float]

    @property
    def width(self) -> int:
        return len(self.cells[0])

    @property
    def height(self) -> int:
        return len(self.cells)

    def pgm(self) -> bytes:
        header = f"P5\n{self.width} {self.height}\n255\n".encode("ascii")
        rows = (bytes(row) for row in reversed(self.cells))
        return header + b"".join(rows)

    def yaml(self, image_name: str) -> str:
        x, y = self.origin
        return (
            f"image: {image_name}\n"
            "mode: trinary\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{x:.3f}, {y:.3f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )


def render(
    world_path: Path,
    resolution: float = 0.05,
    margin: float = 0.3,
    lidar_height: float = 0.195,
    seed: tuple[float, float] = (-2.0, -1.5),
) -> OccupancyGrid:
    """Turn a world file into an occupancy grid.

    `seed` is a point known to be inside the room: free space is whatever can
    be reached from it without crossing an obstacle, which is what keeps the
    inside of a cabinet from being mapped as somewhere to drive.
    """
    world = ET.parse(world_path).getroot()
    shapes = footprints(world, lidar_height)
    if not shapes:
        raise ValueError(f"no box collisions at lidar height in {world_path}")

    room = [
        shape
        for model in world.iter("model")
        if model.get("name") == ROOM_MODEL
        for shape in footprints(model, lidar_height)
    ]
    if not room:
        raise ValueError(f"world has no model named {ROOM_MODEL!r} to bound the map")
    room_min_x, room_min_y, room_max_x, room_max_y = _bounds(room)

    min_x, min_y, max_x, max_y = _bounds(shapes)
    origin = (min_x - margin, min_y - margin)
    width = int(math.ceil((max_x + margin - origin[0]) / resolution))
    height = int(math.ceil((max_y + margin - origin[1]) / resolution))

    def centre(column: int, row: int) -> tuple[float, float]:
        return (
            origin[0] + (column + 0.5) * resolution,
            origin[1] + (row + 0.5) * resolution,
        )

    cells = [[UNKNOWN] * width for _ in range(height)]
    blocked = [[False] * width for _ in range(height)]
    for row in range(height):
        for column in range(width):
            px, py = centre(column, row)
            if any(shape.contains(px, py) for shape in shapes):
                blocked[row][column] = True
                cells[row][column] = OCCUPIED

    # Free space is what is reachable from inside the room. Unreached cells
    # stay unknown, which is what a map_server needs to refuse a goal that is
    # nowhere the robot could have seen.
    start = (
        int((seed[0] - origin[0]) / resolution),
        int((seed[1] - origin[1]) / resolution),
    )
    queue = deque([start])
    while queue:
        column, row = queue.popleft()
        if not (0 <= column < width and 0 <= row < height):
            continue
        if blocked[row][column] or cells[row][column] == FREE:
            continue
        px, py = centre(column, row)
        if not (room_min_x <= px <= room_max_x and room_min_y <= py <= room_max_y):
            continue  # Past the doorway: real, but never observed.
        cells[row][column] = FREE
        queue.extend(
            [(column + 1, row), (column - 1, row), (column, row + 1), (column, row - 1)]
        )

    return OccupancyGrid(cells, resolution, origin)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("world", type=Path, help="World .sdf to render.")
    parser.add_argument(
        "-f",
        "--output",
        type=Path,
        required=True,
        help="Output stem: writes <stem>.pgm and <stem>.yaml, like map_saver_cli.",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    arguments = parser.parse_args()

    grid = render(arguments.world, resolution=arguments.resolution)
    pgm = arguments.output.with_suffix(".pgm")
    pgm.write_bytes(grid.pgm())
    arguments.output.with_suffix(".yaml").write_text(grid.yaml(pgm.name))
    print(f"wrote {pgm} ({grid.width}x{grid.height} cells) and its .yaml")


if __name__ == "__main__":
    main()
