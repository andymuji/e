"""Make up a house, map it, and find the best route between every pair of rooms.

THIS IS A PREVIEW, NOT A NAV2 RESULT. The routes come from the small grid
planner in robot_navigation, kept clear of the walls by the robot's
circumscribed radius and then straightened. Nav2 plans with its own planner
and costmap and will not draw exactly these lines. What the preview does say
honestly is whether the robot, at its URDF size, fits through every doorway.

The robot's size is read from the URDF, so it is a placeholder like the URDF
dimensions are, and the routes move when those are measured.

    python -m robot_bringup.house 1 2 3 4 5 -o docs/house-viewer/houses.js
"""

from __future__ import annotations

import argparse
import io
from itertools import combinations, pairwise
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from robot_navigation import BaseFootprint, GridPlanner

from robot_bringup.world_to_map import FREE, render

ROOM_NAMES = ["kitchen", "bedroom", "bathroom", "living room", "dining room", "office"]
WALL = 0.1  # Wall thickness, m.
DOOR = 0.9  # Doorway width, m: a common interior door.
MIN_ROOM = 2.5  # Shortest room side, m. Keeps cuts on a half-metre grid.
DOOR_CLEARANCE = 0.2  # Wall left either side of a doorway, m.
URDF = Path(__file__).resolve().parents[2] / "robot_description/urdf/robot.urdf.xacro"
XACRO_NS = "http://www.ros.org/wiki/xacro"


def _half_metres(rng: random.Random, low: float, high: float) -> float:
    return low + rng.randrange(int(round((high - low) * 2)) + 1) / 2


def _try_layout(rng: random.Random) -> dict | None:
    """One attempt at a house, or None when the cuts leave no room for a door."""
    width = _half_metres(rng, 8.0, 11.0)
    height = _half_metres(rng, 6.0, 9.0)
    rooms = [(0.0, 0.0, width, height)]
    # Each cut is a wall: (axis, position, start, end). "x" is the line x=position.
    cuts = []
    while len(rooms) < len(ROOM_NAMES):
        rooms.sort(key=lambda r: ((r[2] - r[0]) * (r[3] - r[1]), r))
        x0, y0, x1, y1 = rooms.pop()
        vertical = x1 - x0 >= y1 - y0
        low, high = (x0, x1) if vertical else (y0, y1)
        if high - low < 2 * MIN_ROOM:
            return None
        cut = _half_metres(rng, low + MIN_ROOM, high - MIN_ROOM)
        if vertical:
            rooms += [(x0, y0, cut, y1), (cut, y0, x1, y1)]
            cuts.append(("x", cut, y0, y1))
        else:
            rooms += [(x0, y0, x1, cut), (x0, cut, x1, y1)]
            cuts.append(("y", cut, x0, x1))

    # One door per cut joins the two halves it made, and every half is joined
    # inside itself, so every room is reachable. A door is only put where one
    # room on each side shares enough wall that no other wall meets it there.
    doors = []
    for axis, position, start, end in cuts:
        a = 0 if axis == "x" else 1
        options = []
        for low_room in rooms:
            for high_room in rooms:
                if low_room[a + 2] != position or high_room[a] != position:
                    continue
                o0 = max(low_room[1 - a], high_room[1 - a], start)
                o1 = min(low_room[3 - a], high_room[3 - a], end)
                if o1 - o0 >= DOOR + 2 * DOOR_CLEARANCE:
                    options.append((o0, o1))
        if not options:
            return None
        o0, o1 = rng.choice(sorted(options))
        margin = DOOR / 2 + DOOR_CLEARANCE
        doors.append((axis, position, round(rng.uniform(o0 + margin, o1 - margin), 2)))

    names = ROOM_NAMES[:]
    rng.shuffle(names)
    named = dict(zip(names, rooms, strict=True))

    def outside_walls(room):
        """Outside walls long enough for a door: (axis, position, centre, inward)."""
        x0, y0, x1, y1 = room
        sides = [
            ("x", 0.0, x0 == 0.0, (y0 + y1) / 2, y1 - y0, (1, 0)),
            ("x", width, x1 == width, (y0 + y1) / 2, y1 - y0, (-1, 0)),
            ("y", 0.0, y0 == 0.0, (x0 + x1) / 2, x1 - x0, (0, 1)),
            ("y", height, y1 == height, (x0 + x1) / 2, x1 - x0, (0, -1)),
        ]
        return [
            (axis, pos, centre, inward)
            for axis, pos, outside, centre, length, inward in sides
            if outside and length >= DOOR + 2 * DOOR_CLEARANCE
        ]

    # The front door opens into the living room when it has an outside wall.
    front = outside_walls(named["living room"]) or [
        side for room in rooms for side in outside_walls(room)
    ]
    axis, position, centre, (ix, iy) = rng.choice(front)
    doors.append((axis, position, centre))
    door_xy = (position, centre) if axis == "x" else (centre, position)
    locations = {"front door": (door_xy[0] + 0.6 * ix, door_xy[1] + 0.6 * iy)}
    for name, (x0, y0, x1, y1) in named.items():
        locations[name] = ((x0 + x1) / 2, (y0 + y1) / 2)

    outline = [
        ("x", 0.0, 0.0, height),
        ("x", width, 0.0, height),
        ("y", 0.0, 0.0, width),
        ("y", height, 0.0, width),
    ]
    walls = []
    for axis, position, start, end in outline + cuts:
        gaps = sorted(
            c for a, p, c in doors if a == axis and p == position and start < c < end
        )
        # Wall ends overlap by half a thickness so corners close; door edges do not.
        edges = [start - WALL / 2]
        for gap in gaps:
            edges += [gap - DOOR / 2, gap + DOOR / 2]
        edges.append(end + WALL / 2)
        for a, b in zip(edges[::2], edges[1::2], strict=True):
            middle, length = (a + b) / 2, b - a
            walls.append(
                (position, middle, WALL, length)
                if axis == "x"
                else (middle, position, length, WALL)
            )

    return {
        "width": width,
        "height": height,
        "rooms": {name: list(rect) for name, rect in named.items()},
        "walls": [[round(v, 3) for v in wall] for wall in walls],
        "locations": {name: [round(v, 3) for v in xy] for name, xy in locations.items()},
    }


def generate(seed: int) -> dict:
    """A house. The same seed always gives the same house."""
    rng = random.Random(seed)
    while (house := _try_layout(rng)) is None:
        pass
    return {"seed": seed, **house}


def world_sdf(house: dict) -> str:
    """The house's walls as an SDF world, every wall in the one `walls` model."""
    boxes = "".join(
        f'<collision name="wall_{i}"><pose>{x} {y} 0.5 0 0 0</pose>'
        f"<geometry><box><size>{sx} {sy} 1.0</size></box></geometry></collision>"
        for i, (x, y, sx, sy) in enumerate(house["walls"])
    )
    return (
        f'<sdf version="1.9"><world name="house_{house["seed"]}">'
        f'<model name="walls"><static>true</static><link name="link">{boxes}'
        "</link></model></world></sdf>"
    )


def robot_radius(urdf: Path = URDF) -> float:
    """Smallest circle holding the robot however it is turned, from the URDF."""
    properties = {
        e.get("name"): e.get("value")
        for e in ET.parse(urdf).getroot().iter(f"{{{XACRO_NS}}}property")
    }
    return BaseFootprint(
        float(properties["base_length"]), float(properties["base_width"]), 0.0
    ).circumscribed_radius


def routes(house: dict, radius: float) -> list[dict]:
    """The best route between every pair of named places, in metres.

    A cell the robot's centre may occupy is free and at least `radius` from
    anything not free. Unknown counts as blocked, as it does for Nav2.
    """
    grid = render(
        io.StringIO(world_sdf(house)), seed=next(iter(house["locations"].values()))
    )
    res, (ox, oy) = grid.resolution, grid.origin
    blocked = {
        (c, r)
        for r, row in enumerate(grid.cells)
        for c, value in enumerate(row)
        if value != FREE
    }
    reach = int(math.ceil(radius / res))
    stamp = [
        (dc, dr)
        for dc in range(-reach, reach + 1)
        for dr in range(-reach, reach + 1)
        if math.hypot(dc, dr) * res <= radius
    ]
    edge = [
        (c, r)
        for c, r in blocked
        if any(n not in blocked for n in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)))
    ]
    keep_out = blocked | {
        (c + dc, r + dr)
        for c, r in edge
        for dc, dr in stamp
        if 0 <= c + dc < grid.width and 0 <= r + dr < grid.height
    }
    planner = GridPlanner(grid.width, grid.height, keep_out)

    def clear(a, b) -> bool:
        steps = 2 * max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        return all(
            (
                round(a[0] + (b[0] - a[0]) * i / steps),
                round(a[1] + (b[1] - a[1]) * i / steps),
            )
            not in keep_out
            for i in range(steps + 1)
        )

    def straighten(path):
        # ponytail: greedy line-of-sight shortcut, not true any-angle planning;
        # a route can bend slightly more than it needs to. Theta* if that shows.
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = i + 1
            while j + 1 < len(path) and clear(path[i], path[j + 1]):
                j += 1
            out.append(path[j])
            i = j
        return out

    def cell(xy):
        return int((xy[0] - ox) / res), int((xy[1] - oy) / res)

    found = []
    for start, goal in combinations(sorted(house["locations"]), 2):
        cells = straighten(
            planner.plan(cell(house["locations"][start]), cell(house["locations"][goal]))
        )
        points = [
            [round(ox + (c + 0.5) * res, 3), round(oy + (r + 0.5) * res, 3)]
            for c, r in cells
        ]
        length = sum(math.dist(p, q) for p, q in pairwise(points))
        found.append(
            {"from": start, "to": goal, "length": round(length, 2), "path": points}
        )
    return found


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("seeds", type=int, nargs="+", help="One house per seed.")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Viewer data file to write (.js).",
    )
    arguments = parser.parse_args()

    radius = robot_radius()
    houses = []
    for seed in arguments.seeds:
        house = generate(seed)
        houses.append(
            {**house, "robot_radius": round(radius, 3), "routes": routes(house, radius)}
        )
        print(
            f"house {seed}: {house['width']} x {house['height']} m,"
            f" {len(houses[-1]['routes'])} routes"
        )
    arguments.output.write_text(
        "// Generated by python -m robot_bringup.house. Do not edit.\n"
        f"window.HOUSES = {json.dumps(houses, separators=(',', ':'))};\n"
    )
    print(f"wrote {arguments.output}")


if __name__ == "__main__":
    main()
