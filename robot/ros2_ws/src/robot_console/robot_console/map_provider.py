"""Build the console's floor plan from the map and the robot's transform.

The pose comes from the map->base_link transform rather than from amcl_pose.
AMCL publishes amcl_pose only when its filter updates, which needs the robot
to have moved; taking freshness from it once deadlocked the safety gate, which
declared localization lost a second after the robot stopped. A marker that
freezes on the page is the same bug with a smaller blast radius, and the
transform is broadcast whether or not the robot is moving.

Nothing is invented. No map means no walls, no transform means no marker, and
the page says which is missing rather than drawing a room.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Any

from robot_console.occupancy_image import data_url


@dataclass(frozen=True)
class GridSnapshot:
    """An occupancy grid, without the ROS message around it."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: Sequence[int]

    @property
    def max_x(self) -> float:
        return self.origin_x + self.width * self.resolution

    @property
    def max_y(self) -> float:
        return self.origin_y + self.height * self.resolution


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float


def _finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def _bounds(
    grid: GridSnapshot | None, locations: Sequence[dict[str, Any]]
) -> dict[str, float]:
    """A drawing area that holds the map and every approved destination.

    A location saved outside the mapped area still has to be visible: it is
    somewhere an operator can send the robot, and a label off the edge of the
    picture is a label nobody can check.
    """
    xs = [float(item["x"]) for item in locations]
    ys = [float(item["y"]) for item in locations]
    if grid is not None:
        xs += [grid.origin_x, grid.max_x]
        ys += [grid.origin_y, grid.max_y]
    if not xs:
        return {"min_x": 0.0, "min_y": 0.0, "max_x": 6.0, "max_y": 5.0}

    min_x, max_x = min(xs) - 1.0, max(xs) + 1.0
    min_y, max_y = min(ys) - 1.0, max(ys) + 1.0
    # A zero-width box divides by zero in the page's coordinate maths.
    if max_x - min_x < 1.0:
        max_x = min_x + 1.0
    if max_y - min_y < 1.0:
        max_y = min_y + 1.0
    return {"min_x": min_x, "min_y": min_y, "max_x": max_x, "max_y": max_y}


def build_map(
    grid: GridSnapshot | None,
    robot: RobotPose | None,
    locations: Sequence[dict[str, Any]],
    max_pixels: int = 640_000,
) -> dict[str, Any]:
    """Assemble the map payload, saying plainly what is missing from it."""
    locations = list(locations)
    payload: dict[str, Any] = {
        "available": grid is not None,
        "message": "",
        "bounds": _bounds(grid, locations),
        # The gate's picture of the world is the occupancy grid; there are no
        # vector walls behind a real map, so this stays empty and the page's
        # wall drawing does nothing.
        "walls": [],
        "robot": (
            None if robot is None else {"x": robot.x, "y": robot.y, "yaw": robot.yaw}
        ),
        "locations": locations,
    }

    if grid is not None:
        image = data_url(grid.data, grid.width, grid.height, max_pixels)
        if image is None:
            payload["available"] = False
            payload["message"] = "The map arrived truncated and was not drawn."
        else:
            payload["image"] = {
                "data_url": image,
                "min_x": grid.origin_x,
                "min_y": grid.origin_y,
                "max_x": grid.max_x,
                "max_y": grid.max_y,
            }

    missing = []
    if grid is None:
        missing.append("no map yet")
    if robot is None:
        missing.append("the robot's position is unknown")
    if missing and not payload["message"]:
        payload["message"] = f"{'; '.join(missing).capitalize()}."
    return payload


class RosMapProvider:
    """Hold the latest map, and ask for the pose each time the page draws."""

    def __init__(
        self,
        pose_source: Any,
        max_pixels: int = 640_000,
    ) -> None:
        # pose_source() returns a RobotPose or None, so the transform lookup
        # and its staleness rule stay with the node and this stays testable
        # without a tf tree.
        self._pose_source = pose_source
        self._max_pixels = max_pixels
        self._grid: GridSnapshot | None = None

    def on_grid(self, grid: GridSnapshot | None) -> None:
        self._grid = grid

    def map_data(self, locations: Sequence[dict[str, Any]]) -> dict[str, Any]:
        return build_map(self._grid, self._pose_source(), locations, self._max_pixels)


def snapshot_from_message(message: Any) -> GridSnapshot | None:
    """Convert nav_msgs/OccupancyGrid, refusing one that makes no sense.

    A grid with a non-finite origin or a resolution of zero would put the
    robot marker at an arbitrary place on the page, which is worse than no
    map at all.
    """
    info = message.info
    origin = info.origin.position
    if not _finite(info.resolution, origin.x, origin.y) or info.resolution <= 0.0:
        return None
    if info.width <= 0 or info.height <= 0:
        return None
    return GridSnapshot(
        resolution=float(info.resolution),
        width=int(info.width),
        height=int(info.height),
        origin_x=float(origin.x),
        origin_y=float(origin.y),
        data=list(message.data),
    )


def pose_from_transform(transform: Any) -> RobotPose | None:
    """Read x, y and yaw out of a geometry_msgs/TransformStamped."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    if not _finite(translation.x, translation.y, rotation.z, rotation.w):
        return None
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    return RobotPose(float(translation.x), float(translation.y), yaw)
