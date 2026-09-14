from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("pose values must be finite")


@dataclass(frozen=True)
class NavigationGoal:
    location_name: str
    pose: Pose2D

    def __post_init__(self) -> None:
        if not self.location_name.strip():
            raise ValueError("location_name must not be empty")
