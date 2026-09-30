import json
import os
from pathlib import Path

from robot_core import NavigationGoal, Pose2D


class LocationStore:
    """Persist operator-approved named poses for a single map."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self._locations: dict[str, Pose2D] = {}
        if self.path.exists():
            self.load()

    def save_location(self, name: str, pose: Pose2D) -> None:
        normalized_name = self._normalize_name(name)
        self._write({**self._locations, normalized_name: pose})

    def remove_location(self, name: str) -> None:
        normalized_name = self._normalize_name(name)
        if normalized_name not in self._locations:
            raise KeyError(f"unknown location: {normalized_name}")
        self._write(
            {key: pose for key, pose in self._locations.items() if key != normalized_name}
        )

    def get_goal(self, name: str) -> NavigationGoal:
        normalized_name = self._normalize_name(name)
        try:
            pose = self._locations[normalized_name]
        except KeyError as error:
            raise KeyError(f"unknown location: {normalized_name}") from error
        return NavigationGoal(normalized_name, pose)

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._locations))

    def locations(self) -> tuple[tuple[str, Pose2D], ...]:
        """Return saved locations with their poses in a stable order."""
        return tuple((name, self._locations[name]) for name in sorted(self._locations))

    def load(self) -> None:
        content = json.loads(self.path.read_text())
        if not isinstance(content, dict):
            raise ValueError("location file must contain an object")
        self._locations = {
            self._normalize_name(name): Pose2D(
                float(values["x"]), float(values["y"]), float(values.get("yaw", 0.0))
            )
            for name, values in content.items()
        }

    def _write(self, locations: dict[str, Pose2D]) -> None:
        """Replace the file with `locations`, then adopt them in memory.

        Memory changes only once the file has: a failed write that still
        changed the in-memory copy would answer recalls from a place the next
        start-up no longer knows. The new content goes to a sibling file that
        is renamed over the old one, so a reader, or a power cut, sees the old
        file or the new one and never a half-written one.
        """
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            name: {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
            for name, pose in sorted(locations.items())
        }
        partial = self.path.with_name(self.path.name + ".tmp")
        with partial.open("w") as file:
            file.write(json.dumps(payload, indent=2) + "\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(partial, self.path)
        self._locations = locations

    @staticmethod
    def _normalize_name(name: str) -> str:
        normalized_name = " ".join(name.strip().lower().split())
        if not normalized_name:
            raise ValueError("location name must not be empty")
        return normalized_name
