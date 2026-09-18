import json
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
        self._locations[normalized_name] = pose
        self._write()

    def remove_location(self, name: str) -> None:
        normalized_name = self._normalize_name(name)
        try:
            del self._locations[normalized_name]
        except KeyError as error:
            raise KeyError(f"unknown location: {normalized_name}") from error
        self._write()

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

    def _write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            name: {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
            for name, pose in sorted(self._locations.items())
        }
        self.path.write_text(json.dumps(payload, indent=2) + "\n")

    @staticmethod
    def _normalize_name(name: str) -> str:
        normalized_name = " ".join(name.strip().lower().split())
        if not normalized_name:
            raise ValueError("location name must not be empty")
        return normalized_name
