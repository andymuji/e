from dataclasses import dataclass
from enum import Enum
import math


class SafetyState(str, Enum):
    CLEAR = "clear"
    CAUTION = "caution"
    STOP = "stop"


@dataclass(frozen=True)
class SafetyDecision:
    state: SafetyState
    speed_scale: float
    reason: str


class SafetyController:
    """Convert sensor state into a bounded, fail-closed motion decision."""

    def __init__(
        self,
        stop_distance: float = 0.35,
        caution_distance: float = 0.8,
        sensor_timeout: float = 0.5,
    ):
        if stop_distance <= 0:
            raise ValueError("stop_distance must be positive")
        if caution_distance <= stop_distance:
            raise ValueError("caution_distance must exceed stop_distance")
        if sensor_timeout <= 0 or not math.isfinite(sensor_timeout):
            raise ValueError("sensor_timeout must be finite and positive")

        self.stop_distance = stop_distance
        self.caution_distance = caution_distance
        self.sensor_timeout = sensor_timeout
        self._emergency_stop = False

    def set_emergency_stop(self, active: bool) -> None:
        self._emergency_stop = active

    def evaluate(
        self,
        nearest_obstacle_distance: float | None,
        reading_age: float = 0.0,
    ) -> SafetyDecision:
        if self._emergency_stop:
            return SafetyDecision(SafetyState.STOP, 0.0, "emergency stop active")

        if not math.isfinite(reading_age) or reading_age < 0:
            return SafetyDecision(SafetyState.STOP, 0.0, "invalid obstacle sensor timestamp")

        if reading_age > self.sensor_timeout:
            return SafetyDecision(SafetyState.STOP, 0.0, "obstacle sensor timed out")

        if nearest_obstacle_distance is None or not math.isfinite(nearest_obstacle_distance):
            return SafetyDecision(SafetyState.STOP, 0.0, "invalid obstacle sensor reading")

        if nearest_obstacle_distance <= self.stop_distance:
            return SafetyDecision(SafetyState.STOP, 0.0, "obstacle inside stop distance")

        if nearest_obstacle_distance <= self.caution_distance:
            return SafetyDecision(SafetyState.CAUTION, 0.35, "obstacle inside caution distance")

        return SafetyDecision(SafetyState.CLEAR, 1.0, "path clear")
