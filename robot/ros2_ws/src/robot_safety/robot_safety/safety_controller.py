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


def _stop(reason: str) -> SafetyDecision:
    return SafetyDecision(SafetyState.STOP, 0.0, reason)


class SafetyController:
    """Convert sensor state into a bounded, fail-closed motion decision."""

    # Defaults are the fallback for a node started with no parameter file.
    # They mirror the values derived in robot_bringup/config/base_dynamics.yaml
    # and are asserted to be no less conservative than it by the bringup tests.
    CAUTION_SPEED_SCALE = 0.35

    def __init__(
        self,
        stop_distance: float = 0.45,
        caution_distance: float = 0.85,
        sensor_timeout: float = 0.5,
        command_timeout: float = 0.5,
        require_localization: bool = False,
        localization_timeout: float = 1.0,
        max_localization_covariance: float | None = None,
        low_battery_fraction: float | None = None,
    ):
        if stop_distance <= 0:
            raise ValueError("stop_distance must be positive")
        if caution_distance <= stop_distance:
            raise ValueError("caution_distance must exceed stop_distance")
        if sensor_timeout <= 0 or not math.isfinite(sensor_timeout):
            raise ValueError("sensor_timeout must be finite and positive")
        if command_timeout <= 0 or not math.isfinite(command_timeout):
            raise ValueError("command_timeout must be finite and positive")
        if localization_timeout <= 0 or not math.isfinite(localization_timeout):
            raise ValueError("localization_timeout must be finite and positive")
        if max_localization_covariance is not None and (
            max_localization_covariance <= 0
            or not math.isfinite(max_localization_covariance)
        ):
            raise ValueError(
                "max_localization_covariance must be finite and positive"
            )
        if low_battery_fraction is not None and not 0.0 < low_battery_fraction <= 1.0:
            raise ValueError("low_battery_fraction must be in (0.0, 1.0]")

        self.stop_distance = stop_distance
        self.caution_distance = caution_distance
        self.sensor_timeout = sensor_timeout
        self.command_timeout = command_timeout
        self.require_localization = require_localization
        self.localization_timeout = localization_timeout
        self.max_localization_covariance = max_localization_covariance
        self.low_battery_fraction = low_battery_fraction
        self._emergency_stop = False

    @property
    def emergency_stop_engaged(self) -> bool:
        return self._emergency_stop

    def engage_emergency_stop(self) -> None:
        """Latch the software stop. Only clear_emergency_stop releases it."""
        self._emergency_stop = True

    def clear_emergency_stop(self) -> None:
        """Release the latch. Reserved for a deliberate operator reset."""
        self._emergency_stop = False

    def evaluate(
        self,
        nearest_obstacle_distance: float | None,
        reading_age: float | None = 0.0,
        command_age: float | None = 0.0,
        localization_age: float | None = None,
        localization_covariance: float | None = None,
        battery_fraction: float | None = None,
    ) -> SafetyDecision:
        """Decide how fast the robot may move, given everything known about it.

        `localization_age` and `battery_fraction` are only enforced when the
        corresponding check is configured. A robot being driven by teleop has
        no localization to lose, and a simulated base has no battery, so a
        gate that demanded them unconditionally would refuse to move at all.
        Nav2 bringup turns them on; see robot_bringup/config/safety.yaml.
        """
        if self._emergency_stop:
            return _stop("emergency stop active")

        lost = self._check_localization(localization_age, localization_covariance)
        if lost is not None:
            return lost

        flat = self._check_battery(battery_fraction)
        if flat is not None:
            return flat

        stale = self._check_age(reading_age, self.sensor_timeout, "obstacle sensor")
        if stale is not None:
            return stale

        stale = self._check_age(command_age, self.command_timeout, "motion command")
        if stale is not None:
            return stale

        if nearest_obstacle_distance is None or not math.isfinite(
            nearest_obstacle_distance
        ):
            return _stop("invalid obstacle sensor reading")

        if nearest_obstacle_distance <= self.stop_distance:
            return SafetyDecision(SafetyState.STOP, 0.0, "obstacle inside stop distance")

        if nearest_obstacle_distance <= self.caution_distance:
            return SafetyDecision(
                SafetyState.CAUTION,
                self.CAUTION_SPEED_SCALE,
                "obstacle inside caution distance",
            )

        return SafetyDecision(SafetyState.CLEAR, 1.0, "path clear")

    def _check_localization(
        self, age: float | None, covariance: float | None
    ) -> SafetyDecision | None:
        """Stop when the robot no longer credibly knows where it is.

        A pose that is merely old is not the only failure: AMCL keeps
        publishing after it has diverged, so an implausibly large covariance
        counts as lost too.
        """
        if not self.require_localization:
            return None

        stale = self._check_age(age, self.localization_timeout, "localization")
        if stale is not None:
            return stale

        if self.max_localization_covariance is None:
            return None
        if covariance is None or not math.isfinite(covariance):
            return _stop("invalid localization covariance")
        if covariance > self.max_localization_covariance:
            return _stop("localization uncertainty too high")
        return None

    def _check_battery(self, fraction: float | None) -> SafetyDecision | None:
        """Stop on a flat battery.

        This is the floor, not a power policy: deciding to break off a trip
        and return to a charger belongs above the gate, while there is still
        enough charge to drive there. By the time this fires, the safe thing
        left to do is stop.
        """
        if self.low_battery_fraction is None:
            return None
        if fraction is None or not math.isfinite(fraction):
            return _stop("battery level unknown")
        if fraction <= self.low_battery_fraction:
            return _stop("battery below reserve")
        return None

    @staticmethod
    def _check_age(
        age: float | None, timeout: float, label: str
    ) -> SafetyDecision | None:
        """Return a stop decision when an input is missing, invalid, or stale."""
        if age is None:
            return _stop(f"no {label} received")
        if not math.isfinite(age) or age < 0:
            return _stop(f"invalid {label} timestamp")
        if age > timeout:
            return _stop(f"{label} timed out")
        return None
