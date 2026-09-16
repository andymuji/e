"""Derive the safety gate's distances from the base geometry and dynamics.

`stop_distance` and `caution_distance` are not preferences. They are the
distance the robot still travels after the lidar first sees an obstacle:
however long the reading takes to become a wheel command, plus however far
the base slides while braking, plus whatever sticks out in front of the
sensor. Writing them by hand means that when the base, the sensor rate, or
the speed limit changes, the numbers stay put and quietly stop being true.

Everything here is derived from `BaseDynamics`. The placeholder values live
in `robot_bringup/config/base_dynamics.yaml` with `measured: false`; the test
suite fails if the distances shipped in `safety.yaml` drift away from what
these inputs imply, so replacing the inputs with measured ones is the only
supported way to change the distances.
"""

from dataclasses import dataclass
import math


def _round_up(value: float, step: float) -> float:
    """Round away from zero onto a grid, so rounding never shortens a margin."""
    # The epsilon keeps a value already on the grid from being pushed up a
    # whole step by its own floating-point representation.
    return round(math.ceil(value / step - 1e-9) * step, 6)


@dataclass(frozen=True)
class BaseDynamics:
    """Measured (or, for now, assumed) properties the distances derive from."""

    # Fastest the base is allowed to travel, m/s.
    max_speed: float
    # Deceleration the base can actually achieve on the test floor, m/s^2.
    # This is a property of mass, traction, and the drive, not a setting.
    max_deceleration: float
    # Worst-case age of a scan when it is acted on, s.
    sensor_period: float
    # Gap between safety gate evaluations, s.
    control_period: float
    # Command-to-torque lag in the driver and motor controller, s.
    actuation_lag: float
    # How far the robot sticks out ahead of the obstacle sensor, m. The gate
    # compares raw lidar ranges, so anything forward of the lidar is distance
    # the robot does not have.
    sensor_to_front_edge: float
    # Speed cap inside the caution zone, as a fraction of max_speed.
    caution_speed_scale: float
    # Multiplier on the computed travel, covering the errors these inputs do
    # not model: sensor noise, slope, wheel slip, and payload.
    safety_factor: float
    # Grid the published distances are rounded up onto, m.
    distance_step: float
    # False while the inputs are plausible guesses rather than measurements.
    measured: bool = False

    def __post_init__(self) -> None:
        positive = (
            "max_speed",
            "max_deceleration",
            "sensor_period",
            "control_period",
            "safety_factor",
            "distance_step",
        )
        for field in positive:
            value = getattr(self, field)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field} must be finite and positive")

        for field in ("actuation_lag", "sensor_to_front_edge"):
            value = getattr(self, field)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field} must be finite and non-negative")

        if not 0.0 <= self.caution_speed_scale < 1.0:
            raise ValueError("caution_speed_scale must be in [0.0, 1.0)")
        if self.safety_factor < 1.0:
            raise ValueError("safety_factor must be at least 1.0")

    @property
    def reaction_time(self) -> float:
        """Seconds between an obstacle appearing and the wheels responding."""
        return self.sensor_period + self.control_period + self.actuation_lag

    @property
    def caution_speed(self) -> float:
        return self.max_speed * self.caution_speed_scale

    @classmethod
    def from_mapping(cls, values: dict) -> "BaseDynamics":
        """Build from a parsed YAML mapping, rejecting unknown keys.

        A typo in the config would otherwise be silently ignored and leave the
        default in place, which is the failure mode this module exists to stop.
        """
        known = set(cls.__dataclass_fields__)
        unknown = sorted(set(values) - known)
        if unknown:
            raise ValueError(f"unknown base dynamics keys: {', '.join(unknown)}")

        missing = sorted(known - set(values) - {"measured"})
        if missing:
            raise ValueError(f"missing base dynamics keys: {', '.join(missing)}")

        return cls(**values)


@dataclass(frozen=True)
class SafetyDistances:
    """Distances the safety gate should be configured with."""

    stop_distance: float
    caution_distance: float
    reaction_distance: float
    braking_distance: float


def derive_safety_distances(dynamics: BaseDynamics) -> SafetyDistances:
    """Compute the gate distances implied by a base's geometry and dynamics."""
    reaction_distance = dynamics.max_speed * dynamics.reaction_time
    braking_distance = dynamics.max_speed**2 / (2.0 * dynamics.max_deceleration)

    # Stop before the robot's nose touches the obstacle, not before its lidar
    # reaches it.
    stop_distance = _round_up(
        dynamics.sensor_to_front_edge
        + dynamics.safety_factor * (reaction_distance + braking_distance),
        dynamics.distance_step,
    )

    # By the time the robot reaches stop_distance it must already be down to
    # caution speed, so the caution zone has to be deep enough to shed the
    # difference.
    slowing_distance = (
        dynamics.max_speed**2 - dynamics.caution_speed**2
    ) / (2.0 * dynamics.max_deceleration)
    caution_distance = _round_up(
        stop_distance
        + dynamics.safety_factor * (reaction_distance + slowing_distance),
        dynamics.distance_step,
    )

    return SafetyDistances(
        stop_distance=stop_distance,
        caution_distance=caution_distance,
        reaction_distance=reaction_distance,
        braking_distance=braking_distance,
    )
