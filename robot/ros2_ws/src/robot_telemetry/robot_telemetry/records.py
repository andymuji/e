"""What a recorded run looks like once the ROS types are stripped off it.

The invariant checks in `invariants` are the part that has to be trusted, so
they are kept free of rosbag2, rclpy, and message classes: they read a plain
sequence of `Record`s. Everything ROS-shaped stops here, in `convert`, which
needs nothing from a message but its fields. That is what lets the checks be
tested against handwritten runs, on a machine with no ROS and no robot.
"""

from dataclasses import dataclass
import math
from typing import Any

# The topics a run is judged on. `record.launch.py` records these and a few
# more (tf, map, battery) that are context for a human rather than inputs to
# a check.
CMD_VEL = "/cmd_vel"
CMD_VEL_REQUESTED = "/cmd_vel_requested"
SAFETY_STATE = "/safety_state"
SCAN = "/scan"
EMERGENCY_STOP = "/emergency_stop"
EMERGENCY_STOP_RESET = "/emergency_stop_reset"

# Velocities arrive as floats that have been through a multiplication by a
# speed scale, so an exact == 0.0 would eventually call a rounding artefact
# motion. A millimetre per second is far below anything the base can execute.
ZERO_SPEED_TOLERANCE = 1e-6


@dataclass(frozen=True)
class Velocity:
    """One Twist, flattened. Only whether it is zero ever matters here."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    @property
    def components(self) -> tuple[float, ...]:
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )

    @property
    def magnitude(self) -> float:
        """Largest component, which is what "is the robot moving" turns on.

        A non-finite component reports as infinite rather than propagating a
        NaN, because `max` over a NaN depends on argument order and would let
        an uninterpretable command compare as stopped.
        """
        if any(not math.isfinite(component) for component in self.components):
            return math.inf
        return max(abs(component) for component in self.components)

    def is_zero(self, tolerance: float = ZERO_SPEED_TOLERANCE) -> bool:
        return self.magnitude <= tolerance


@dataclass(frozen=True)
class SafetyStatus:
    """One `/safety_state` message: what the gate decided, and why."""

    state: str
    reason: str

    @classmethod
    def parse(cls, text: str) -> "SafetyStatus":
        """Split the gate's "state: reason" string.

        A message that does not carry a reason is kept rather than dropped:
        an unreadable status is still evidence about what the gate was doing.
        """
        state, separator, reason = text.partition(":")
        if not separator:
            return cls(state.strip(), "")
        return cls(state.strip(), reason.strip())

    @property
    def stopped(self) -> bool:
        return self.state == "stop"

    @property
    def emergency(self) -> bool:
        """Whether this stop is the latched software emergency stop."""
        return self.stopped and "emergency stop" in self.reason

    def __str__(self) -> str:
        return f"{self.state}: {self.reason}" if self.reason else self.state


@dataclass(frozen=True)
class Scan:
    """One laser scan, reduced to the nearest thing it saw."""

    nearest: float | None


@dataclass(frozen=True)
class Flag:
    """One boolean topic message: the stop and reset buttons."""

    value: bool


@dataclass(frozen=True)
class Opaque:
    """A recorded message no check reads, kept so the counts stay honest."""

    kind: str


@dataclass(frozen=True)
class Record:
    """One recorded message: when, where, and what."""

    timestamp: float
    topic: str
    message: Any


def canonical_topic(topic: str) -> str:
    """Give a topic a leading slash, so remapped and bare names compare equal."""
    return topic if topic.startswith("/") else "/" + topic


def nearest_range(message: Any) -> float | None:
    """The closest valid return in a scan, or None if it saw nothing usable.

    This mirrors the filter in `robot_safety`'s `_on_scan` rather than sharing
    it, because the two are doing different jobs: the gate is deciding, and
    this is reconstructing what the gate was looking at. If they ever disagree
    the report says so by measuring a latency the gate did not act on.
    """
    range_min = getattr(message, "range_min", -math.inf)
    range_max = getattr(message, "range_max", math.inf)
    valid = [
        float(distance)
        for distance in getattr(message, "ranges", ())
        if math.isfinite(distance) and range_min <= distance <= range_max
    ]
    return min(valid) if valid else None


def convert(message: Any) -> Any:
    """Turn a deserialised ROS message into one of the plain types above.

    Dispatch is on the shape of the message, not on the topic it arrived on:
    a remapped topic still carries a Twist, and a test can hand this function
    any object with the right fields without importing a message package.
    """
    if hasattr(message, "linear") and hasattr(message, "angular"):
        linear, angular = message.linear, message.angular
        return Velocity(
            float(linear.x),
            float(linear.y),
            float(linear.z),
            float(angular.x),
            float(angular.y),
            float(angular.z),
        )
    if hasattr(message, "ranges"):
        return Scan(nearest_range(message))
    if hasattr(message, "data"):
        data = message.data
        if isinstance(data, bool):
            return Flag(data)
        if isinstance(data, str):
            return SafetyStatus.parse(data)
    return Opaque(type(message).__name__)
