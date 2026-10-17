"""What the base driver tells the wheels, decided without ROS.

The driver is the hardware consumer at the end of the motion path:

    /cmd_vel_requested -> robot_safety -> /cmd_vel -> this driver -> Pico

It has no authority of its own. It turns the gate's output into wheel
commands and refuses to turn anything else into motion. It commands zero
whenever:

- nothing has arrived on /cmd_vel yet, or the last message is older than
  command_timeout. The gate publishes every cycle, stop or go, so silence
  means the gate has died, not that it wants the last speed held;
- the message holds something that is not a finite number;
- anything but exactly one safety gate is publishing /cmd_vel. AGENTS.md
  allows only the gate there; a teleop started without its remap, or a
  second gate, would otherwise reach the wheels unchecked or fight the one
  that is trying to stop.
"""

from dataclasses import dataclass
import math

from robot_base.kinematics import MecanumGeometry, limit_body, wheel_commands

STOPPED = (0, 0, 0, 0)


@dataclass(frozen=True)
class DriveLimits:
    """The fastest the driver will ever ask for, whatever /cmd_vel says.

    max_linear_speed must not exceed max_speed in base_dynamics.yaml: the
    gate's stop distance is derived assuming the base never goes faster.
    robot_bringup's tests hold base.yaml to that.
    """

    max_linear_speed: float
    max_angular_speed: float
    # Wheel speed at full motor voltage, rad/s. Guessing HIGH is the safe
    # side: the real wheels then turn slower than asked, never faster.
    max_wheel_speed: float

    def __post_init__(self) -> None:
        for name in ("max_linear_speed", "max_angular_speed", "max_wheel_speed"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")


@dataclass(frozen=True)
class DriveDecision:
    commands: tuple[int, ...]
    reason: str


def wheel_topic_problem(
    publisher_names: list[str], gate_name: str
) -> str | None:
    """Why the publishers on /cmd_vel cannot be trusted, or None if they can."""
    if not publisher_names:
        return "nothing publishes cmd_vel: the safety gate is not running"
    others = sorted({name for name in publisher_names if name != gate_name})
    if others:
        return (
            f"{', '.join(others)} publishes cmd_vel directly; only "
            f"{gate_name} may. Remap it to cmd_vel_requested."
        )
    if len(publisher_names) > 1:
        return f"{len(publisher_names)} safety gates publish cmd_vel; there must be one"
    return None


class BaseController:
    def __init__(
        self,
        geometry: MecanumGeometry,
        limits: DriveLimits,
        command_timeout: float,
        allow_sideways: bool = True,
        max_step: int | None = None,
    ) -> None:
        """`allow_sideways=False` drives the base like a tank: vy is dropped.

        `max_step` caps how much any wheel's command may GROW per decision,
        in thousandths of full power, so the four motors do not all draw
        their starting current at once from a battery that allows 10 A.
        Slowing and stopping are never ramped: a stop takes effect at once.
        """
        if not math.isfinite(command_timeout) or command_timeout <= 0.0:
            raise ValueError("command_timeout must be finite and greater than zero")
        if max_step is not None and max_step <= 0:
            raise ValueError("max_step must be positive")
        self.geometry = geometry
        self.limits = limits
        self.command_timeout = command_timeout
        self.allow_sideways = allow_sideways
        self.max_step = max_step
        self._command: tuple[float, float, float] | None = None
        self._command_time: float | None = None
        self._sent = STOPPED

    def on_command(self, vx: float, vy: float, wz: float, now: float) -> None:
        """Take one /cmd_vel message. An unreadable one is kept as None."""
        values = (vx, vy, wz)
        self._command = values if all(math.isfinite(v) for v in values) else None
        self._command_time = now

    def decide(self, now: float, topic_problem: str | None = None) -> DriveDecision:
        decision = self._target(now, topic_problem)
        self._sent = tuple(
            self._ramp(sent, wanted)
            for sent, wanted in zip(self._sent, decision.commands, strict=True)
        )
        return DriveDecision(self._sent, decision.reason)

    def _ramp(self, sent: int, wanted: int) -> int:
        """Grow a wheel's command by at most max_step; shrink it at once.

        A reversal goes through zero first, so it too starts from rest.
        """
        if self.max_step is None or (abs(wanted) <= abs(sent) and sent * wanted >= 0):
            return wanted
        if sent * wanted < 0:
            return 0
        step = min(self.max_step, abs(wanted) - abs(sent))
        return sent + step if wanted > 0 else sent - step

    def _target(self, now: float, topic_problem: str | None) -> DriveDecision:
        if topic_problem is not None:
            return DriveDecision(STOPPED, topic_problem)
        if self._command_time is None:
            return DriveDecision(STOPPED, "no motion command received")

        age = now - self._command_time
        if not 0.0 <= age <= self.command_timeout:
            # Dropped, so a stream that resumes cannot replay it.
            self._command = None
            self._command_time = None
            return DriveDecision(STOPPED, "motion command timed out")
        if self._command is None:
            return DriveDecision(STOPPED, "invalid motion command")

        vx, vy, wz = self._command
        if not self.allow_sideways:
            vy = 0.0
        vx, vy, wz = limit_body(
            vx, vy, wz, self.limits.max_linear_speed, self.limits.max_angular_speed
        )
        commands = wheel_commands(
            self.geometry.inverse(vx, vy, wz), self.limits.max_wheel_speed
        )
        return DriveDecision(commands, "driving" if any(commands) else "stopped")
