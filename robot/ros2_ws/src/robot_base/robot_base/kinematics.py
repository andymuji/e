"""Mecanum wheel kinematics: body velocity to wheel speeds, and back.

Wheel order everywhere is front-left, front-right, rear-left, rear-right.
The rollers are assumed in the usual "X" layout seen from above (each
wheel's rollers point towards the centre), which is what lets the base move
sideways. x is forward, y is left, z is up, rotation is anticlockwise.

    w_fl = (vx - vy - k*wz) / r        vx = r/4       * ( w_fl + w_fr + w_rl + w_rr)
    w_fr = (vx + vy + k*wz) / r        vy = r/4       * (-w_fl + w_fr + w_rl - w_rr)
    w_rl = (vx + vy - k*wz) / r        wz = r/(4k)    * (-w_fl + w_fr - w_rl + w_rr)
    w_rr = (vx - vy + k*wz) / r

with r the wheel radius and k = half_wheelbase + half_track.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math

from robot_base.protocol import FULL_SCALE


def _positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius: float
    # Centre of the base to the front axle, m.
    half_wheelbase: float
    # Centre of the base to the middle of a wheel, sideways, m.
    half_track: float

    def __post_init__(self) -> None:
        _positive("wheel_radius", self.wheel_radius)
        _positive("half_wheelbase", self.half_wheelbase)
        _positive("half_track", self.half_track)

    @property
    def _k(self) -> float:
        return self.half_wheelbase + self.half_track

    def inverse(self, vx: float, vy: float, wz: float) -> tuple[float, ...]:
        """Wheel speeds in rad/s for a body velocity."""
        r, k = self.wheel_radius, self._k
        return (
            (vx - vy - k * wz) / r,
            (vx + vy + k * wz) / r,
            (vx + vy - k * wz) / r,
            (vx - vy + k * wz) / r,
        )

    def forward(self, wheels: Sequence[float]) -> tuple[float, float, float]:
        """Body velocity (vx, vy, wz) from wheel speeds, or displacement
        from wheel angle changes: the relation is linear either way."""
        fl, fr, rl, rr = wheels
        r, k = self.wheel_radius, self._k
        return (
            r / 4.0 * (fl + fr + rl + rr),
            r / 4.0 * (-fl + fr + rl - rr),
            r / (4.0 * k) * (-fl + fr - rl + rr),
        )


def limit_body(
    vx: float, vy: float, wz: float, max_linear: float, max_angular: float
) -> tuple[float, float, float]:
    """Cap the speed in any direction, and the turn rate, keeping direction.

    The cap is on the length of (vx, vy), not on each component: a mecanum
    base asked for full speed forward and full speed sideways at once would
    otherwise go 1.4 times faster than the limit, diagonally.
    """
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        vx, vy = vx * max_linear / speed, vy * max_linear / speed
    wz = max(-max_angular, min(max_angular, wz))
    return vx, vy, wz


def wheel_commands(
    wheel_speeds: Sequence[float], max_wheel_speed: float
) -> tuple[int, ...]:
    """Wheel speeds in rad/s as the Pico's -1000..1000 commands.

    If any wheel would need more than it can give, all four are scaled down
    together. Clipping one wheel alone would change the direction the base
    moves in, not just its speed.
    """
    _positive("max_wheel_speed", max_wheel_speed)
    fractions = [speed / max_wheel_speed for speed in wheel_speeds]
    largest = max(abs(fraction) for fraction in fractions)
    if largest > 1.0:
        fractions = [fraction / largest for fraction in fractions]
    return tuple(
        max(-FULL_SCALE, min(FULL_SCALE, round(fraction * FULL_SCALE)))
        for fraction in fractions
    )


class OdometryIntegrator:
    """Where the wheels say the base has gone since start-up.

    Mecanum rollers slip, sideways most of all, so this drifts faster than
    ordinary wheel odometry. Cartographer does not use it on the car today;
    it is published so it can be compared with the lidar's answer.
    """

    def __init__(self, geometry: MecanumGeometry) -> None:
        self.geometry = geometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(
        self, wheel_angles: Sequence[float], dt: float
    ) -> tuple[float, float, float]:
        """Integrate one step of wheel rotation (rad). Returns body velocity."""
        dx, dy, dtheta = self.geometry.forward(wheel_angles)
        # Rotate the body-frame step into the odometry frame at the mid-step
        # heading, which is exact for a constant-velocity arc to second order.
        heading = self.theta + dtheta / 2.0
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        self.x += dx * cos_h - dy * sin_h
        self.y += dx * sin_h + dy * cos_h
        self.theta = math.atan2(
            math.sin(self.theta + dtheta), math.cos(self.theta + dtheta)
        )
        return dx / dt, dy / dt, dtheta / dt
