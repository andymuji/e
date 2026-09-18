"""Derive Nav2's costmap geometry from the base dimensions.

Nav2 decides what counts as a collision from the footprint and the inflation
radius. Both are the robot's shape restated in another file, so hand-writing
them means the robot can be resized without the planner noticing: a base that
grew 10 cm plans through gaps it no longer fits.

Everything here comes from the URDF's `base_length` and `base_width`. The
bringup tests fail if the numbers shipped in `nav2.yaml` drift away from what
those dimensions imply.
"""

from dataclasses import dataclass
import math


def _round_up(value: float, step: float) -> float:
    """Round away from zero onto a grid, so rounding never shrinks a margin."""
    return round(math.ceil(value / step - 1e-9) * step, 6)


@dataclass(frozen=True)
class BaseFootprint:
    """The robot's rectangular extent, as the planner needs to see it."""

    # From the URDF, m.
    length: float
    width: float
    # Clearance the planner keeps beyond the robot's own outline, m. This is
    # not a safety margin: robot_safety is the thing that stops the robot.
    # It is how much room the planner leaves so the gate is not constantly
    # having to.
    clearance_margin: float
    # Grid the published radii are rounded up onto, m.
    distance_step: float = 0.05

    def __post_init__(self) -> None:
        for field in ("length", "width", "distance_step"):
            value = getattr(self, field)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field} must be finite and positive")
        if not math.isfinite(self.clearance_margin) or self.clearance_margin < 0.0:
            raise ValueError("clearance_margin must be finite and non-negative")

    @property
    def polygon(self) -> list[tuple[float, float]]:
        """Corners in base_link, clockwise from the front left.

        In base_link's x-forward, y-left frame this order walks front left,
        front right, rear right, rear left, which is clockwise. The order is
        restated in `nav2.yaml` and compared corner by corner by the bringup
        tests, so it is fixed: change it and that comparison fails.

        Centred on base_link because that is where the URDF puts the chassis.
        A base whose wheels are not centred needs this offset to match, or
        the planner turns the robot through walls.
        """
        half_length = self.length / 2.0
        half_width = self.width / 2.0
        return [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ]

    @property
    def inscribed_radius(self) -> float:
        """Largest circle that fits inside the robot."""
        return min(self.length, self.width) / 2.0

    @property
    def circumscribed_radius(self) -> float:
        """Smallest circle that contains the robot, whatever way it is turned."""
        return math.hypot(self.length / 2.0, self.width / 2.0)

    @property
    def inflation_radius(self) -> float:
        """How far obstacle cost is spread out from an obstacle.

        Anything less than the circumscribed radius lets the planner produce
        paths the robot cannot rotate on the spot without hitting something.
        """
        return _round_up(
            self.circumscribed_radius + self.clearance_margin, self.distance_step
        )

    def as_costmap_footprint(self) -> str:
        """The footprint in the string-of-pairs form Nav2 parameters take.

        Enough significant digits to reproduce the polygon exactly: this
        string is what gets pasted into `nav2.yaml`, and the bringup tests
        compare it back against these dimensions to 6 decimal places. At 4
        significant digits a measured 0.4123 m base emitted a half-length of
        0.2061 instead of 0.20615 - a footprint 0.05 mm smaller than the
        robot, and a drift check that fails for no visible reason.
        """
        pairs = ", ".join(f"[{x:.10g}, {y:.10g}]" for x, y in self.polygon)
        return f"[{pairs}]"
