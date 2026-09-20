"""Show the real safety gate's state, and work its latch from the console.

SAFETY: engaging publishes `true` on `emergency_stop`. Releasing publishes
`true` on `emergency_stop_reset`; this adapter never publishes `false` on
`emergency_stop`, because a `false` there does not release the latch and
must never be mistaken for the thing that does.

What the console displays comes from the gate's own `safety_state` topic, not
from a controller of its own. A second controller would agree with the gate
only by luck, and the moment it disagreed the console would be reporting a
robot that does not exist.

Liveness is the harder half. `safety_state` is published on change, so on a
settled robot it is silent for minutes at a time and its age says nothing.
The gate's velocity output is what runs every control cycle, so that is the
heartbeat this adapter times out on: no `cmd_vel` for `timeout` seconds means
the gate's control loop has stopped and the state on screen is worth nothing.
Subscribing is all this does with that topic; publishing to it belongs to the
gate alone.
"""

from collections.abc import Callable
from threading import Lock
from typing import Any

# The states the gate publishes. Anything else is a gate this console does not
# understand, which is reported as unknown rather than guessed at.
KNOWN_STATES = ("clear", "caution", "stop")

# robot_safety.SafetyController's reason when the latch is down. It checks the
# latch before everything else, so this reason and only this reason means
# engaged - any other stop reason means the latch is up and something else
# stopped the robot.
EMERGENCY_STOP_REASON = "emergency stop active"

UNKNOWN_REFUSAL = "the safety gate has not reported its state"


def parse_state(text: str) -> tuple[str, str]:
    """Split the gate's "state: reason" line, failing closed on anything else.

    A line this console cannot read is not permission to show a clear light,
    so it becomes "unknown" rather than a best guess at the state.
    """
    state, separator, reason = text.partition(":")
    state = state.strip().lower()
    if not separator or state not in KNOWN_STATES:
        return "unknown", f"the safety gate said something unreadable: {text!r}"
    return state, reason.strip()


class RosSafetyAdapter:
    """The console's half of the gate: publish the latch, read the state."""

    def __init__(
        self,
        stop_publisher: Any,
        reset_publisher: Any,
        bool_message: Callable[[bool], Any],
        clock: Callable[[], float],
        timeout: float = 2.0,
        logger: Any | None = None,
    ) -> None:
        # The publishers and the message factory are injected so this class
        # can be exercised without a ROS graph; clock() is the node clock in
        # seconds, so use_sim_time governs the timeout as it does in the gate.
        if not timeout > 0.0:
            raise ValueError("timeout must be greater than zero")
        self._stop_publisher = stop_publisher
        self._reset_publisher = reset_publisher
        self._bool_message = bool_message
        self._clock = clock
        self._timeout = timeout
        self._logger = logger
        self._lock = Lock()
        self._state: str | None = None
        self._reason = ""
        self._state_time: float | None = None
        self._heartbeat_time: float | None = None

    def engage_emergency_stop(self) -> None:
        """Latch the gate. Only an operator reset releases it again."""
        self._stop_publisher.publish(self._bool_message(True))

    def reset_emergency_stop(self) -> None:
        """Release the latch, on the one topic that can."""
        self._reset_publisher.publish(self._bool_message(True))
        if self._logger is not None:
            self._logger.warning("software emergency stop reset from the console")

    def on_state(self, text: str) -> None:
        """Record what the gate last said about itself."""
        state, reason = parse_state(text)
        with self._lock:
            self._state = state
            self._reason = reason
            self._state_time = self._clock()

    def on_heartbeat(self) -> None:
        """Note that the gate's control loop ran, whatever it decided."""
        with self._lock:
            self._heartbeat_time = self._clock()

    @property
    def emergency_stop_engaged(self) -> bool:
        """True unless the gate has positively said the latch is released."""
        return self.status()["emergency_stop"] is not False

    def motion_refusal(self) -> str | None:
        status = self.status()
        if status["emergency_stop"] is None:
            return UNKNOWN_REFUSAL
        if status["emergency_stop"]:
            return "the emergency stop is engaged"
        return None

    def status(self) -> dict[str, Any]:
        with self._lock:
            state, reason = self._state, self._reason
            state_age = self._age(self._state_time)
            heartbeat_age = self._age(self._heartbeat_time)

        if heartbeat_age is None:
            return self._unknown("the safety gate is not running", state_age)
        if heartbeat_age < 0.0 or heartbeat_age > self._timeout:
            # A negative age is a clock that jumped backwards, which is no more
            # a reason to trust the reading than an old one is.
            return self._unknown(
                f"no word from the safety gate for {heartbeat_age:.1f} s", state_age
            )
        if state is None:
            # Told apart from a gate that is not there at all, because the two
            # need different things done about them. A console opened after
            # the robot settled lands here: safety_state is published on
            # change, so there has been nothing to hear since it started.
            return self._unknown(
                "the safety gate is running but has not said what it is doing "
                "yet: it reports only when its state changes",
                state_age,
            )
        if state == "unknown":
            # A line this console could not read says nothing about the latch,
            # and "nothing" must not render as released.
            return self._unknown(reason, state_age)

        return {
            "state": state,
            # The gate does not publish its speed scale, and a number this
            # console worked out itself would be its own opinion, not the
            # robot's.
            "speed_scale": None,
            "reason": reason,
            "nearest_obstacle_distance": None,
            "reading_age": None,
            "command_age": None,
            "safety_state_age": state_age,
            "emergency_stop": reason == EMERGENCY_STOP_REASON,
        }

    def _unknown(self, reason: str, state_age: float | None) -> dict[str, Any]:
        """Report a gate this console cannot see as stopped and unknown.

        The latch is None rather than False: the console has not been told it
        is released, and saying released would invite someone to walk up to a
        robot whose stop may still be down.
        """
        return {
            "state": "unknown",
            "speed_scale": None,
            "reason": reason,
            "nearest_obstacle_distance": None,
            "reading_age": None,
            "command_age": None,
            "safety_state_age": state_age,
            "emergency_stop": None,
        }

    def _age(self, timestamp: float | None) -> float | None:
        return None if timestamp is None else self._clock() - timestamp
