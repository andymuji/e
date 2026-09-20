"""Map the Nav2 NavigateToPose action onto the console's goal contract.

SAFETY: asking Nav2 for a goal is the only way this console causes motion. It
publishes no velocity; Nav2's velocity output is remapped to
cmd_vel_requested, so the gate still decides what reaches the wheels.

Nothing here blocks. Every call the console makes arrives on an HTTP thread
and every answer from Nav2 arrives on an executor thread, so the trip state is
guarded by one lock and the action client is only ever asked for futures.
"""

from collections.abc import Callable
from dataclasses import replace
from threading import RLock
from typing import Any
import uuid

from robot_console.web_console import GoalStatus

# action_msgs/msg/GoalStatus, whose values are fixed by the action protocol.
# Spelled out rather than imported so this module stays importable, and
# testable, without ROS; test_goal_dispatcher checks them against the real
# message when rclpy is installed.
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

_RESULT_STATES: dict[int, tuple[str, str]] = {
    STATUS_SUCCEEDED: ("succeeded", "Arrived."),
    STATUS_CANCELED: ("cancelled", "Trip cancelled"),
    STATUS_ABORTED: ("aborted", "Navigation could not finish the trip."),
}

# States in which a trip is still the robot's business: a second goal is
# refused and a cancel is accepted.
_RUNNING = ("navigating", "cancelling")


class RosGoalDispatcher:
    """Drive one trip at a time through a NavigateToPose action client."""

    def __init__(
        self,
        navigator: Any,
        goal_factory: Callable[[Any], Any],
        logger: Any | None = None,
    ) -> None:
        # navigator is an rclpy ActionClient, or anything offering
        # server_is_ready() and send_goal_async(); goal_factory turns a
        # NavigationGoal into the action's goal message. Both are injected so
        # the translation below can be tested without a ROS graph.
        self._navigator = navigator
        self._goal_factory = goal_factory
        self._logger = logger
        # Reentrant: the response callback can settle a trip while already
        # holding the lock to decide that it should.
        self._lock = RLock()
        self._status = GoalStatus("ready", message="Robot is ready")
        self._goal_handle: Any | None = None
        # A cancel can arrive while Nav2 is still accepting the goal, so this
        # flag, not the handle, decides whether the trip survives.
        self._trip_wanted = False

    def send_goal(self, goal: Any) -> str:
        with self._lock:
            if self._status.state in _RUNNING:
                raise RuntimeError("robot is already navigating")
            if not self._navigator.server_is_ready():
                # A goal sent to an absent server is dropped silently, which
                # would show in the console as a trip under way.
                raise RuntimeError("navigation is not running: the goal was not sent")

            goal_id = uuid.uuid4().hex
            self._goal_handle = None
            self._trip_wanted = True
            self._status = GoalStatus(
                "navigating",
                location_name=goal.location_name,
                goal_id=goal_id,
                message=f"Going to {goal.location_name}",
            )

        future = self._navigator.send_goal_async(self._goal_factory(goal))
        future.add_done_callback(lambda done: self._on_goal_response(goal_id, done))
        return goal_id

    def cancel_goal(self, goal_id: str) -> None:
        with self._lock:
            if self._status.goal_id != goal_id or self._status.state not in _RUNNING:
                raise ValueError("no matching active goal")

            self._trip_wanted = False
            handle = self._goal_handle
            if handle is None:
                # Nav2 has not answered the send yet. The trip never started,
                # and _on_goal_response cancels the goal if it is accepted.
                self._settle(goal_id, "cancelled", "Trip cancelled")
                return
            self._status = replace(
                self._status, state="cancelling", message="Cancelling the trip"
            )

        handle.cancel_goal_async()

    def status(self) -> GoalStatus:
        with self._lock:
            return self._status

    def _on_goal_response(self, goal_id: str, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            # Reported and turned into a stopped trip, never swallowed: a
            # failed send that left the state at "navigating" would show in
            # the console as a robot on its way somewhere.
            self._log(f"navigation did not answer the goal: {error}")
            self._settle(goal_id, "aborted", "Navigation did not answer.")
            return

        if not handle.accepted:
            self._log("navigation refused the goal")
            self._settle(goal_id, "rejected", "Navigation would not take that trip.")
            return

        with self._lock:
            # Superseded or already cancelled: let go of the goal rather than
            # leaving Nav2 driving towards somewhere nobody is watching.
            if self._status.goal_id != goal_id or not self._trip_wanted:
                handle.cancel_goal_async()
                return
            self._goal_handle = handle

        handle.get_result_async().add_done_callback(
            lambda done: self._on_result(goal_id, done)
        )

    def _on_result(self, goal_id: str, future: Any) -> None:
        try:
            result = future.result()
        except Exception as error:
            self._log(f"navigation stopped answering: {error}")
            self._settle(goal_id, "aborted", "Navigation stopped answering.")
            return

        state, message = _RESULT_STATES.get(
            getattr(result, "status", None),
            # An end state this console does not recognise is not an arrival.
            ("aborted", "Navigation ended in a state this console cannot read."),
        )
        self._settle(goal_id, state, message)

    def _settle(self, goal_id: str, state: str, message: str) -> None:
        """Record how a trip ended, unless a newer one has taken over."""
        with self._lock:
            if self._status.goal_id != goal_id:
                return
            self._status = GoalStatus(
                state, location_name=self._status.location_name, message=message
            )
            self._goal_handle = None
            self._trip_wanted = False

    def _log(self, message: str) -> None:
        if self._logger is not None:
            self._logger.warning(message)
