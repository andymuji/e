import unittest

from robot_console.goal_dispatcher import (
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    RosGoalDispatcher,
)
from robot_core import NavigationGoal, Pose2D


class FakeFuture:
    """A future with no executor: the test decides when the answer arrives."""

    def __init__(self, value=None, error: Exception | None = None) -> None:
        self.value = value
        self.error = error
        self._callback = None

    def add_done_callback(self, callback) -> None:
        self._callback = callback

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value

    def answer(self) -> None:
        self._callback(self)


class FakeResult:
    def __init__(self, status: int) -> None:
        self.status = status


class FakeGoalHandle:
    def __init__(self, accepted: bool = True) -> None:
        self.accepted = accepted
        self.cancelled = False
        self.result_future = FakeFuture(FakeResult(STATUS_SUCCEEDED))

    def cancel_goal_async(self):
        self.cancelled = True
        return FakeFuture(None)

    def get_result_async(self):
        return self.result_future


class FakeNavigator:
    """The Nav2 action client, minus Nav2."""

    def __init__(self, ready: bool = True, handle: FakeGoalHandle | None = None) -> None:
        self.ready = ready
        self.handle = handle if handle is not None else FakeGoalHandle()
        self.sent = []
        self.response_future = FakeFuture(self.handle)

    def server_is_ready(self) -> bool:
        return self.ready

    def send_goal_async(self, message):
        self.sent.append(message)
        return self.response_future


def goal(name: str = "kitchen") -> NavigationGoal:
    return NavigationGoal(name, Pose2D(1.0, 2.0, 0.0))


class GoalDispatcherTests(unittest.TestCase):
    def setUp(self) -> None:
        self.navigator = FakeNavigator()
        self.dispatcher = RosGoalDispatcher(self.navigator, lambda item: item)

    def test_starts_ready(self) -> None:
        self.assertEqual(self.dispatcher.status().state, "ready")

    def test_a_goal_with_no_action_server_is_refused_not_reported_as_sent(self) -> None:
        # The failure this guards: a goal sent to an absent server disappears
        # silently, and the console shows a trip that is not happening.
        self.navigator.ready = False

        with self.assertRaises(RuntimeError) as caught:
            self.dispatcher.send_goal(goal())

        self.assertIn("navigation is not running", str(caught.exception))
        self.assertEqual(self.dispatcher.status().state, "ready")
        self.assertEqual(self.navigator.sent, [])

    def test_an_accepted_goal_is_navigating(self) -> None:
        goal_id = self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()

        status = self.dispatcher.status()
        self.assertEqual(status.state, "navigating")
        self.assertEqual(status.goal_id, goal_id)
        self.assertEqual(status.location_name, "kitchen")

    def test_a_rejected_goal_is_not_a_trip(self) -> None:
        self.navigator.handle.accepted = False

        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()

        status = self.dispatcher.status()
        self.assertEqual(status.state, "rejected")
        self.assertIsNone(status.goal_id)

    def test_a_send_that_raises_ends_the_trip(self) -> None:
        self.navigator.response_future = FakeFuture(error=RuntimeError("no server"))

        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()

        self.assertEqual(self.dispatcher.status().state, "aborted")

    def test_arrival_is_reported(self) -> None:
        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()
        self.navigator.handle.result_future.answer()

        self.assertEqual(self.dispatcher.status().state, "succeeded")

    def test_an_aborted_trip_is_not_an_arrival(self) -> None:
        self.navigator.handle.result_future = FakeFuture(FakeResult(STATUS_ABORTED))

        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()
        self.navigator.handle.result_future.answer()

        self.assertEqual(self.dispatcher.status().state, "aborted")

    def test_an_unrecognised_end_state_is_not_an_arrival(self) -> None:
        # Fail closed: a status this console cannot read says nothing about
        # whether the robot got there.
        self.navigator.handle.result_future = FakeFuture(FakeResult(99))

        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()
        self.navigator.handle.result_future.answer()

        self.assertEqual(self.dispatcher.status().state, "aborted")

    def test_cancelling_an_accepted_goal_cancels_it(self) -> None:
        self.navigator.handle.result_future = FakeFuture(FakeResult(STATUS_CANCELED))
        goal_id = self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()

        self.dispatcher.cancel_goal(goal_id)

        self.assertTrue(self.navigator.handle.cancelled)
        self.assertEqual(self.dispatcher.status().state, "cancelling")

        self.navigator.handle.result_future.answer()
        self.assertEqual(self.dispatcher.status().state, "cancelled")

    def test_cancelling_before_nav2_answers_still_drops_the_trip(self) -> None:
        # A cancel can arrive while Nav2 is still accepting the goal. Without
        # this the robot drives off after the operator pressed cancel.
        goal_id = self.dispatcher.send_goal(goal())

        self.dispatcher.cancel_goal(goal_id)
        self.assertEqual(self.dispatcher.status().state, "cancelled")

        self.navigator.response_future.answer()
        self.assertTrue(self.navigator.handle.cancelled)
        self.assertEqual(self.dispatcher.status().state, "cancelled")

    def test_cancelling_an_unknown_goal_is_refused(self) -> None:
        self.dispatcher.send_goal(goal())

        with self.assertRaises(ValueError):
            self.dispatcher.cancel_goal("not-the-goal-id")

    def test_one_trip_at_a_time(self) -> None:
        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()

        with self.assertRaises(RuntimeError):
            self.dispatcher.send_goal(goal("bedroom"))

    def test_a_finished_trip_does_not_block_the_next_one(self) -> None:
        self.dispatcher.send_goal(goal())
        self.navigator.response_future.answer()
        self.navigator.handle.result_future.answer()

        self.navigator.handle = FakeGoalHandle()
        self.navigator.response_future = FakeFuture(self.navigator.handle)
        self.dispatcher.send_goal(goal("bedroom"))

        self.assertEqual(self.dispatcher.status().state, "navigating")

    def test_a_late_answer_does_not_overwrite_a_newer_trip(self) -> None:
        stale = self.navigator.response_future
        goal_id = self.dispatcher.send_goal(goal())
        self.dispatcher.cancel_goal(goal_id)

        self.navigator.handle = FakeGoalHandle()
        self.navigator.response_future = FakeFuture(self.navigator.handle)
        new_goal_id = self.dispatcher.send_goal(goal("bedroom"))
        stale.answer()

        status = self.dispatcher.status()
        self.assertEqual(status.state, "navigating")
        self.assertEqual(status.goal_id, new_goal_id)


class ActionStatusConstantTests(unittest.TestCase):
    """The action status numbers are spelled out; check them against ROS."""

    def test_they_match_action_msgs(self) -> None:
        try:
            from action_msgs.msg import GoalStatus
        except ImportError:
            self.skipTest("ROS 2 is not installed in this environment")

        self.assertEqual(STATUS_SUCCEEDED, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(STATUS_CANCELED, GoalStatus.STATUS_CANCELED)
        self.assertEqual(STATUS_ABORTED, GoalStatus.STATUS_ABORTED)


if __name__ == "__main__":
    unittest.main()
