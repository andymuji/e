"""The console page driven through the ROS adapters, without a ROS graph.

The adapters are injected with fakes, so what is under test is the thing an
operator actually touches: the endpoints the page calls, answering with what
the robot said rather than with what a local controller decided.
"""

from pathlib import Path
import tempfile
import unittest

from robot_console.goal_dispatcher import RosGoalDispatcher
from robot_console.map_provider import RosMapProvider
from robot_console.safety_adapter import RosSafetyAdapter
from robot_console.web_console import RobotWebApp
from robot_core import Pose2D
from robot_locations import LocationStore


class FakePublisher:
    def __init__(self) -> None:
        self.published = []

    def publish(self, message) -> None:
        self.published.append(message)


class Clock:
    def __init__(self) -> None:
        self.seconds = 100.0

    def __call__(self) -> float:
        return self.seconds


class FakeFuture:
    """A future with no executor: the test decides when the answer arrives."""

    def __init__(self, value) -> None:
        self.value = value
        self._callback = None

    def add_done_callback(self, callback) -> None:
        self._callback = callback

    def result(self):
        return self.value

    def answer(self) -> None:
        self._callback(self)


class FakeGoalHandle:
    def __init__(self) -> None:
        self.accepted = True
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True
        return FakeFuture(None)

    def get_result_async(self):
        return FakeFuture(None)


class FakeNavigator:
    """The Nav2 action client, minus Nav2."""

    def __init__(self) -> None:
        self.handle = FakeGoalHandle()
        self.sent = []
        self.response_future = FakeFuture(self.handle)

    def server_is_ready(self) -> bool:
        return True

    def send_goal_async(self, message):
        self.sent.append(message)
        return self.response_future


class ConsoleAppTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("Kitchen", Pose2D(1.0, 2.0))

        self.navigator = FakeNavigator()
        self.clock = Clock()
        self.stop, self.reset = FakePublisher(), FakePublisher()
        self.safety = RosSafetyAdapter(
            self.stop, self.reset, lambda value: value, self.clock
        )
        self.app = RobotWebApp(
            store,
            RosGoalDispatcher(self.navigator, lambda item: item),
            safety=self.safety,
            map_provider=RosMapProvider(lambda: None),
        )

    def tearDown(self) -> None:
        self.directory.cleanup()

    def hear_gate(self, text: str = "clear: path clear") -> None:
        self.safety.on_state(text)
        self.safety.on_heartbeat()

    def test_a_goal_is_refused_while_the_gate_is_unseen(self) -> None:
        # Not because the latch is down, but because the console cannot say
        # that it is not. Sending anyway would look like a working trip.
        with self.assertRaises(RuntimeError) as caught:
            self.app.send_goal("kitchen")

        self.assertIn("has not reported its state", str(caught.exception))

    def test_a_goal_reaches_navigation_once_the_gate_is_heard(self) -> None:
        self.hear_gate()

        result = self.app.send_goal("kitchen")

        self.assertEqual(result["location_name"], "kitchen")
        self.assertEqual(len(self.navigator.sent), 1)

    def test_a_goal_is_refused_while_the_gate_reports_the_latch_down(self) -> None:
        self.hear_gate("stop: emergency stop active")

        with self.assertRaises(RuntimeError):
            self.app.send_goal("kitchen")

    def test_the_stop_button_publishes_true_on_emergency_stop(self) -> None:
        self.hear_gate()

        self.app.engage_emergency_stop()

        self.assertEqual(self.stop.published, [True])

    def test_the_reset_button_publishes_only_on_the_reset_topic(self) -> None:
        self.app.reset_emergency_stop()

        self.assertEqual(self.reset.published, [True])
        self.assertEqual(self.stop.published, [])

    def test_the_latch_readout_follows_the_gate_not_the_button(self) -> None:
        # Pressing stop does not make the console claim the latch is down.
        # The gate saying so is what does, which is the difference between
        # reporting the robot and reporting the console.
        self.hear_gate()
        self.app.engage_emergency_stop()

        self.assertFalse(self.app.status()["safety"]["emergency_stop"])

        self.hear_gate("stop: emergency stop active")
        self.assertTrue(self.app.status()["safety"]["emergency_stop"])

    def test_a_silent_gate_reports_unknown_rather_than_the_last_light(self) -> None:
        self.hear_gate()
        self.assertEqual(self.app.status()["safety_state"], "clear")

        self.clock.seconds += 30.0

        status = self.app.status()
        self.assertEqual(status["safety_state"], "unknown")
        self.assertIsNone(status["safety"]["emergency_stop"])

    def test_engaging_the_stop_abandons_the_trip_in_progress(self) -> None:
        self.hear_gate()
        self.app.send_goal("kitchen")
        self.navigator.response_future.answer()

        self.app.engage_emergency_stop()

        self.assertTrue(self.navigator.handle.cancelled)

    def test_the_demo_scenario_control_is_refused_against_a_real_gate(self) -> None:
        with self.assertRaises(ValueError):
            self.app.set_safety_scenario("clear")

    def test_the_map_says_what_is_missing_rather_than_drawing_a_room(self) -> None:
        payload = self.app.map_data()

        self.assertFalse(payload["available"])
        self.assertEqual(payload["walls"], [])
        self.assertIsNone(payload["robot"])
        self.assertEqual(payload["locations"][0]["name"], "kitchen")

    def test_a_voice_command_is_refused_while_the_gate_is_unseen(self) -> None:
        result = self.app.handle_voice_command("take me to the kitchen")

        self.assertEqual(result["action"], "refused")
        self.assertEqual(self.navigator.sent, [])


if __name__ == "__main__":
    unittest.main()
