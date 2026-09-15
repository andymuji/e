import json
from pathlib import Path
import tempfile
import unittest

from robot_core import Pose2D
from robot_locations import LocationStore

from web_ui.app import DemoGoalDispatcher, RobotWebApp, SafetyScenarioAdapter


class RobotWebAppTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("Kitchen", Pose2D(1.0, 2.0))
        self.dispatcher = DemoGoalDispatcher()
        self.safety = SafetyScenarioAdapter()
        self.app = RobotWebApp(store, self.dispatcher, self.safety)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_lists_only_approved_locations(self) -> None:
        self.assertEqual(self.app.locations(), {"locations": [{"name": "kitchen"}]})

    def test_sends_named_goal_and_exposes_status(self) -> None:
        result = self.app.send_goal(" KITCHEN ")

        self.assertEqual(result["location_name"], "kitchen")
        self.assertEqual(self.app.status()["goal"]["state"], "navigating")
        self.assertEqual(self.app.status()["goal"]["location_name"], "kitchen")

    def test_rejects_unknown_location(self) -> None:
        with self.assertRaises(KeyError):
            self.app.send_goal("bedroom")

    def test_cancels_active_goal(self) -> None:
        self.app.send_goal("kitchen")

        self.assertEqual(self.app.cancel_goal(), {"message": "Trip cancelled"})
        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

    def test_only_one_trip_can_be_active(self) -> None:
        self.app.send_goal("kitchen")

        with self.assertRaises(RuntimeError):
            self.app.send_goal("kitchen")

    def test_reports_safety_scenario_decision(self) -> None:
        self.app.set_safety_scenario("caution")

        status = self.app.status()

        self.assertEqual(status["safety_state"], "caution")
        self.assertEqual(status["safety"]["speed_scale"], 0.35)
        self.assertEqual(status["safety"]["nearest_obstacle_distance"], 0.6)

    def test_emergency_stop_scenario_wins_over_clear_path(self) -> None:
        safety = self.app.set_safety_scenario("emergency_stop")

        self.assertEqual(safety["state"], "stop")
        self.assertEqual(safety["reason"], "emergency stop active")

    def test_rejects_unknown_safety_scenario(self) -> None:
        with self.assertRaises(ValueError):
            self.app.set_safety_scenario("drive_fast")

    def test_voice_command_sends_an_approved_destination(self) -> None:
        result = self.app.handle_voice_command("go to the kitchen")

        self.assertEqual(result["action"], "go_to")
        self.assertEqual(result["location_name"], "kitchen")
        self.assertEqual(self.app.status()["goal"]["state"], "navigating")

    def test_voice_command_refusal_never_starts_a_trip(self) -> None:
        result = self.app.handle_voice_command("go to the moon")

        self.assertEqual(result["action"], "refused")
        self.assertNotIn("goal_id", result)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_voice_stop_cancels_an_active_trip(self) -> None:
        self.app.send_goal("kitchen")

        result = self.app.handle_voice_command("robot please stop")

        self.assertEqual(result["action"], "stop")
        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

    def test_voice_stop_is_safe_when_nothing_is_moving(self) -> None:
        result = self.app.handle_voice_command("stop")

        self.assertEqual(result["action"], "stop")
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_voice_reports_the_current_trip(self) -> None:
        self.app.send_goal("kitchen")

        result = self.app.handle_voice_command("where are you")

        self.assertEqual(result["action"], "report_location")
        self.assertEqual(result["response"], "I am on my way to the kitchen.")


if __name__ == "__main__":
    unittest.main()