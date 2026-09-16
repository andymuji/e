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
        self.assertEqual(
            self.app.locations(),
            {"locations": [{"name": "kitchen", "x": 1.0, "y": 2.0, "yaw": 0.0}]},
        )

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

    def test_emergency_stop_wins_over_a_clear_path(self) -> None:
        self.app.set_safety_scenario("clear")

        status = self.app.engage_emergency_stop()

        self.assertEqual(status["safety_state"], "stop")
        self.assertEqual(status["safety_message"], "emergency stop active")

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

    def test_emergency_stop_latch_survives_polls_and_scenario_changes(self) -> None:
        self.app.engage_emergency_stop()

        for _ in range(3):
            self.assertEqual(self.app.status()["safety_state"], "stop")

        # Picking a clear-path scenario must not release an engaged stop.
        self.app.set_safety_scenario("clear")

        self.assertEqual(self.app.status()["safety_state"], "stop")
        self.assertTrue(self.app.status()["safety"]["emergency_stop"])

    def test_only_an_explicit_reset_releases_the_stop(self) -> None:
        self.app.engage_emergency_stop()

        status = self.app.reset_emergency_stop()

        self.assertEqual(status["safety_state"], "clear")
        self.assertFalse(status["safety"]["emergency_stop"])

    def test_emergency_stop_abandons_the_active_trip(self) -> None:
        self.app.send_goal("kitchen")

        status = self.app.engage_emergency_stop()

        self.assertEqual(status["goal"]["state"], "cancelled")

    def test_no_goal_can_be_sent_while_stopped(self) -> None:
        self.app.engage_emergency_stop()

        with self.assertRaises(RuntimeError):
            self.app.send_goal("kitchen")

    def test_voice_cannot_move_the_robot_while_stopped(self) -> None:
        self.app.engage_emergency_stop()

        result = self.app.handle_voice_command("go to the kitchen")

        self.assertEqual(result["action"], "refused")
        self.assertNotIn("goal_id", result)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_saves_and_removes_named_locations(self) -> None:
        result = self.app.save_location(" Front Door ", 3.0, -1.0, 1.57)

        self.assertIn(
            {"name": "front door", "x": 3.0, "y": -1.0, "yaw": 1.57},
            result["locations"],
        )

        remaining = self.app.remove_location("front door")

        self.assertEqual([entry["name"] for entry in remaining["locations"]], ["kitchen"])

    def test_saved_locations_become_voice_destinations(self) -> None:
        self.app.save_location("front door", 3.0, -1.0)

        result = self.app.handle_voice_command("go to the front door")

        self.assertEqual(result["action"], "go_to")
        self.assertEqual(result["location_name"], "front door")

    def test_rejects_an_unnamed_or_unknown_location(self) -> None:
        with self.assertRaises(ValueError):
            self.app.save_location("   ", 1.0, 1.0)

        with self.assertRaises(KeyError):
            self.app.remove_location("garage")

    def test_stale_command_scenario_stops_robot(self) -> None:
        safety = self.app.set_safety_scenario("stale_command")

        self.assertEqual(safety["state"], "stop")
        self.assertEqual(safety["reason"], "motion command timed out")


    def test_scenarios_no_longer_include_the_emergency_stop(self) -> None:
        # The stop is a latched control, not a sensor condition. Leaving it in
        # the scenario list would let a scenario click release an engaged stop.
        self.assertNotIn("emergency_stop", SafetyScenarioAdapter.SCENARIOS)


if __name__ == "__main__":
    unittest.main()
