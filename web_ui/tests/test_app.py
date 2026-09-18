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
        self.assertEqual(
            self.app.locations(),
            {"locations": [{"name": "kitchen", "x": 1.0, "y": 2.0, "yaw": 0.0}]},
        )

    def test_map_is_built_from_saved_locations(self) -> None:
        map_data = self.app.map_data()

        self.assertEqual(map_data["locations"][0]["name"], "kitchen")
        self.assertGreaterEqual(map_data["bounds"]["max_x"], 6.0)
        self.assertTrue(map_data["walls"])

    def test_can_save_a_label_from_the_map(self) -> None:
        result = self.app.label_location("Reading nook", 3.25, 1.5)

        self.assertEqual(result["name"], "reading nook")
        self.assertEqual(self.app.locations()["locations"][1]["x"], 3.25)

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


if __name__ == "__main__":
    unittest.main()
