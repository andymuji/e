import json
from pathlib import Path
import tempfile
import unittest

from robot_core import Pose2D
from robot_locations import LocationStore

from web_ui.app import DemoGoalDispatcher, RobotWebApp


class RobotWebAppTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("Kitchen", Pose2D(1.0, 2.0))
        self.dispatcher = DemoGoalDispatcher()
        self.app = RobotWebApp(store, self.dispatcher)

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


if __name__ == "__main__":
    unittest.main()