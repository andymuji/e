import tempfile
import unittest
from pathlib import Path

from robot_core import Pose2D
from robot_locations import LocationStore


class LocationStoreTests(unittest.TestCase):
    def test_saves_and_loads_named_goal(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "locations.json"
            store = LocationStore(path)
            store.save_location(" Kitchen ", Pose2D(1.0, 2.0, 0.5))

            loaded = LocationStore(path)
            goal = loaded.get_goal("kitchen")

            self.assertEqual(loaded.names(), ("kitchen",))
            self.assertEqual(goal.location_name, "kitchen")
            self.assertEqual(goal.pose, Pose2D(1.0, 2.0, 0.5))

    def test_unknown_location_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = LocationStore(Path(directory) / "locations.json")
            with self.assertRaises(KeyError):
                store.get_goal("sofa")

    def test_empty_name_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = LocationStore(Path(directory) / "locations.json")
            with self.assertRaises(ValueError):
                store.save_location("  ", Pose2D(0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
