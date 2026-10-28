from pathlib import Path
import tempfile
import unittest
from unittest import mock

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

    def test_a_save_that_cannot_be_written_is_not_remembered(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "locations.json"
            store = LocationStore(path)
            store.save_location("kitchen", Pose2D(1.0, 2.0))
            before = path.read_text()

            with mock.patch("os.replace", side_effect=OSError("disk full")):
                with self.assertRaises(OSError):
                    store.save_location("sofa", Pose2D(3.0, 4.0))
                with self.assertRaises(OSError):
                    store.remove_location("kitchen")

            # Answering from a place the file does not hold would lose it on
            # the next start-up, and a torn file would lose every place.
            self.assertEqual(store.names(), ("kitchen",))
            self.assertEqual(path.read_text(), before)
            self.assertEqual(LocationStore(path).names(), ("kitchen",))

    def test_a_file_with_a_non_finite_pose_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "locations.json"
            path.write_text('{"kitchen": {"x": NaN, "y": 0.0}}')

            # Python's json accepts NaN; a destination there can be sent and
            # never reached, so loading has to stop at it.
            with self.assertRaises(ValueError):
                LocationStore(path)


if __name__ == "__main__":
    unittest.main()
