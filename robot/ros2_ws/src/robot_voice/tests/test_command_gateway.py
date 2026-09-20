from pathlib import Path
import tempfile
import unittest

from robot_core import Pose2D
from robot_locations import LocationStore
from robot_voice import CommandGateway


class CommandGatewayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.store = LocationStore(Path(self.directory.name) / "locations.json")
        self.store.save_location("kitchen", Pose2D(1.0, 2.0))
        self.store.save_location("living room", Pose2D(3.0, 4.0))
        self.gateway = CommandGateway(self.store)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_approved_destination_becomes_a_goal(self) -> None:
        outcome = self.gateway.handle("Go to the Kitchen")

        self.assertEqual(outcome.action, "go_to")
        self.assertEqual(outcome.goal.location_name, "kitchen")
        self.assertEqual(outcome.goal.pose, Pose2D(1.0, 2.0))

    def test_unapproved_destination_is_refused_with_the_approved_list(self) -> None:
        outcome = self.gateway.handle("go to the moon")

        self.assertEqual(outcome.action, "refused")
        self.assertIsNone(outcome.goal)
        self.assertEqual(
            outcome.response,
            "I do not know a place called moon. I can go to the kitchen or the living room.",
        )

    def test_loose_destination_phrases_never_become_goals(self) -> None:
        for transcript in ("go to sleep", "go away", "navigate outside"):
            with self.subTest(transcript=transcript):
                outcome = self.gateway.handle(transcript)
                self.assertEqual(outcome.action, "refused")
                self.assertIsNone(outcome.goal)

    def test_stop_is_heard_inside_a_partial_transcript(self) -> None:
        for transcript in ("robot please stop now", "uh stop", "halt the robot"):
            with self.subTest(transcript=transcript):
                self.assertEqual(self.gateway.handle(transcript).action, "stop")

    def test_negated_stop_still_stops(self) -> None:
        self.assertEqual(self.gateway.handle("do not stop").action, "stop")

    def test_a_truncated_stop_halts_without_asking_for_the_latch(self) -> None:
        # What an engine makes of someone shouting stop at a moving robot.
        for transcript in ("sto", "hal", "whoa", "wait", "hold on", "stahp"):
            with self.subTest(transcript=transcript):
                outcome = self.gateway.handle(transcript)
                self.assertEqual(outcome.action, "halt")
                self.assertIsNone(outcome.goal)

    def test_a_truncated_stop_never_becomes_a_destination(self) -> None:
        self.assertEqual(self.gateway.handle("go to the sto").action, "halt")

    def test_a_whole_stop_word_still_wins_over_a_mangled_one(self) -> None:
        self.assertEqual(self.gateway.handle("wait stop").action, "stop")

    def test_stop_wins_over_a_destination_in_the_same_transcript(self) -> None:
        outcome = self.gateway.handle("go to the kitchen no stop")

        self.assertEqual(outcome.action, "stop")
        self.assertIsNone(outcome.goal)

    def test_home_is_refused_when_no_home_is_configured(self) -> None:
        outcome = self.gateway.handle("go home")

        self.assertEqual(outcome.action, "refused")
        self.assertIn("I do not know a place called home", outcome.response)

    def test_configured_home_resolves_to_its_approved_location(self) -> None:
        gateway = CommandGateway(self.store, home_location="Living Room")

        outcome = gateway.handle("return home")

        self.assertEqual(outcome.action, "go_to")
        self.assertEqual(outcome.goal.location_name, "living room")

    def test_rejects_a_home_that_is_not_an_approved_location(self) -> None:
        with self.assertRaises(ValueError):
            CommandGateway(self.store, home_location="garage")

    def test_reports_location_without_commanding_motion(self) -> None:
        outcome = self.gateway.handle("where are you")

        self.assertEqual(outcome.action, "report_location")
        self.assertIsNone(outcome.goal)

    def test_unknown_command_is_refused(self) -> None:
        outcome = self.gateway.handle("find my keys")

        self.assertEqual(outcome.action, "refused")
        self.assertIsNone(outcome.goal)

    def test_refusal_is_usable_with_an_empty_location_store(self) -> None:
        empty_store = LocationStore(Path(self.directory.name) / "empty.json")

        outcome = CommandGateway(empty_store).handle("go to the kitchen")

        self.assertEqual(outcome.action, "refused")
        self.assertIn("No destinations have been approved yet.", outcome.response)


if __name__ == "__main__":
    unittest.main()
