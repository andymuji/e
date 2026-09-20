import unittest

from robot_voice import VoiceIntentParser
from robot_voice.intent_parser import normalize_transcript


class VoiceIntentParserTests(unittest.TestCase):
    def setUp(self) -> None:
        self.parser = VoiceIntentParser()

    def test_parses_named_destination(self) -> None:
        self.assertEqual(
            self.parser.parse("Go to the Kitchen").location_name,
            "kitchen",
        )

    def test_parses_stop_variants(self) -> None:
        commands = (
            " emergency stop ",
            "Stop!",
            "please stop",
            "halt now.",
            "stop the robot now",
        )
        for command in commands:
            with self.subTest(command=command):
                self.assertEqual(self.parser.parse(command).action, "stop")

    def test_does_not_treat_negated_stop_as_stop(self) -> None:
        self.assertEqual(self.parser.parse("do not stop").action, "unknown")

    def test_parses_home(self) -> None:
        intent = self.parser.parse("return home")
        self.assertEqual(intent.action, "go_to")
        self.assertEqual(intent.location_name, "home")

    def test_rejects_ambiguous_command(self) -> None:
        self.assertEqual(self.parser.parse("find my keys").action, "unknown")


class EngineOutputTests(unittest.TestCase):
    """What a speech engine emits, rather than what a person would type."""

    def setUp(self) -> None:
        self.parser = VoiceIntentParser()

    def test_reads_through_hesitation_and_courtesy(self) -> None:
        transcripts = (
            "um go to the kitchen",
            "uh, go to the kitchen.",
            "hey robot go to the kitchen",
            "go to the kitchen please",
            "go to the kitchen, now",
            "GO TO THE KITCHEN",
        )
        for transcript in transcripts:
            with self.subTest(transcript=transcript):
                self.assertEqual(self.parser.parse(transcript).location_name, "kitchen")

    def test_reads_through_a_false_start(self) -> None:
        self.assertEqual(
            self.parser.parse("go go to the the kitchen").location_name, "kitchen"
        )

    def test_reads_through_a_phrase_heard_twice(self) -> None:
        self.assertEqual(
            self.parser.parse("go to the kitchen go to the kitchen").location_name,
            "kitchen",
        )

    def test_stop_survives_hesitation(self) -> None:
        for transcript in ("um stop", "uh, stop!", "stop stop", "please stop now"):
            with self.subTest(transcript=transcript):
                self.assertEqual(self.parser.parse(transcript).action, "stop")

    def test_truncated_destination_does_not_become_a_neighbour(self) -> None:
        """A half-heard room must reach the store as the half it was heard as.

        The store matches names exactly, so "kitch" is a refusal there; what
        must never happen is this parser completing it to "kitchen".
        """
        for transcript, expected in (
            ("go to the kitch", "kitch"),
            ("go to the bed", "bed"),
            ("navigate to the bath", "bath"),
        ):
            with self.subTest(transcript=transcript):
                self.assertEqual(self.parser.parse(transcript).location_name, expected)

    def test_two_destinations_stay_ambiguous(self) -> None:
        for transcript in (
            "go to the kitchen and the bedroom",
            "go to the bedroom go to the kitchen",
        ):
            with self.subTest(transcript=transcript):
                self.assertNotIn(
                    self.parser.parse(transcript).location_name,
                    ("kitchen", "bedroom"),
                )

    def test_truncated_stop_is_not_heard_as_a_destination(self) -> None:
        """A stop cut mid-word must not become a trip.

        It parses as unknown here, which the gateway answers with a refusal.
        Making a degraded "sto" stop the robot is the gateway's stop matching,
        not this parser's job.
        """
        for transcript in ("sto", "hal", "stoppp"):
            with self.subTest(transcript=transcript):
                self.assertEqual(self.parser.parse(transcript).action, "unknown")

    def test_normalizing_only_ever_removes_words(self) -> None:
        """The safety property behind the clean-up: no word is invented."""
        transcripts = (
            "um go to the kitchen please",
            "go go to the the kitchen",
            "go to the kitchen go to the kitchen",
            "hey robot, stop!",
        )
        for transcript in transcripts:
            with self.subTest(transcript=transcript):
                spoken = set(transcript.lower().replace(",", " ").split())
                for word in normalize_transcript(transcript).split():
                    self.assertIn(word, {token.strip("!.,?") for token in spoken})

    def test_empty_and_noise_only_transcripts_are_unknown(self) -> None:
        for transcript in ("", "   ", "um", "...", "please"):
            with self.subTest(transcript=transcript):
                self.assertEqual(self.parser.parse(transcript).action, "unknown")


if __name__ == "__main__":
    unittest.main()
