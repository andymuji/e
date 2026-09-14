import unittest

from robot_voice import VoiceIntentParser


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


if __name__ == "__main__":
    unittest.main()
