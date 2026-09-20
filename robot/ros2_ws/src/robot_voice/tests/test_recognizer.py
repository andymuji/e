import unittest

from robot_voice.recognizer import SpeechRecognizer, TypedTextRecognizer


def _typist(*lines: str):
    """A prompt that types the given lines and then presses Ctrl-D."""
    remaining = list(lines)

    def prompt() -> str:
        if not remaining:
            raise EOFError
        return remaining.pop(0)

    return prompt


class TypedTextRecognizerTests(unittest.TestCase):
    def test_is_a_speech_recognizer(self) -> None:
        """The seam is what lets a real engine replace this one later."""
        self.assertIsInstance(TypedTextRecognizer(phrases=("stop",)), SpeechRecognizer)

    def test_yields_phrases_in_order(self) -> None:
        recognizer = TypedTextRecognizer(phrases=("go to the kitchen", "stop"))
        self.assertEqual(
            list(recognizer.transcripts()), ["go to the kitchen", "stop"]
        )

    def test_yields_typed_lines_until_end_of_input(self) -> None:
        recognizer = TypedTextRecognizer(prompt=_typist("stop", "where are you"))
        self.assertEqual(list(recognizer.transcripts()), ["stop", "where are you"])

    def test_blank_input_is_not_an_utterance(self) -> None:
        """An empty transcript would be answered as a refusal nobody asked for."""
        recognizer = TypedTextRecognizer(phrases=("", "   "), prompt=_typist("", "stop"))
        self.assertEqual(list(recognizer.transcripts()), ["stop"])

    def test_needs_something_to_say(self) -> None:
        with self.assertRaises(ValueError):
            TypedTextRecognizer()

    def test_close_is_safe_to_repeat(self) -> None:
        recognizer = TypedTextRecognizer(phrases=("stop",))
        recognizer.close()
        recognizer.close()


if __name__ == "__main__":
    unittest.main()
