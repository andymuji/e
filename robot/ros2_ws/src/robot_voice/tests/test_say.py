import unittest

from robot_voice.say import TRANSCRIPT_TOPIC, phrases_from_args, wait_for_listeners


class Listeners:
    """A subscription count that appears after a given number of polls."""

    def __init__(self, appears_after: int) -> None:
        self.polls = 0
        self._appears_after = appears_after

    def count(self) -> int:
        return 1 if self.polls >= self._appears_after else 0

    def settle(self) -> None:
        self.polls += 1


class PhrasesFromArgsTests(unittest.TestCase):
    def test_joins_unquoted_words_into_one_sentence(self) -> None:
        self.assertEqual(
            phrases_from_args(["go", "to", "the", "kitchen"]),
            ("go to the kitchen",),
        )

    def test_keeps_a_quoted_sentence_whole(self) -> None:
        self.assertEqual(
            phrases_from_args(["go to the kitchen"]), ("go to the kitchen",)
        )

    def test_no_words_means_interactive(self) -> None:
        self.assertEqual(phrases_from_args([]), ())
        self.assertEqual(phrases_from_args(["  "]), ())


class WaitForListenersTests(unittest.TestCase):
    def test_publishes_at_once_when_someone_is_listening(self) -> None:
        listeners = Listeners(appears_after=0)
        self.assertTrue(wait_for_listeners(listeners.count, listeners.settle, 10))
        self.assertEqual(listeners.polls, 0)

    def test_waits_for_discovery(self) -> None:
        """Discovery takes a moment; publishing before it drops the message."""
        listeners = Listeners(appears_after=3)
        self.assertTrue(wait_for_listeners(listeners.count, listeners.settle, 10))
        self.assertEqual(listeners.polls, 3)

    def test_gives_up_rather_than_publishing_into_nothing(self) -> None:
        listeners = Listeners(appears_after=99)
        self.assertFalse(wait_for_listeners(listeners.count, listeners.settle, 5))

    def test_publishes_the_transcript_topic_only(self) -> None:
        """The tool's whole authority: one topic, and not a velocity."""
        self.assertEqual(TRANSCRIPT_TOPIC, "speech_transcript")


if __name__ == "__main__":
    unittest.main()
