"""The seam between a speech engine and the command path.

A recognizer produces text and nothing else. It never decides what the robot
does: every transcript still goes through the parser and CommandGateway, so
swapping Vosk, Whisper, or a cloud service in here cannot widen what counts as
an approved destination and cannot reach a velocity. Which engine sits behind
this seam is ADR 0002's decision, not this module's.

Until that ADR is settled, TypedTextRecognizer is the engine: typed text is a
transcript like any other, which is what makes the voice path demonstrable
with no microphone and no model download.
"""

from collections.abc import Callable, Iterator
from typing import Protocol, runtime_checkable


@runtime_checkable
class SpeechRecognizer(Protocol):
    """Yield final transcripts, one per recognized utterance."""

    def transcripts(self) -> Iterator[str]:
        """Yield transcripts until the source ends.

        An engine that yields partial hypotheses must yield only its final
        ones here: a half-recognized destination is exactly the input the
        rest of the path has to refuse, so it must never arrive as if it
        were what the person finished saying.
        """
        ...

    def close(self) -> None:
        """Release the microphone, model, or socket. Safe to call twice."""
        ...


class TypedTextRecognizer:
    """Typed text standing in for a microphone (see ADR 0002).

    One-shot when constructed with phrases, interactive when constructed with
    a prompt function. Blank input yields nothing rather than an empty
    transcript, because an empty transcript is a refusal the person did not
    ask for.
    """

    def __init__(
        self,
        phrases: tuple[str, ...] | list[str] | None = None,
        prompt: Callable[[], str] | None = None,
    ) -> None:
        if phrases is None and prompt is None:
            raise ValueError("TypedTextRecognizer needs phrases or a prompt")
        self._phrases = tuple(phrases or ())
        self._prompt = prompt

    def transcripts(self) -> Iterator[str]:
        for phrase in self._phrases:
            if phrase.strip():
                yield phrase
        if self._prompt is None:
            return

        while True:
            try:
                line = self._prompt()
            except EOFError:
                return  # Ctrl-D: the person is done talking.
            if line.strip():
                yield line

    def close(self) -> None:
        """Nothing to release: the keyboard is not a device this owns."""
