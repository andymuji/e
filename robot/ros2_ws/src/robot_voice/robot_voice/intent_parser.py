from dataclasses import dataclass
import re

# What a real engine emits is not what a person types: no capitals, no
# punctuation, and a sprinkling of hesitation noises it heard as words.
_DISFLUENCIES = frozenset({"um", "uhm", "uh", "erm", "er", "ah", "eh", "hmm", "mm"})

# Politeness and address that frame a command without being part of it. Only
# stripped at the edges: "stop the robot" ends in a word this set contains, and
# that sentence is a stop, not a truncated one.
_LEADING_FRAMING = frozenset({"hey", "robot", "please", "okay", "ok", "so", "well"})
_TRAILING_FRAMING = frozenset({"please", "now", "thanks"})

_PUNCTUATION = re.compile(r"[^a-z0-9' ]+")


def normalize_transcript(transcript: str) -> str:
    """Reduce what an engine emitted to the words the person meant.

    Lower-cased, punctuation-stripped, hesitations dropped, false starts and a
    repeated phrase collapsed. This only ever removes words, so a transcript
    can lose noise but can never gain a destination nobody said.
    """
    text = _PUNCTUATION.sub(" ", transcript.lower())
    words = [word for word in text.split() if word not in _DISFLUENCIES]

    while words and words[0] in _LEADING_FRAMING:
        words.pop(0)
    while words and words[-1] in _TRAILING_FRAMING:
        words.pop()

    return " ".join(_collapse_repeats(words))


def _collapse_repeats(words: list[str]) -> list[str]:
    """Drop a stutter and a sentence said twice, keeping the words said once."""
    collapsed = [
        word for index, word in enumerate(words) if index == 0 or word != words[index - 1]
    ]

    # "go to the kitchen go to the kitchen": one request, heard twice, and
    # otherwise read as a destination called "kitchen go to the kitchen".
    half = len(collapsed) // 2
    if half and collapsed[:half] == collapsed[half:]:
        return collapsed[:half]
    return collapsed


@dataclass(frozen=True)
class VoiceIntent:
    action: str
    location_name: str | None = None


class VoiceIntentParser:
    """Parse only explicit, allow-listed robot commands."""

    _stop_pattern = re.compile(
        r"^(?:please )?(?:stop|halt|emergency stop|stop the robot)(?: now)?$"
    )
    _go_patterns = (
        re.compile(r"^(?:go|navigate|take me) to the (.+)$"),
        re.compile(r"^(?:go|navigate) (?:to )?(.+)$"),
    )

    def parse(self, transcript: str) -> VoiceIntent:
        command = normalize_transcript(transcript)

        if self._stop_pattern.fullmatch(command):
            return VoiceIntent("stop")
        if command in {"where are you", "report location"}:
            return VoiceIntent("report_location")
        if command in {"return", "return home", "go home", "go to home"}:
            return VoiceIntent("go_to", "home")

        for pattern in self._go_patterns:
            match = pattern.fullmatch(command)
            if match:
                location_name = " ".join(match.group(1).strip().split())
                if location_name:
                    return VoiceIntent("go_to", location_name)

        return VoiceIntent("unknown")
