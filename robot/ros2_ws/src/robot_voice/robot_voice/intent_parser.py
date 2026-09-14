from dataclasses import dataclass
import re


@dataclass(frozen=True)
class VoiceIntent:
    action: str
    location_name: str | None = None


class VoiceIntentParser:
    """Parse only explicit, allow-listed robot commands."""

    _go_patterns = (
        re.compile(r"^(?:go|navigate|take me) to the (.+)$"),
        re.compile(r"^(?:go|navigate) (?:to )?(.+)$"),
    )

    def parse(self, transcript: str) -> VoiceIntent:
        command = " ".join(transcript.lower().strip().split())
        if command in {"stop", "halt", "emergency stop"}:
            return VoiceIntent("stop")
        if command in {"where are you", "where are you?", "report location"}:
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
