from dataclasses import dataclass
import re


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
        command = " ".join(transcript.lower().strip().split())
        command = re.sub(r"[.!?]+$", "", command).strip()

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
