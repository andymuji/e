"""Resolve parsed voice intents into approved goals or spoken refusals."""

from dataclasses import dataclass

from robot_core import NavigationGoal
from robot_locations import LocationStore

from robot_voice.intent_parser import VoiceIntentParser, normalize_transcript

# A stop word heard whole. Matched anywhere in the transcript rather than as a
# whole utterance, so a partially recognised sentence still stops the robot.
# This deliberately stops on phrases like "do not stop": a needless stop is a
# nuisance, a missed one is a hazard.
_STOP_WORDS = frozenset({"stop", "halt", "emergency", "freeze"})

# A stop word the engine mangled, and the words people shout at a moving robot
# when a clean "stop" does not come out. A real engine truncates: "sto", "hal".
# These stop the wheels without latching the emergency stop, because the latch
# takes an operator to release and a robot stranded by a misheard syllable is
# its own hazard - it cannot go and fetch anyone either.
_UNCERTAIN_STOP_WORDS = frozenset(
    {"sto", "stp", "stah", "stahp", "hal", "hold", "wait", "whoa", "woah"}
)


@dataclass(frozen=True)
class CommandOutcome:
    action: str
    response: str
    goal: NavigationGoal | None = None


class CommandGateway:
    """Turn a transcript into an approved navigation goal, a stop, or a refusal."""

    def __init__(
        self,
        location_store: LocationStore,
        parser: VoiceIntentParser | None = None,
        home_location: str | None = None,
    ):
        self._location_store = location_store
        self._parser = parser or VoiceIntentParser()

        if home_location is None:
            self._home_location = None
        else:
            try:
                self._home_location = location_store.get_goal(home_location).location_name
            except KeyError as error:
                raise ValueError(
                    f"home location is not an approved destination: {home_location}"
                ) from error

    def handle(self, transcript: str) -> CommandOutcome:
        # Judged on the same cleaned-up text the parser sees, so the stop check
        # and the destination check cannot disagree about what was said.
        spoken = set(normalize_transcript(transcript).split())

        if spoken & _STOP_WORDS:
            return CommandOutcome("stop", "Stopping now.")
        if spoken & _UNCERTAIN_STOP_WORDS:
            # Not confident enough to latch, far too confident to keep driving.
            return CommandOutcome(
                "halt",
                "I think you asked me to stop, so I have. "
                "Say stop if you want me to stay stopped.",
            )

        intent = self._parser.parse(transcript)

        if intent.action == "report_location":
            return CommandOutcome("report_location", "Checking where I am.")
        if intent.action == "go_to":
            return self._resolve_destination(intent.location_name)

        return CommandOutcome(
            "refused",
            "I did not understand that. You can ask me to stop, "
            f"or to go somewhere. {self._approved_destinations()}",
        )

    def _resolve_destination(self, location_name: str) -> CommandOutcome:
        if location_name == "home" and self._home_location is not None:
            location_name = self._home_location

        try:
            goal = self._location_store.get_goal(location_name)
        except KeyError:
            return CommandOutcome(
                "refused",
                f"I do not know a place called {location_name}. "
                f"{self._approved_destinations()}",
            )

        return CommandOutcome("go_to", f"Going to the {goal.location_name}.", goal)

    def _approved_destinations(self) -> str:
        names = self._location_store.names()
        if not names:
            return "No destinations have been approved yet."
        if len(names) == 1:
            return f"I can go to the {names[0]}."
        if len(names) == 2:
            return f"I can go to the {names[0]} or the {names[1]}."
        return f"I can go to the {', '.join(names[:-1])}, or {names[-1]}."
