"""Small local web control surface for approved robot destinations."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from threading import Lock
from typing import Any, Protocol
from urllib.parse import urlparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import uuid

from robot_locations import LocationStore
from robot_safety import SafetyController
from robot_voice import CommandGateway


@dataclass(frozen=True)
class GoalStatus:
    state: str
    location_name: str | None = None
    goal_id: str | None = None
    message: str = ""


class GoalDispatcher(Protocol):
    def send_goal(self, goal: Any) -> str: ...

    def cancel_goal(self, goal_id: str) -> None: ...

    def status(self) -> GoalStatus: ...


class DemoGoalDispatcher:
    """In-memory adapter used until this boundary is connected to Nav2."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._status = GoalStatus("ready", message="Robot is ready")

    def send_goal(self, goal: Any) -> str:
        with self._lock:
            if self._status.state == "navigating":
                raise RuntimeError("robot is already navigating")
            goal_id = uuid.uuid4().hex
            self._status = GoalStatus(
                "navigating",
                location_name=goal.location_name,
                goal_id=goal_id,
                message=f"Going to {goal.location_name}",
            )
            return goal_id

    def cancel_goal(self, goal_id: str) -> None:
        with self._lock:
            if self._status.goal_id != goal_id or self._status.state != "navigating":
                raise ValueError("no matching active goal")
            self._status = GoalStatus("cancelled", message="Trip cancelled")

    def status(self) -> GoalStatus:
        with self._lock:
            return self._status


class SafetyScenarioAdapter:
    """Deterministic safety inputs for the local developer console."""

    SCENARIOS = {
        "clear": (1.5, 0.0, False),
        "caution": (0.6, 0.0, False),
        "stop": (0.2, 0.0, False),
        "emergency_stop": (1.5, 0.0, True),
        "stale_sensor": (1.5, 1.0, False),
        "invalid_sensor": (None, 0.0, False),
    }

    def __init__(self) -> None:
        self._controller = SafetyController()
        self._scenario = "clear"

    def set_scenario(self, scenario: str) -> None:
        if scenario not in self.SCENARIOS:
            raise ValueError(f"unknown safety scenario: {scenario}")
        self._scenario = scenario

    def status(self) -> dict[str, Any]:
        distance, reading_age, emergency_stop = self.SCENARIOS[self._scenario]
        self._controller.set_emergency_stop(emergency_stop)
        decision = self._controller.evaluate(distance, reading_age)
        return {
            "scenario": self._scenario,
            "state": decision.state.value,
            "speed_scale": decision.speed_scale,
            "reason": decision.reason,
            "nearest_obstacle_distance": distance,
            "reading_age": reading_age,
            "emergency_stop": emergency_stop,
        }


class RobotWebApp:
    def __init__(
        self,
        location_store: LocationStore,
        dispatcher: GoalDispatcher,
        safety: SafetyScenarioAdapter | None = None,
        gateway: CommandGateway | None = None,
    ):
        self.location_store = location_store
        self.dispatcher = dispatcher
        self.safety = safety or SafetyScenarioAdapter()
        self.gateway = gateway or CommandGateway(location_store)

    def locations(self) -> dict[str, list[dict[str, str]]]:
        return {"locations": [{"name": name} for name in self.location_store.names()]}

    def status(self) -> dict[str, Any]:
        safety = self.safety.status()
        return {
            "goal": asdict(self.dispatcher.status()),
            "safety_state": safety["state"],
            "safety_message": safety["reason"],
            "safety": safety,
        }

    def set_safety_scenario(self, scenario: str) -> dict[str, Any]:
        self.safety.set_scenario(scenario)
        return self.safety.status()

    def send_goal(self, location_name: str) -> dict[str, Any]:
        goal = self.location_store.get_goal(location_name)
        goal_id = self.dispatcher.send_goal(goal)
        return {"goal_id": goal_id, "location_name": goal.location_name}

    def handle_voice_command(self, transcript: str) -> dict[str, Any]:
        outcome = self.gateway.handle(transcript)
        result: dict[str, Any] = {"action": outcome.action, "response": outcome.response}
        status = self.dispatcher.status()

        if outcome.action == "stop":
            if status.state == "navigating" and status.goal_id is not None:
                self.dispatcher.cancel_goal(status.goal_id)
        elif outcome.action == "go_to":
            result["goal_id"] = self.dispatcher.send_goal(outcome.goal)
            result["location_name"] = outcome.goal.location_name
        elif outcome.action == "report_location":
            result["response"] = (
                f"I am on my way to the {status.location_name}."
                if status.state == "navigating"
                else "I am not moving right now."
            )

        return result

    def cancel_goal(self) -> dict[str, str]:
        status = self.dispatcher.status()
        if status.state != "navigating" or status.goal_id is None:
            raise ValueError("there is no active trip to cancel")
        self.dispatcher.cancel_goal(status.goal_id)
        return {"message": "Trip cancelled"}


def make_handler(app: RobotWebApp, web_root: Path):
    class RobotRequestHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/api/locations":
                self._send_json(200, app.locations())
            elif path == "/api/status":
                self._send_json(200, app.status())
            elif path == "/" or path == "/index.html":
                self._send_file(web_root / "index.html", "text/html; charset=utf-8")
            elif path == "/styles.css":
                self._send_file(web_root / "styles.css", "text/css; charset=utf-8")
            elif path == "/app.js":
                self._send_file(web_root / "app.js", "text/javascript; charset=utf-8")
            else:
                self._send_json(404, {"error": "not found"})

        def do_POST(self) -> None:
            try:
                path = urlparse(self.path).path
                if path == "/api/goals":
                    body = json.loads(self._read_body())
                    result = app.send_goal(body["location_name"])
                    self._send_json(201, result)
                elif path == "/api/goals/cancel":
                    self._send_json(200, app.cancel_goal())
                elif path == "/api/voice":
                    body = json.loads(self._read_body())
                    self._send_json(200, app.handle_voice_command(body["transcript"]))
                elif path == "/api/safety/scenario":
                    body = json.loads(self._read_body())
                    self._send_json(200, app.set_safety_scenario(body["scenario"]))
                else:
                    self._send_json(404, {"error": "not found"})
            except (KeyError, TypeError, json.JSONDecodeError) as error:
                self._send_json(400, {"error": str(error)})
            except (ValueError, RuntimeError) as error:
                self._send_json(409, {"error": str(error)})

        def _read_body(self) -> str:
            length = int(self.headers.get("Content-Length", "0"))
            return self.rfile.read(length).decode("utf-8")

        def _send_json(self, status: int, payload: dict[str, Any]) -> None:
            encoded = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(encoded)

        def _send_file(self, path: Path, content_type: str) -> None:
            try:
                content = path.read_bytes()
            except FileNotFoundError:
                self._send_json(404, {"error": "not found"})
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, format: str, *args: Any) -> None:
            return

    return RobotRequestHandler


def main() -> None:
    root = Path(__file__).parent
    store = LocationStore(root / "locations.json")
    app = RobotWebApp(store, DemoGoalDispatcher(), SafetyScenarioAdapter())
    server = ThreadingHTTPServer(("127.0.0.1", 8080), make_handler(app, root / "web"))
    print("Robot destination app: http://127.0.0.1:8080")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()