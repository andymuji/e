"""Small local web control surface for approved robot destinations."""

from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from threading import Lock, RLock
import traceback
from typing import Any, Protocol
from urllib.parse import urlparse
import uuid

from robot_core import Pose2D
from robot_locations import LocationStore
from robot_safety import SafetyController
from robot_voice import CommandGateway


class BadRequest(ValueError):
    """The request itself is unusable, as opposed to refused by the robot.

    A ValueError so that callers using RobotWebApp directly still see the
    kind of error they saw before, while the HTTP layer can answer 400
    instead of 409: a malformed field is not a conflict with robot state,
    and telling an operator "conflict" for a typo sends them looking at the
    robot instead of at the form.
    """


def _text_field(value: object, field: str) -> str:
    """Require a string. Anything else used to escape as an AttributeError."""
    if not isinstance(value, str):
        raise BadRequest(f"{field} must be text")
    return value


def _number_field(value: object, field: str) -> float:
    """Require a real, finite number.

    JSON's NaN and Infinity parse happily into floats, and a pose at NaN is
    a destination the robot can be sent to and never arrive at.
    """
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise BadRequest(f"{field} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise BadRequest(f"{field} must be a finite number")
    return number


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
    """Deterministic safety inputs for the local developer console.

    Scenarios vary sensor conditions only. The emergency stop is not a sensor
    condition, so it is a separate control with its own latch, exactly as it
    is on the robot: engaging it publishes to `emergency_stop` and releasing
    it publishes to `emergency_stop_reset`. Replace this class with one that
    publishes those topics when the console is pointed at a real robot.
    """

    # nearest obstacle distance, scan age, motion command age
    SCENARIOS = {
        "clear": (1.5, 0.0, 0.0),
        "caution": (0.6, 0.0, 0.0),
        "stop": (0.2, 0.0, 0.0),
        "stale_sensor": (1.5, 1.0, 0.0),
        "stale_command": (1.5, 0.0, 1.0),
        "invalid_sensor": (None, 0.0, 0.0),
    }

    def __init__(self) -> None:
        self._controller = SafetyController()
        self._scenario = "clear"

    def set_scenario(self, scenario: str) -> None:
        if scenario not in self.SCENARIOS:
            raise ValueError(f"unknown safety scenario: {scenario}")
        self._scenario = scenario

    def engage_emergency_stop(self) -> None:
        self._controller.engage_emergency_stop()

    def reset_emergency_stop(self) -> None:
        self._controller.clear_emergency_stop()

    @property
    def emergency_stop_engaged(self) -> bool:
        return self._controller.emergency_stop_engaged

    def status(self) -> dict[str, Any]:
        distance, reading_age, command_age = self.SCENARIOS[self._scenario]
        decision = self._controller.evaluate(distance, reading_age, command_age)
        return {
            "scenario": self._scenario,
            "state": decision.state.value,
            "speed_scale": decision.speed_scale,
            "reason": decision.reason,
            "nearest_obstacle_distance": distance,
            "reading_age": reading_age,
            "command_age": command_age,
            "emergency_stop": self._controller.emergency_stop_engaged,
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
        # The server is threaded, so two operator requests can reach the
        # store at once. Reentrant because the mutating calls report the new
        # list by calling locations() while still holding it.
        self._store_lock = RLock()

    def locations(self) -> dict[str, list[dict[str, Any]]]:
        with self._store_lock:
            # names() then get_goal() per name is a read-modify-read: without
            # the lock a concurrent removal between the two raised KeyError
            # and the destination list vanished behind a failed request.
            return {
                "locations": [
                    {"name": goal.location_name, "x": goal.pose.x, "y": goal.pose.y,
                     "yaw": goal.pose.yaw}
                    for goal in map(
                        self.location_store.get_goal, self.location_store.names()
                    )
                ]
            }

    def save_location(
        self, name: str, x: float, y: float, yaw: float = 0.0
    ) -> dict[str, Any]:
        """Approve a named pose. Saving an existing name overwrites it."""
        name = _text_field(name, "name")
        if not name.strip():
            raise BadRequest("name must not be empty")
        pose = Pose2D(
            _number_field(x, "x"), _number_field(y, "y"), _number_field(yaw, "yaw")
        )
        with self._store_lock:
            self.location_store.save_location(name, pose)
            return self.locations()

    def remove_location(self, name: str) -> dict[str, Any]:
        name = _text_field(name, "name")
        with self._store_lock:
            self.location_store.remove_location(name)
            return self.locations()

    def engage_emergency_stop(self) -> dict[str, Any]:
        """Latch the stop and abandon any trip in progress.

        A stopped robot is not on its way anywhere, so leaving the goal
        showing as active would misreport what the robot is doing.
        """
        self.safety.engage_emergency_stop()
        status = self.dispatcher.status()
        if status.state == "navigating" and status.goal_id is not None:
            self.dispatcher.cancel_goal(status.goal_id)
        return self.status()

    def reset_emergency_stop(self) -> dict[str, Any]:
        """Release the latch. Deliberate, and never triggered by polling."""
        self.safety.reset_emergency_stop()
        return self.status()

    def status(self) -> dict[str, Any]:
        safety = self.safety.status()
        return {
            "goal": asdict(self.dispatcher.status()),
            "safety_state": safety["state"],
            "safety_message": safety["reason"],
            "safety": safety,
        }

    def set_safety_scenario(self, scenario: str) -> dict[str, Any]:
        scenario = _text_field(scenario, "scenario")
        if scenario not in self.safety.SCENARIOS:
            raise BadRequest(f"unknown safety scenario: {scenario}")
        self.safety.set_scenario(scenario)
        # The same envelope every other endpoint returns. It used to answer
        # with the bare safety dict, so a caller that rendered this response
        # the way it renders /api/status lost the goal and the stop latch.
        return self.status()

    def send_goal(self, location_name: str) -> dict[str, Any]:
        location_name = _text_field(location_name, "location_name")
        # Checked before the lookup, so an engaged stop is reported as the
        # reason for refusing rather than being masked by a bad name.
        self._refuse_while_stopped()
        with self._store_lock:
            goal = self.location_store.get_goal(location_name)
            goal_id = self.dispatcher.send_goal(goal)
        return {"goal_id": goal_id, "location_name": goal.location_name}

    def _refuse_while_stopped(self) -> None:
        if self.safety.emergency_stop_engaged:
            raise RuntimeError(
                "the emergency stop is engaged: reset it before sending a goal"
            )

    def handle_voice_command(self, transcript: str) -> dict[str, Any]:
        transcript = _text_field(transcript, "transcript")
        with self._store_lock:
            # Held across the whole decision so the approved destinations
            # cannot change between resolving the goal and dispatching it.
            return self._handle_voice_command(transcript)

    def _handle_voice_command(self, transcript: str) -> dict[str, Any]:
        outcome = self.gateway.handle(transcript)
        result: dict[str, Any] = {"action": outcome.action, "response": outcome.response}
        status = self.dispatcher.status()

        if outcome.action == "stop":
            if status.state == "navigating" and status.goal_id is not None:
                self.dispatcher.cancel_goal(status.goal_id)
        elif outcome.action == "go_to":
            if self.safety.emergency_stop_engaged:
                return {
                    "action": "refused",
                    "response": "I cannot move: the emergency stop is engaged.",
                }
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


@dataclass(frozen=True)
class _Reply:
    """One HTTP answer: a JSON payload, or raw bytes for a static file."""

    status: int
    payload: dict[str, Any] | None = None
    body: bytes | None = None
    content_type: str = "application/json"


def make_handler(app: RobotWebApp, web_root: Path):
    # A body larger than this is not a request this console has any use for,
    # and reading an arbitrary Content-Length ties up a server thread.
    max_body_bytes = 64 * 1024

    static_files = {
        "/": ("index.html", "text/html; charset=utf-8"),
        "/index.html": ("index.html", "text/html; charset=utf-8"),
        "/styles.css": ("styles.css", "text/css; charset=utf-8"),
        "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    }

    class RobotRequestHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            self._respond(self._route_get)

        def do_POST(self) -> None:
            self._respond(self._route_post)

        def _respond(self, route) -> None:
            """Answer exactly once, whatever the route does.

            Every branch below ends in a response. An exception that escaped
            instead used to drop the connection unanswered, and an
            unanswered poll leaves the console showing its last state - the
            stop latch included - with nothing to say it is out of date.
            """
            try:
                reply = route(urlparse(self.path).path)
            except BadRequest as error:
                reply = _Reply(400, {"error": str(error)})
            except KeyError as error:
                # str(KeyError) quotes its argument, which reads badly in the
                # UI. A store lookup carries a sentence; a missing body field
                # carries a bare field name.
                message = str(error.args[0]) if error.args else "missing field"
                if " " not in message:
                    message = f"missing required field: {message}"
                reply = _Reply(400, {"error": message})
            except (TypeError, json.JSONDecodeError) as error:
                reply = _Reply(400, {"error": str(error)})
            except (ValueError, RuntimeError) as error:
                reply = _Reply(409, {"error": str(error)})
            except Exception:
                # Last resort. The traceback goes to the console's own output
                # rather than to the operator, who gets an answer either way.
                traceback.print_exc()
                reply = _Reply(500, {"error": "the robot hit an internal error"})

            self._send(reply)

        def _route_get(self, path: str) -> "_Reply":
            if path == "/api/locations":
                return _Reply(200, app.locations())
            if path == "/api/status":
                return _Reply(200, app.status())
            static = static_files.get(path)
            if static is not None:
                name, content_type = static
                try:
                    content = (web_root / name).read_bytes()
                except OSError:
                    return _Reply(404, {"error": "not found"})
                return _Reply(200, None, content, content_type)
            return _Reply(404, {"error": "not found"})

        def _route_post(self, path: str) -> "_Reply":
            if path == "/api/goals/cancel":
                return _Reply(200, app.cancel_goal())
            if path == "/api/emergency_stop":
                return _Reply(200, app.engage_emergency_stop())
            if path == "/api/emergency_stop/reset":
                return _Reply(200, app.reset_emergency_stop())
            if path == "/api/goals":
                body = self._json_object()
                return _Reply(201, app.send_goal(self._field(body, "location_name")))
            if path == "/api/voice":
                body = self._json_object()
                transcript = self._field(body, "transcript")
                return _Reply(200, app.handle_voice_command(transcript))
            if path == "/api/safety/scenario":
                body = self._json_object()
                scenario = self._field(body, "scenario")
                return _Reply(200, app.set_safety_scenario(scenario))
            if path == "/api/locations":
                body = self._json_object()
                return _Reply(201, app.save_location(
                    self._field(body, "name"),
                    self._field(body, "x"),
                    self._field(body, "y"),
                    body.get("yaw", 0.0),
                ))
            if path == "/api/locations/remove":
                body = self._json_object()
                return _Reply(200, app.remove_location(self._field(body, "name")))
            return _Reply(404, {"error": "not found"})

        @staticmethod
        def _field(body: dict[str, Any], name: str) -> Any:
            if name not in body:
                raise BadRequest(f"missing required field: {name}")
            return body[name]

        def _json_object(self) -> dict[str, Any]:
            try:
                parsed = json.loads(self._read_body())
            except json.JSONDecodeError as error:
                raise BadRequest(f"body is not valid JSON: {error}") from error
            if not isinstance(parsed, dict):
                raise BadRequest("body must be a JSON object")
            return parsed

        def _read_body(self) -> str:
            raw = self.headers.get("Content-Length", "0")
            try:
                length = int(raw)
            except ValueError as error:
                raise BadRequest(f"invalid Content-Length: {raw!r}") from error
            if length < 0:
                raise BadRequest(f"invalid Content-Length: {raw!r}")
            if length > max_body_bytes:
                raise BadRequest(f"request body is larger than {max_body_bytes} bytes")
            body = self.rfile.read(length)
            if len(body) != length:
                raise BadRequest("request body ended early")
            try:
                return body.decode("utf-8")
            except UnicodeDecodeError as error:
                raise BadRequest("request body must be UTF-8") from error

        def _send(self, reply: "_Reply") -> None:
            if reply.body is None:
                content = json.dumps(reply.payload).encode("utf-8")
                content_type = "application/json"
            else:
                content, content_type = reply.body, reply.content_type
            self.send_response(reply.status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
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
