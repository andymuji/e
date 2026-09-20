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

from robot_core import NavigationGoal, Pose2D
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


class SafetyAdapter(Protocol):
    """The console's view of the safety gate, real or simulated.

    `motion_refusal` is the one the console acts on, and it answers "may this
    console ask for motion" rather than "is the latch down". An adapter that
    has not heard from the gate answers with a reason, because a console that
    cannot see the latch must not dispatch a trip. `status()` carries the
    nuance for display and may report the latch as None, which the page
    renders as "unknown" rather than as released.
    """

    @property
    def emergency_stop_engaged(self) -> bool: ...

    def motion_refusal(self) -> str | None: ...

    def engage_emergency_stop(self) -> None: ...

    def reset_emergency_stop(self) -> None: ...

    def status(self) -> dict[str, Any]: ...


class MapProvider(Protocol):
    """Where the floor plan comes from: invented, or a real occupancy grid."""

    def map_data(self, locations: list[dict[str, Any]]) -> dict[str, Any]: ...


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
    it publishes to `emergency_stop_reset`. Pointed at a real robot the
    console uses robot_console's adapter, which publishes those topics and
    reads the gate's own state back; this one stands in when there is no
    robot to ask.
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

    def motion_refusal(self) -> str | None:
        """The local controller always knows its own latch, so this is it."""
        if self._controller.emergency_stop_engaged:
            return "the emergency stop is engaged"
        return None

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


class DemoMapProvider:
    """A floor plan with no robot behind it, for the local console.

    The walls and the robot pose are placeholders and describe no room that
    exists. A provider backed by a SLAM map supplies the same contract, which
    is why this is a separate object rather than a method on the app.
    """

    def map_data(self, locations: list[dict[str, Any]]) -> dict[str, Any]:
        max_x = max((float(item["x"]) for item in locations), default=5.0)
        max_y = max((float(item["y"]) for item in locations), default=4.0)
        return {
            "available": True,
            "message": "Demonstration floor plan: no robot is connected.",
            "bounds": {
                "min_x": 0.0,
                "min_y": 0.0,
                "max_x": max(6.0, max_x + 1.0),
                "max_y": max(5.0, max_y + 1.0),
            },
            "walls": [
                [[0.4, 0.4], [5.6, 0.4], [5.6, 4.6], [0.4, 4.6], [0.4, 0.4]],
                [[3.1, 0.4], [3.1, 1.55]],
                [[3.1, 2.25], [3.1, 4.6]],
                [[0.4, 2.55], [1.35, 2.55]],
                [[2.1, 2.55], [3.1, 2.55]],
            ],
            "robot": {"x": 0.85, "y": 0.9, "yaw": 0.0},
            "locations": locations,
        }


class RobotWebApp:
    def __init__(
        self,
        location_store: LocationStore,
        dispatcher: GoalDispatcher,
        safety: SafetyAdapter | None = None,
        gateway: CommandGateway | None = None,
        map_provider: MapProvider | None = None,
    ):
        self.location_store = location_store
        self.dispatcher = dispatcher
        self.safety = safety or SafetyScenarioAdapter()
        self.gateway = gateway or CommandGateway(location_store)
        self.map_provider = map_provider or DemoMapProvider()
        # The server is threaded, so two operator requests can reach the
        # store at once. Reentrant because the mutating calls report the new
        # list by calling locations() while still holding it.
        self._store_lock = RLock()

    def locations(self) -> dict[str, list[dict[str, Any]]]:
        with self._store_lock:
            # LocationStore.locations() reads names and poses together. Asking
            # for names and then a goal per name was a read-modify-read, and a
            # concurrent removal between the two raised KeyError and took the
            # whole destination list down with it.
            return {
                "locations": [
                    {"name": name, "x": pose.x, "y": pose.y, "yaw": pose.yaw}
                    for name, pose in self.location_store.locations()
                ]
            }

    def _approve(self, name: str, x: float, y: float, yaw: float) -> NavigationGoal:
        """Validate a requested pose and save it. Caller holds the lock."""
        name = _text_field(name, "name")
        if not name.strip():
            raise BadRequest("name must not be empty")
        pose = Pose2D(
            _number_field(x, "x"), _number_field(y, "y"), _number_field(yaw, "yaw")
        )
        self.location_store.save_location(name, pose)
        return self.location_store.get_goal(name)

    def save_location(
        self, name: str, x: float, y: float, yaw: float = 0.0
    ) -> dict[str, Any]:
        """Approve a named pose. Saving an existing name overwrites it."""
        with self._store_lock:
            self._approve(name, x, y, yaw)
            return self.locations()

    def label_location(
        self, name: str, x: float, y: float, yaw: float = 0.0
    ) -> dict[str, Any]:
        """Approve a pose dropped on the floor plan, reporting just that one.

        Same approval path as save_location, so a label dragged onto the map
        is validated exactly like one typed into the form: a NaN coordinate is
        a destination the robot can be sent to and never arrive at.
        """
        with self._store_lock:
            goal = self._approve(name, x, y, yaw)
            return {
                "name": goal.location_name,
                "x": goal.pose.x,
                "y": goal.pose.y,
                "yaw": goal.pose.yaw,
            }

    def map_data(self) -> dict[str, Any]:
        """Draw the floor plan the configured provider knows about."""
        return self.map_provider.map_data(self.locations()["locations"])

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
        scenarios = getattr(self.safety, "SCENARIOS", None)
        if scenarios is None:
            # Pointed at a real gate there are no scenarios to pick: the
            # sensors decide. Refusing here keeps a stale demo button from
            # looking like it changed something about the robot.
            raise BadRequest("this console is connected to a robot: no demo scenarios")
        if scenario not in scenarios:
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
        self._refuse_motion()
        with self._store_lock:
            goal = self.location_store.get_goal(location_name)
            goal_id = self.dispatcher.send_goal(goal)
        return {"goal_id": goal_id, "location_name": goal.location_name}

    def _motion_refusal(self) -> str | None:
        """Why this console will not ask for motion right now, if it will not.

        Asked of the adapter rather than derived from the latch, because an
        adapter connected to a real gate has a third answer: it has not heard
        from the gate and so cannot say the latch is released.
        """
        describe = getattr(self.safety, "motion_refusal", None)
        if describe is not None:
            return describe()
        return (
            "the emergency stop is engaged"
            if self.safety.emergency_stop_engaged
            else None
        )

    def _refuse_motion(self) -> None:
        reason = self._motion_refusal()
        if reason is not None:
            raise RuntimeError(f"{reason}: the goal was not sent")

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

        if outcome.action in ("stop", "halt"):
            # A halt is a stop the recogniser mangled - "sto", "hal". It drops
            # the trip exactly as a stop does; what it does not do is ask for
            # the latch, which is the gateway's decision, not this console's.
            if status.state == "navigating" and status.goal_id is not None:
                self.dispatcher.cancel_goal(status.goal_id)
        elif outcome.action == "go_to":
            refusal = self._motion_refusal()
            if refusal is not None:
                return {"action": "refused", "response": f"I cannot move: {refusal}."}
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
            if path == "/api/map":
                return _Reply(200, app.map_data())
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
