import contextlib
import http.client
from http.server import ThreadingHTTPServer
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest

from robot_core import Pose2D
from robot_locations import LocationStore

from web_ui.app import (
    BadRequest,
    DemoGoalDispatcher,
    RobotWebApp,
    SafetyScenarioAdapter,
    make_handler,
)

WEB_ROOT = Path(__file__).resolve().parent.parent / "web"


class RobotWebAppTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("Kitchen", Pose2D(1.0, 2.0))
        self.dispatcher = DemoGoalDispatcher()
        self.safety = SafetyScenarioAdapter()
        self.app = RobotWebApp(store, self.dispatcher, self.safety)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_lists_only_approved_locations(self) -> None:
        self.assertEqual(
            self.app.locations(),
            {"locations": [{"name": "kitchen", "x": 1.0, "y": 2.0, "yaw": 0.0}]},
        )

    def test_sends_named_goal_and_exposes_status(self) -> None:
        result = self.app.send_goal(" KITCHEN ")

        self.assertEqual(result["location_name"], "kitchen")
        self.assertEqual(self.app.status()["goal"]["state"], "navigating")
        self.assertEqual(self.app.status()["goal"]["location_name"], "kitchen")

    def test_rejects_unknown_location(self) -> None:
        with self.assertRaises(KeyError):
            self.app.send_goal("bedroom")

    def test_cancels_active_goal(self) -> None:
        self.app.send_goal("kitchen")

        self.assertEqual(self.app.cancel_goal(), {"message": "Trip cancelled"})
        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

    def test_only_one_trip_can_be_active(self) -> None:
        self.app.send_goal("kitchen")

        with self.assertRaises(RuntimeError):
            self.app.send_goal("kitchen")

    def test_reports_safety_scenario_decision(self) -> None:
        self.app.set_safety_scenario("caution")

        status = self.app.status()

        self.assertEqual(status["safety_state"], "caution")
        self.assertEqual(status["safety"]["speed_scale"], 0.35)
        self.assertEqual(status["safety"]["nearest_obstacle_distance"], 0.6)

    def test_emergency_stop_wins_over_a_clear_path(self) -> None:
        self.app.set_safety_scenario("clear")

        status = self.app.engage_emergency_stop()

        self.assertEqual(status["safety_state"], "stop")
        self.assertEqual(status["safety_message"], "emergency stop active")

    def test_rejects_unknown_safety_scenario(self) -> None:
        with self.assertRaises(ValueError):
            self.app.set_safety_scenario("drive_fast")

    def test_voice_command_sends_an_approved_destination(self) -> None:
        result = self.app.handle_voice_command("go to the kitchen")

        self.assertEqual(result["action"], "go_to")
        self.assertEqual(result["location_name"], "kitchen")
        self.assertEqual(self.app.status()["goal"]["state"], "navigating")

    def test_voice_command_refusal_never_starts_a_trip(self) -> None:
        result = self.app.handle_voice_command("go to the moon")

        self.assertEqual(result["action"], "refused")
        self.assertNotIn("goal_id", result)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_voice_stop_cancels_an_active_trip(self) -> None:
        self.app.send_goal("kitchen")

        result = self.app.handle_voice_command("robot please stop")

        self.assertEqual(result["action"], "stop")
        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

    def test_voice_stop_is_safe_when_nothing_is_moving(self) -> None:
        result = self.app.handle_voice_command("stop")

        self.assertEqual(result["action"], "stop")
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_voice_reports_the_current_trip(self) -> None:
        self.app.send_goal("kitchen")

        result = self.app.handle_voice_command("where are you")

        self.assertEqual(result["action"], "report_location")
        self.assertEqual(result["response"], "I am on my way to the kitchen.")

    def test_emergency_stop_latch_survives_polls_and_scenario_changes(self) -> None:
        self.app.engage_emergency_stop()

        for _ in range(3):
            self.assertEqual(self.app.status()["safety_state"], "stop")

        # Picking a clear-path scenario must not release an engaged stop.
        self.app.set_safety_scenario("clear")

        self.assertEqual(self.app.status()["safety_state"], "stop")
        self.assertTrue(self.app.status()["safety"]["emergency_stop"])

    def test_only_an_explicit_reset_releases_the_stop(self) -> None:
        self.app.engage_emergency_stop()

        status = self.app.reset_emergency_stop()

        self.assertEqual(status["safety_state"], "clear")
        self.assertFalse(status["safety"]["emergency_stop"])

    def test_emergency_stop_abandons_the_active_trip(self) -> None:
        self.app.send_goal("kitchen")

        status = self.app.engage_emergency_stop()

        self.assertEqual(status["goal"]["state"], "cancelled")

    def test_no_goal_can_be_sent_while_stopped(self) -> None:
        self.app.engage_emergency_stop()

        with self.assertRaises(RuntimeError):
            self.app.send_goal("kitchen")

    def test_voice_cannot_move_the_robot_while_stopped(self) -> None:
        self.app.engage_emergency_stop()

        result = self.app.handle_voice_command("go to the kitchen")

        self.assertEqual(result["action"], "refused")
        self.assertNotIn("goal_id", result)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_saves_and_removes_named_locations(self) -> None:
        result = self.app.save_location(" Front Door ", 3.0, -1.0, 1.57)

        self.assertIn(
            {"name": "front door", "x": 3.0, "y": -1.0, "yaw": 1.57},
            result["locations"],
        )

        remaining = self.app.remove_location("front door")

        self.assertEqual([entry["name"] for entry in remaining["locations"]], ["kitchen"])

    def test_saved_locations_become_voice_destinations(self) -> None:
        self.app.save_location("front door", 3.0, -1.0)

        result = self.app.handle_voice_command("go to the front door")

        self.assertEqual(result["action"], "go_to")
        self.assertEqual(result["location_name"], "front door")

    def test_rejects_an_unnamed_or_unknown_location(self) -> None:
        with self.assertRaises(ValueError):
            self.app.save_location("   ", 1.0, 1.0)

        with self.assertRaises(KeyError):
            self.app.remove_location("garage")

    def test_stale_command_scenario_stops_robot(self) -> None:
        status = self.app.set_safety_scenario("stale_command")

        self.assertEqual(status["safety"]["state"], "stop")
        self.assertEqual(status["safety"]["reason"], "motion command timed out")


    def test_scenarios_no_longer_include_the_emergency_stop(self) -> None:
        # The stop is a latched control, not a sensor condition. Leaving it in
        # the scenario list would let a scenario click release an engaged stop.
        self.assertNotIn("emergency_stop", SafetyScenarioAdapter.SCENARIOS)

    def test_every_mutating_endpoint_reports_the_stop_latch(self) -> None:
        # Whatever an operator just did, the reply says whether the robot is
        # latched. A response shape that omitted it let a caller render a
        # panel with no latch state at all.
        self.app.engage_emergency_stop()

        for label, result in (
            ("engage", self.app.engage_emergency_stop()),
            ("scenario", self.app.set_safety_scenario("clear")),
            ("status", self.app.status()),
            ("reset", self.app.reset_emergency_stop()),
        ):
            with self.subTest(call=label):
                self.assertIn("safety", result)
                self.assertIn("emergency_stop", result["safety"])
                self.assertIn("goal", result)

    def test_a_scenario_change_never_reports_a_released_latch(self) -> None:
        self.app.engage_emergency_stop()

        for scenario in SafetyScenarioAdapter.SCENARIOS:
            with self.subTest(scenario=scenario):
                status = self.app.set_safety_scenario(scenario)

                self.assertTrue(status["safety"]["emergency_stop"])
                self.assertEqual(status["safety_state"], "stop")

    def test_rejects_non_text_fields_instead_of_crashing(self) -> None:
        # These reached LocationStore and the voice gateway as-is and died on
        # AttributeError, which the HTTP layer did not catch: the request got
        # no reply at all.
        for label, call in (
            ("save_location", lambda: self.app.save_location(123, 1.0, 1.0)),
            ("remove_location", lambda: self.app.remove_location(["kitchen"])),
            ("send_goal", lambda: self.app.send_goal(None)),
            ("handle_voice_command", lambda: self.app.handle_voice_command(42)),
            ("set_safety_scenario", lambda: self.app.set_safety_scenario({"c": True})),
        ):
            with self.subTest(call=label):
                with self.assertRaises(BadRequest):
                    call()

    def test_rejects_coordinates_that_are_not_finite_numbers(self) -> None:
        # JSON parses NaN and Infinity happily, and a pose at NaN is a
        # destination the robot can be given and never reach.
        for x, y, yaw in (
            (float("nan"), 1.0, 0.0),
            (1.0, float("inf"), 0.0),
            (1.0, 1.0, float("-inf")),
            ("3.0", 1.0, 0.0),
            (True, 1.0, 0.0),
            (None, 1.0, 0.0),
        ):
            with self.subTest(x=x, y=y, yaw=yaw):
                with self.assertRaises(BadRequest):
                    self.app.save_location("somewhere", x, y, yaw)

        self.assertNotIn(
            "somewhere", [entry["name"] for entry in self.app.locations()["locations"]]
        )

    def test_an_engaged_stop_is_the_reason_a_goal_is_refused(self) -> None:
        # The lookup used to happen first, so a stopped robot asked for an
        # unknown place blamed the name and never mentioned the stop.
        self.app.engage_emergency_stop()

        with self.assertRaises(RuntimeError) as caught:
            self.app.send_goal("no such place")

        self.assertIn("emergency stop", str(caught.exception))

    def test_concurrent_saves_and_reads_keep_the_list_readable(self) -> None:
        # locations() takes a name snapshot and then looks each name up. A
        # removal landing between the two raised KeyError out of a GET, which
        # the handler did not catch, and the operator's list vanished.
        for index in range(20):
            self.app.save_location(f"place {index}", float(index), 0.0)

        failures: list[str] = []
        stop = threading.Event()

        def churn(worker: int) -> None:
            index = 0
            while not stop.is_set():
                name = f"churn {worker} {index % 5}"
                try:
                    self.app.save_location(name, 1.0, 1.0)
                    self.app.remove_location(name)
                except Exception as error:  # noqa: BLE001 - recorded and asserted
                    failures.append(f"{type(error).__name__}: {error}")
                index += 1

        def read() -> None:
            while not stop.is_set():
                try:
                    self.app.locations()
                except Exception as error:  # noqa: BLE001 - recorded and asserted
                    failures.append(f"{type(error).__name__}: {error}")

        workers = [threading.Thread(target=churn, args=(worker,)) for worker in range(4)]
        workers += [threading.Thread(target=read) for _ in range(4)]
        for worker in workers:
            worker.start()
        stop.wait(1.0)
        stop.set()
        for worker in workers:
            worker.join()

        self.assertEqual(failures, [])


class RobotHttpApiTests(unittest.TestCase):
    """The HTTP layer, which is where malformed operator input arrives."""

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("kitchen", Pose2D(1.0, 2.0))
        self.app = RobotWebApp(store, DemoGoalDispatcher(), SafetyScenarioAdapter())
        self.server = ThreadingHTTPServer(
            ("127.0.0.1", 0), make_handler(self.app, WEB_ROOT)
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_address[1]

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)
        self.directory.cleanup()

    def call(self, method, path, body=None, headers=None):
        """Return (status, decoded body). Never raises on an error response."""
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        try:
            request_headers = {"Content-Type": "application/json"}
            if headers:
                request_headers.update(headers)
            connection.request(method, path, body=body, headers=request_headers)
            response = connection.getresponse()
            raw = response.read().decode("utf-8")
            try:
                return response.status, json.loads(raw)
            except json.JSONDecodeError:
                return response.status, raw
        finally:
            connection.close()

    def test_status_reports_the_latch(self) -> None:
        status, body = self.call("GET", "/api/status")

        self.assertEqual(status, 200)
        self.assertFalse(body["safety"]["emergency_stop"])

        self.assertEqual(self.call("POST", "/api/emergency_stop")[0], 200)
        _, body = self.call("GET", "/api/status")

        self.assertTrue(body["safety"]["emergency_stop"])
        self.assertEqual(body["safety_state"], "stop")

    def test_polling_never_releases_the_latch(self) -> None:
        self.call("POST", "/api/emergency_stop")

        for _ in range(5):
            _, body = self.call("GET", "/api/status")
            self.assertTrue(body["safety"]["emergency_stop"])

        # Nor does a scenario change, which is the other thing the UI posts.
        self.call("POST", "/api/safety/scenario", '{"scenario":"clear"}')
        _, body = self.call("GET", "/api/status")

        self.assertTrue(body["safety"]["emergency_stop"])

    def test_only_the_reset_endpoint_releases_the_latch(self) -> None:
        self.call("POST", "/api/emergency_stop")

        status, body = self.call("POST", "/api/emergency_stop/reset")

        self.assertEqual(status, 200)
        self.assertFalse(body["safety"]["emergency_stop"])

    def test_malformed_requests_always_get_an_answer(self) -> None:
        # Each of these used to escape the handler as an uncaught exception,
        # which closed the connection with no response: the console showed a
        # bare network error and kept displaying its last known safety state.
        cases = [
            ("non-text name", "/api/locations", json.dumps({"name": 1, "x": 0, "y": 0})),
            ("non-text transcript", "/api/voice", json.dumps({"transcript": 42})),
            ("non-text remove", "/api/locations/remove", json.dumps({"name": [1]})),
            ("nan coordinate", "/api/locations", '{"name":"a","x":NaN,"y":0}'),
            ("string coordinate", "/api/locations", '{"name":"a","x":"abc","y":0}'),
            ("blank name", "/api/locations", '{"name":"   ","x":0,"y":0}'),
            ("json array body", "/api/locations", "[1,2,3]"),
            ("json string body", "/api/locations", '"kitchen"'),
            ("json null body", "/api/locations", "null"),
            ("broken json", "/api/locations", "{oops"),
            ("empty body", "/api/locations", ""),
            ("missing field", "/api/goals", "{}"),
            ("unknown scenario", "/api/safety/scenario", '{"scenario":"drive_fast"}'),
        ]

        for label, path, body in cases:
            with self.subTest(case=label):
                status, payload = self.call("POST", path, body)

                self.assertEqual(status, 400, f"{label} -> {payload}")
                self.assertIn("error", payload)

    def test_a_bad_content_length_is_a_bad_request_not_a_conflict(self) -> None:
        status, payload = self.call(
            "POST", "/api/locations", '{"name":"a","x":0,"y":0}',
            {"Content-Length": "not-a-number"},
        )

        self.assertEqual(status, 400)
        self.assertIn("Content-Length", payload["error"])

    def test_an_oversized_body_is_refused_rather_than_read(self) -> None:
        status, payload = self.call(
            "POST", "/api/locations", '{"name":"a","x":0,"y":0}',
            {"Content-Length": str(64 * 1024 + 1)},
        )

        self.assertEqual(status, 400)
        self.assertIn("larger than", payload["error"])

    def test_refusals_that_are_about_robot_state_are_conflicts(self) -> None:
        # 400 says "fix your request", 409 says "the robot will not do that
        # right now". An operator needs to be able to tell those apart.
        status, _ = self.call("POST", "/api/goals/cancel")
        self.assertEqual(status, 409)

        self.call("POST", "/api/emergency_stop")
        status, payload = self.call("POST", "/api/goals", '{"location_name":"kitchen"}')

        self.assertEqual(status, 409)
        self.assertIn("emergency stop", payload["error"])

    def test_unknown_routes_answer_json(self) -> None:
        for method, path in (("GET", "/api/nope"), ("POST", "/api/nope")):
            with self.subTest(method=method, path=path):
                status, payload = self.call(method, path)

                self.assertEqual(status, 404)
                self.assertEqual(payload, {"error": "not found"})

    def test_static_console_is_served(self) -> None:
        for path in ("/", "/index.html", "/app.js", "/styles.css"):
            with self.subTest(path=path):
                status, _ = self.call("GET", path)

                self.assertEqual(status, 200)

    def test_a_failing_status_read_still_answers(self) -> None:
        # The panel can only be honest about the robot if a broken read comes
        # back as an error rather than as a dropped connection.
        def explode() -> dict:
            raise OSError("the safety adapter is gone")

        self.app.status = explode

        # The handler logs the traceback to stderr on purpose; swallow it so
        # a deliberate failure does not look like a broken test run.
        with contextlib.redirect_stderr(io.StringIO()):
            status, payload = self.call("GET", "/api/status")

        self.assertEqual(status, 500)
        self.assertIn("error", payload)


class ConsoleMarkupTests(unittest.TestCase):
    """What the console asserts before it has heard from the robot.

    The page ships with static text in the safety panel, and that text is on
    screen until the first poll returns - and again whenever one fails. It
    must not claim the stop latch is released, because nothing has said so.
    """

    def setUp(self) -> None:
        self.markup = (WEB_ROOT / "index.html").read_text()
        self.script = (WEB_ROOT / "app.js").read_text()

    def test_the_shipped_latch_readout_does_not_claim_released(self) -> None:
        latch = self.markup.split('id="latch-value"')[1].split("</dd>")[0]

        self.assertNotIn("released", latch)
        self.assertIn("unknown", latch)

    def test_the_reset_control_starts_hidden(self) -> None:
        reset = self.markup.split('id="estop-reset"')[1].split(">")[0]

        # "Reset stop" asserts the latch is engaged. Before any status read
        # that is not something the page knows.
        self.assertIn("hidden", reset)

    def test_a_failed_status_read_marks_the_latch_unknown(self) -> None:
        unavailable = self.script.split("function showStatusUnavailable")[1]

        self.assertIn("renderLatch(null)", unavailable)

    def test_an_unknown_latch_never_offers_a_reset(self) -> None:
        unknown_branch = self.script.split("if (engaged === null)")[1].split("return;")[0]

        self.assertIn("estopReset.hidden = true", unknown_branch)

    def test_a_missing_latch_field_is_treated_as_unknown(self) -> None:
        # Not as released: a status payload without the field tells the
        # console nothing, and "nothing" must not render as safe.
        self.assertIn('typeof safety.emergency_stop === "boolean"', self.script)


if __name__ == "__main__":
    unittest.main()
