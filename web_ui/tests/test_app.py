import contextlib
import http.client
from http.server import ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from robot_core import Pose2D
from robot_locations import LocationStore

import web_ui.app as app_module
from web_ui.app import (
    BadRequest,
    DemoGoalDispatcher,
    FreeLLMAPIChat,
    RobotWebApp,
    SafetyScenarioAdapter,
    make_handler,
    make_resident_handler,
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

    def test_map_is_built_from_saved_locations(self) -> None:
        map_data = self.app.map_data()

        self.assertEqual(map_data["locations"][0]["name"], "kitchen")
        self.assertGreaterEqual(map_data["bounds"]["max_x"], 6.0)
        self.assertTrue(map_data["walls"])

    def test_can_save_a_label_from_the_map(self) -> None:
        result = self.app.label_location("Reading nook", 3.25, 1.5)

        self.assertEqual(result["name"], "reading nook")
        self.assertEqual(self.app.locations()["locations"][1]["x"], 3.25)

    def test_a_label_dropped_on_the_map_is_validated_like_any_other(self) -> None:
        # The map label path shares save_location's approval, so a NaN
        # coordinate is refused rather than stored as a destination the robot
        # can be sent to and never arrive at.
        with self.assertRaises(ValueError):
            self.app.label_location("nowhere", float("nan"), 1.0)

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

    def test_free_text_uses_chat_without_issuing_robot_commands(self) -> None:
        chat = Mock()
        chat.reply.return_value = "A robot console can help manage saved destinations."
        app = RobotWebApp(
            self.app.location_store,
            self.dispatcher,
            self.safety,
            chat_responder=chat,
        )

        result = app.handle_voice_command("What can this console do?")

        self.assertEqual(result["action"], "chat")
        self.assertEqual(result["response"], chat.reply.return_value)
        self.assertEqual(self.dispatcher.status().state, "ready")
        chat.reply.assert_called_once_with("What can this console do?")

    def test_robot_commands_do_not_call_chat(self) -> None:
        chat = Mock()
        app = RobotWebApp(
            self.app.location_store,
            self.dispatcher,
            self.safety,
            chat_responder=chat,
        )

        result = app.handle_voice_command("go to the kitchen")

        self.assertEqual(result["action"], "go_to")
        chat.reply.assert_not_called()

    def test_chat_api_uses_bearer_auth_and_openai_compatible_payload(self) -> None:
        response = io.BytesIO(
            b'{"choices":[{"message":{"content":"Hello from chat."}}]}'
        )
        client = FreeLLMAPIChat("test-key", model="auto")

        with patch("web_ui.app.urlopen", return_value=response) as request_call:
            answer = client.reply("Hello")

        self.assertEqual(answer, "Hello from chat.")
        request = request_call.call_args.args[0]
        self.assertEqual(request.full_url, "http://localhost:3001/v1/chat/completions")
        self.assertEqual(request.get_header("Authorization"), "Bearer test-key")
        self.assertEqual(json.loads(request.data)["model"], "auto")

    def test_voice_stop_cancels_an_active_trip(self) -> None:
        self.app.send_goal("kitchen")

        result = self.app.handle_voice_command("robot please stop")

        self.assertEqual(result["action"], "stop")
        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

    def test_a_mangled_stop_cancels_the_trip_too(self) -> None:
        # "sto" is what an engine makes of a shouted stop. Answering it
        # politely while the robot kept driving would be the worst outcome.
        self.app.send_goal("kitchen")

        result = self.app.handle_voice_command("sto")

        self.assertEqual(result["action"], "halt")
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

    def test_a_stop_pressed_as_the_trip_ends_still_answers(self) -> None:
        # The trip can finish between the stop reading it as active and
        # cancelling it. The latch is down either way; answering the STOP
        # button with an error would tell the operator it had not worked.
        self.app.send_goal("kitchen")

        def already_over(goal_id: str) -> None:
            raise ValueError("no matching active goal")

        self.dispatcher.cancel_goal = already_over

        status = self.app.engage_emergency_stop()
        self.assertTrue(status["safety"]["emergency_stop"])
        self.assertEqual(self.app.handle_voice_command("stop")["action"], "stop")

    def test_a_goal_racing_the_stop_is_not_left_behind_the_latch(self) -> None:
        # The goal passes the latch check, and the stop lands before it is
        # dispatched. Left alone, that trip sits behind the latch and drives
        # off the moment someone resets it.
        stopper = []
        check = self.safety.motion_refusal

        def refusal_as_the_stop_is_pressed():
            answer = check()
            stopper.append(threading.Thread(target=self.app.engage_emergency_stop))
            stopper[0].start()
            stopper[0].join(timeout=0.2)
            return answer

        self.safety.motion_refusal = refusal_as_the_stop_is_pressed

        self.app.send_goal("kitchen")
        stopper[0].join(timeout=5)

        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

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

    def test_another_site_cannot_release_the_stop_or_send_the_robot(self) -> None:
        # What a page elsewhere in the operator's browser can send without
        # the console's permission: a form post or a no-cors fetch, neither
        # of which can be JSON, and a foreign Origin on anything it sends.
        self.call("POST", "/api/emergency_stop")
        cross_site = [
            {"Content-Type": "text/plain"},
            {"Content-Type": "application/x-www-form-urlencoded"},
            {"Origin": "http://evil.example"},
            {"Origin": "null"},
        ]

        for headers in cross_site:
            with self.subTest(headers=headers):
                status, _ = self.call("POST", "/api/emergency_stop/reset", "", headers)
                self.assertEqual(status, 403)
                status, _ = self.call(
                    "POST", "/api/goals", '{"location_name":"kitchen"}', headers
                )
                self.assertEqual(status, 403)

        _, body = self.call("GET", "/api/status")
        self.assertTrue(body["safety"]["emergency_stop"])
        self.assertEqual(body["goal"]["state"], "ready")

    def test_the_stop_is_engaged_whoever_asks(self) -> None:
        status, body = self.call(
            "POST", "/api/emergency_stop", "",
            {"Content-Type": "text/plain", "Origin": "http://evil.example"},
        )

        self.assertEqual(status, 200)
        self.assertTrue(body["safety"]["emergency_stop"])

    def test_a_trip_is_cancelled_whoever_asks(self) -> None:
        # Cancelling only ever ends a trip, so like the stop it must never
        # fail on a header.
        self.call("POST", "/api/goals", '{"location_name":"kitchen"}')

        status, _ = self.call(
            "POST", "/api/goals/cancel", "",
            {"Content-Type": "text/plain", "Origin": "http://evil.example"},
        )

        self.assertEqual(status, 200)
        _, body = self.call("GET", "/api/status")
        self.assertEqual(body["goal"]["state"], "cancelled")

    def test_the_consoles_own_page_can_still_reset(self) -> None:
        self.call("POST", "/api/emergency_stop")

        status, body = self.call(
            "POST", "/api/emergency_stop/reset", "",
            {"Origin": f"http://127.0.0.1:{self.port}"},
        )

        self.assertEqual(status, 200)
        self.assertFalse(body["safety"]["emergency_stop"])

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


class ResidentHttpApiTests(unittest.TestCase):
    """The resident's tablet page: go somewhere, cancel, stop. Nothing else.

    The page hides what it does not offer, but hiding a button is not a
    control. Each refusal here is the server's, so a resident's tablet - or
    anything else on the home network that finds the port - cannot release
    the stop, change the saved rooms, or reach the developer console.
    """

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("kitchen", Pose2D(1.0, 2.0))
        self.app = RobotWebApp(store, DemoGoalDispatcher(), SafetyScenarioAdapter())
        self.server = ThreadingHTTPServer(
            ("127.0.0.1", 0), make_resident_handler(self.app, WEB_ROOT)
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

    def test_the_resident_page_and_its_assets_are_served(self) -> None:
        for path in ("/", "/resident.html", "/resident.js", "/resident.css"):
            with self.subTest(path=path):
                status, _ = self.call("GET", path)
                self.assertEqual(status, 200)

    def test_the_root_is_the_resident_page_not_the_console(self) -> None:
        _, body = self.call("GET", "/")

        self.assertIn("resident.js", body)
        self.assertNotIn("app.js", body)

    def test_the_developer_console_is_not_served(self) -> None:
        for path in ("/index.html", "/app.js", "/styles.css"):
            with self.subTest(path=path):
                status, _ = self.call("GET", path)
                self.assertEqual(status, 404)

    def test_the_saved_rooms_and_status_can_be_read(self) -> None:
        status, body = self.call("GET", "/api/locations")
        self.assertEqual(status, 200)
        self.assertEqual(body["locations"][0]["name"], "kitchen")

        status, body = self.call("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertFalse(body["safety"]["emergency_stop"])

    def test_the_map_is_not_offered(self) -> None:
        status, _ = self.call("GET", "/api/map")

        self.assertEqual(status, 404)

    def test_a_trip_can_be_started_and_cancelled(self) -> None:
        status, body = self.call("POST", "/api/goals", '{"location_name":"kitchen"}')
        self.assertEqual(status, 201)
        self.assertEqual(body["location_name"], "kitchen")

        status, _ = self.call("POST", "/api/goals/cancel")
        self.assertEqual(status, 200)
        self.assertEqual(self.app.status()["goal"]["state"], "cancelled")

    def test_the_stop_can_be_engaged(self) -> None:
        status, body = self.call("POST", "/api/emergency_stop")

        self.assertEqual(status, 200)
        self.assertTrue(body["safety"]["emergency_stop"])
        self.assertTrue(self.app.safety.emergency_stop_engaged)

    def test_the_stop_cannot_be_reset_from_here(self) -> None:
        self.call("POST", "/api/emergency_stop")

        status, _ = self.call("POST", "/api/emergency_stop/reset")

        self.assertEqual(status, 404)
        # Refused, not merely answered: the latch is still down.
        self.assertTrue(self.app.safety.emergency_stop_engaged)

    def test_the_reset_is_refused_even_from_the_pages_own_origin(self) -> None:
        self.call("POST", "/api/emergency_stop")

        status, _ = self.call(
            "POST", "/api/emergency_stop/reset", "",
            {"Origin": f"http://127.0.0.1:{self.port}"},
        )

        self.assertEqual(status, 404)
        self.assertTrue(self.app.safety.emergency_stop_engaged)

    def test_the_safety_scenario_cannot_be_changed(self) -> None:
        status, _ = self.call("POST", "/api/safety/scenario", '{"scenario":"stop"}')

        self.assertEqual(status, 404)
        self.assertEqual(self.app.status()["safety"]["scenario"], "clear")

    def test_rooms_cannot_be_saved(self) -> None:
        status, _ = self.call(
            "POST", "/api/locations", '{"name":"garden","x":0,"y":0}'
        )

        self.assertEqual(status, 404)
        names = [place["name"] for place in self.app.locations()["locations"]]
        self.assertEqual(names, ["kitchen"])

    def test_rooms_cannot_be_removed(self) -> None:
        status, _ = self.call("POST", "/api/locations/remove", '{"name":"kitchen"}')

        self.assertEqual(status, 404)
        self.assertEqual(len(self.app.locations()["locations"]), 1)

    def test_voice_commands_are_not_accepted(self) -> None:
        status, _ = self.call("POST", "/api/voice", '{"transcript":"go to the kitchen"}')

        self.assertEqual(status, 404)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_unknown_routes_are_refused(self) -> None:
        for method, path in (("GET", "/api/nope"), ("POST", "/api/nope"), ("GET", "/nope")):
            with self.subTest(method=method, path=path):
                status, payload = self.call(method, path)
                self.assertEqual(status, 404)
                self.assertEqual(payload, {"error": "not found"})

    def test_another_site_cannot_send_the_robot(self) -> None:
        for headers in (
            {"Content-Type": "text/plain"},
            {"Content-Type": "application/x-www-form-urlencoded"},
            {"Origin": "http://evil.example"},
        ):
            with self.subTest(headers=headers):
                status, _ = self.call(
                    "POST", "/api/goals", '{"location_name":"kitchen"}', headers
                )
                self.assertEqual(status, 403)

        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_the_stop_is_engaged_whoever_asks(self) -> None:
        status, _ = self.call(
            "POST", "/api/emergency_stop", "",
            {"Content-Type": "text/plain", "Origin": "http://evil.example"},
        )

        self.assertEqual(status, 200)
        self.assertTrue(self.app.safety.emergency_stop_engaged)

    def test_one_backend_serves_both_pages(self) -> None:
        # A stop pressed on the tablet is the stop the developer console sees,
        # and only the developer console can release it.
        console = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(self.app, WEB_ROOT))
        thread = threading.Thread(target=console.serve_forever, daemon=True)
        thread.start()
        try:
            self.call("POST", "/api/emergency_stop")
            connection = http.client.HTTPConnection(
                "127.0.0.1", console.server_address[1], timeout=5
            )
            connection.request("GET", "/api/status")
            body = json.loads(connection.getresponse().read())
            connection.close()
        finally:
            console.shutdown()
            console.server_close()
            thread.join(timeout=5)

        self.assertTrue(body["safety"]["emergency_stop"])

    def test_other_methods_are_refused(self) -> None:
        for method in ("HEAD", "OPTIONS", "PUT", "DELETE", "PATCH"):
            with self.subTest(method=method):
                status, _ = self.call(method, "/api/emergency_stop/reset")
                self.assertEqual(status, 405)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_near_misses_of_the_reset_are_refused(self) -> None:
        self.call("POST", "/api/emergency_stop")

        for path in (
            "/api/emergency_stop/reset/",
            "/api/emergency_stop/reset?x=1",
            "//api/emergency_stop/reset",
            "/api/emergency_stop/../emergency_stop/reset",
            "/API/EMERGENCY_STOP/RESET",
        ):
            with self.subTest(path=path):
                status, _ = self.call("POST", path)
                self.assertEqual(status, 404)
        self.assertTrue(self.app.safety.emergency_stop_engaged)

    def test_a_query_string_does_not_widen_the_allow_list(self) -> None:
        status, _ = self.call("GET", "/api/map?x=1")

        self.assertEqual(status, 404)

    def test_a_rebound_name_cannot_read_or_send_the_robot(self) -> None:
        # DNS rebinding: a page on evil.example re-points its own name at
        # this tablet server. Its Origin then matches its Host, so only the
        # Host itself gives it away.
        rebound = {
            "Host": f"evil.example:{self.port}",
            "Origin": f"http://evil.example:{self.port}",
        }

        for method, path in (("GET", "/"), ("GET", "/api/status"), ("GET", "/api/locations")):
            with self.subTest(method=method, path=path):
                status, _ = self.call(method, path, headers=rebound)
                self.assertEqual(status, 421)
        status, _ = self.call("POST", "/api/goals", '{"location_name":"kitchen"}', rebound)
        self.assertEqual(status, 421)
        self.assertEqual(self.app.status()["goal"]["state"], "ready")

    def test_a_rebound_name_can_still_stop_and_cancel(self) -> None:
        # The same fail-safe exemption as the cross-site check: both only
        # ever end motion, and refusing a real one is the dangerous mistake.
        rebound = {"Host": f"evil.example:{self.port}", "Content-Type": "text/plain"}
        self.app.send_goal("kitchen")

        self.assertEqual(self.call("POST", "/api/goals/cancel", "", rebound)[0], 200)
        self.assertEqual(self.call("POST", "/api/emergency_stop", "", rebound)[0], 200)
        self.assertTrue(self.app.safety.emergency_stop_engaged)

    def test_ip_literals_and_localhost_are_accepted_hosts(self) -> None:
        for host in (f"localhost:{self.port}", f"192.168.1.20:{self.port}", "[::1]:8081"):
            with self.subTest(host=host):
                status, _ = self.call("GET", "/api/status", headers={"Host": host})
                self.assertEqual(status, 200)

    def test_every_reply_carries_the_security_headers(self) -> None:
        for method, path in (("GET", "/"), ("GET", "/api/status"), ("POST", "/api/nope")):
            with self.subTest(method=method, path=path):
                connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
                connection.request(method, path, body="", headers={"Content-Type": "application/json"})
                response = connection.getresponse()
                response.read()
                connection.close()
                self.assertEqual(response.getheader("X-Frame-Options"), "DENY")
                self.assertEqual(response.getheader("Referrer-Policy"), "no-referrer")
                policy = response.getheader("Content-Security-Policy")
                self.assertIn("default-src 'self'", policy)
                self.assertIn("frame-ancestors 'none'", policy)


class AllowedHostsTests(unittest.TestCase):
    """A name the household chose, e.g. robot.local, is accepted when listed."""

    def serve(self, handler) -> int:
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        def stop() -> None:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)

        self.addCleanup(stop)
        return server.server_address[1]

    def get(self, port: int, host: str) -> int:
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        connection.request("GET", "/api/status", headers={"Host": host})
        status = connection.getresponse().status
        connection.close()
        return status

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        store = LocationStore(Path(self.directory.name) / "locations.json")
        self.app = RobotWebApp(store, DemoGoalDispatcher(), SafetyScenarioAdapter())

    def test_a_listed_name_is_accepted_on_both_servers(self) -> None:
        for handler in (
            make_handler(self.app, WEB_ROOT, allowed_hosts={"robot.local"}),
            make_resident_handler(self.app, WEB_ROOT, allowed_hosts={"robot.local"}),
        ):
            port = self.serve(handler)
            with self.subTest(handler=handler):
                self.assertEqual(self.get(port, f"robot.local:{port}"), 200)
                self.assertEqual(self.get(port, f"Robot.Local:{port}"), 200)
                self.assertEqual(self.get(port, f"evil.example:{port}"), 421)

    def test_the_developer_console_refuses_a_rebound_name_too(self) -> None:
        port = self.serve(make_handler(self.app, WEB_ROOT))

        self.assertEqual(self.get(port, f"evil.example:{port}"), 421)
        self.assertEqual(self.get(port, f"127.0.0.1:{port}"), 200)


class ConnectedAdapter:
    """A safety adapter shaped like one wired to a real gate.

    No SCENARIOS and a refusal that is not the latch: the two things a demo
    adapter never has, and the two the console has to cope with. Deliberately
    a plain object, so nothing in web_ui/tests needs rclpy.
    """

    def __init__(self, refusal: str | None = None) -> None:
        self.refusal = refusal

    @property
    def emergency_stop_engaged(self) -> bool:
        return self.refusal is not None

    def motion_refusal(self) -> str | None:
        return self.refusal

    def engage_emergency_stop(self) -> None:
        self.refusal = "the emergency stop is engaged"

    def reset_emergency_stop(self) -> None:
        self.refusal = None

    def status(self) -> dict:
        return {"state": "unknown", "reason": "no gate here", "emergency_stop": None}


class FlatMapProvider:
    def __init__(self) -> None:
        self.asked_with = None

    def map_data(self, locations):
        self.asked_with = locations
        return {"available": False, "walls": [], "robot": None, "locations": locations}


class PluggedInAdapterTests(unittest.TestCase):
    """The seams the ROS adapters plug into, exercised without ROS."""

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        store = LocationStore(Path(self.directory.name) / "locations.json")
        store.save_location("Kitchen", Pose2D(1.0, 2.0))
        self.safety = ConnectedAdapter()
        self.map_provider = FlatMapProvider()
        self.app = RobotWebApp(
            store,
            DemoGoalDispatcher(),
            safety=self.safety,
            map_provider=self.map_provider,
        )

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_the_map_comes_from_the_provider_with_the_approved_places(self) -> None:
        payload = self.app.map_data()

        self.assertFalse(payload["available"])
        self.assertEqual(self.map_provider.asked_with[0]["name"], "kitchen")

    def test_an_adapter_with_no_scenarios_refuses_the_demo_control(self) -> None:
        with self.assertRaises(BadRequest):
            self.app.set_safety_scenario("clear")

    def test_the_adapters_own_reason_is_what_refuses_a_goal(self) -> None:
        # Not the latch: an adapter that cannot see the gate refuses for a
        # different reason, and saying "the stop is engaged" would send an
        # operator to look at the wrong thing.
        self.safety.refusal = "the safety gate has not reported its state"

        with self.assertRaises(RuntimeError) as caught:
            self.app.send_goal("kitchen")

        self.assertIn("has not reported its state", str(caught.exception))

    def test_a_voice_command_is_refused_for_the_adapters_reason(self) -> None:
        self.safety.refusal = "the safety gate has not reported its state"

        result = self.app.handle_voice_command("take me to the kitchen")

        self.assertEqual(result["action"], "refused")
        self.assertIn("has not reported its state", result["response"])

    def test_a_goal_goes_through_once_nothing_refuses_it(self) -> None:
        result = self.app.send_goal("kitchen")

        self.assertEqual(result["location_name"], "kitchen")

    def test_the_demo_adapters_stay_the_default(self) -> None:
        # web_ui has to keep running with no ROS installed, so an app built
        # without adapters gets the local ones.
        app = RobotWebApp(self.app.location_store, DemoGoalDispatcher())

        self.assertIsInstance(app.safety, SafetyScenarioAdapter)
        self.assertTrue(app.map_data()["walls"])


def contrast(foreground: str, background: str) -> float:
    """WCAG 2 contrast ratio between two #rrggbb colours."""

    def luminance(colour: str) -> float:
        channels = [int(colour[i:i + 2], 16) / 255 for i in (1, 3, 5)]
        r, g, b = (
            c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4
            for c in channels
        )
        return 0.2126 * r + 0.7152 * g + 0.0722 * b

    light, dark = sorted((luminance(foreground), luminance(background)), reverse=True)
    return (light + 0.05) / (dark + 0.05)


class ConsoleMarkupTests(unittest.TestCase):
    """What the console asserts before it has heard from the robot.

    The page ships with static text in the safety panel, and that text is on
    screen until the first poll returns - and again whenever one fails. It
    must not claim the stop latch is released, because nothing has said so.
    """

    def setUp(self) -> None:
        self.markup = (WEB_ROOT / "index.html").read_text()
        self.script = (WEB_ROOT / "app.js").read_text()
        self.styles = (WEB_ROOT / "styles.css").read_text()

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

    def test_the_page_sends_json_wherever_the_server_requires_it(self) -> None:
        # The server refuses anything else as cross-site, so a call the page
        # makes without it is a reset or cancel button that silently 403s.
        for path in ("/api/goals/cancel", "/api/emergency_stop/reset"):
            with self.subTest(path=path):
                call = self.script.split(f'request("{path}"')[1].split("})")[0]
                self.assertIn('"Content-Type": "application/json"', call)

    def test_the_maps_places_are_not_hidden_inside_an_image(self) -> None:
        # role="img" makes everything inside it presentational, so a screen
        # reader never found the "Go to" buttons drawn on the map.
        floor_map = self.markup.split('<svg id="floor-map"')[1].split(">")[0]

        self.assertNotIn('role="img"', floor_map)

    def test_a_redraw_gives_the_focused_place_its_focus_back(self) -> None:
        render = self.script.split("function renderMap")[1].split("\n}\n")[0]

        self.assertIn("focusedName", render.split("replaceChildren")[0])
        self.assertIn("group.focus()", render)

    def test_a_press_on_a_place_is_not_a_new_label(self) -> None:
        press = self.script.split('addEventListener("pointerdown"')[1].split("});")[0]

        self.assertIn('closest(".map-location")) return', press)

    def test_a_failed_map_read_stops_drawing_the_robot(self) -> None:
        poll = self.script.split("setInterval(() => {\n  loadMap()")[1].split("MAP_POLL_MS")[0]

        self.assertIn("robot: null", poll)

    def test_text_and_fields_meet_the_contrast_minimums(self) -> None:
        # WCAG AA: 4.5:1 for text, 3:1 for the edge that shows a field is one.
        colours = dict(re.findall(r"--([a-z-]+): (#[0-9a-f]{6});", self.styles))

        for background in ("panel", "paper", "teal-soft"):
            with self.subTest(background=background):
                ratio = contrast(colours["muted"], colours[background])
                self.assertGreaterEqual(ratio, 4.5)
        self.assertGreaterEqual(contrast(colours["field"], colours["panel"]), 3.0)
        self.assertEqual(self.styles.count("border: 1px solid var(--field)"), 3)

    def test_a_missing_latch_field_is_treated_as_unknown(self) -> None:
        # Not as released: a status payload without the field tells the
        # console nothing, and "nothing" must not render as safe.
        self.assertIn('typeof safety.emergency_stop === "boolean"', self.script)


class SynchronousThread:
    """Runs its target on start(), so what it did is settled by the next line."""

    def __init__(self, target, **_kwargs) -> None:
        self.target = target

    def start(self) -> None:
        self.target()


class MainTests(unittest.TestCase):
    def test_main_serves_both_pages_from_one_backend(self) -> None:
        # One RobotWebApp means one latch: a stop pressed on the tablet is
        # the stop the console shows, not a second, private one.
        console, resident = Mock(), Mock()
        console.serve_forever.side_effect = KeyboardInterrupt
        with (
            patch("web_ui.app.ThreadingHTTPServer", side_effect=[console, resident]) as server,
            patch("web_ui.app.make_handler") as console_handler,
            patch("web_ui.app.make_resident_handler") as resident_handler,
            patch("web_ui.app.Thread", SynchronousThread),
            patch.dict("os.environ", {}, clear=False),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            # The defaults are under test, so neither override may leak in.
            os.environ.pop("RESIDENT_HOST", None)
            os.environ.pop("RESIDENT_PORT", None)
            os.environ["ALLOWED_HOSTS"] = "Robot.Local, tablet.home"
            app_module.main()

        addresses = [call.args[0] for call in server.call_args_list]
        self.assertEqual(addresses, [("127.0.0.1", 8080), ("127.0.0.1", 8081)])
        self.assertIs(console_handler.call_args.args[0], resident_handler.call_args.args[0])
        self.assertEqual(
            console_handler.call_args.kwargs["allowed_hosts"], {"robot.local", "tablet.home"}
        )
        self.assertEqual(resident_handler.call_args.args[2], {"robot.local", "tablet.home"})
        resident.serve_forever.assert_called_once()
        resident.shutdown.assert_called_once()
        resident.server_close.assert_called_once()
        console.server_close.assert_called_once()


@unittest.skipUnless(shutil.which("node"), "node is not installed")
class ResidentScriptTests(unittest.TestCase):
    """resident.js driven in Node against a fake robot and a fake clock.

    The harness parses the real resident.html into a small fake DOM, so these
    exercise the shipped page: the STOP that has to be confirmed, the races a
    slow robot opens up, and what the resident is told at each step.
    """

    def test_the_page_behaves(self) -> None:
        harness = Path(__file__).resolve().parent / "resident_harness.js"
        run = subprocess.run(
            ["node", str(harness)], capture_output=True, text=True, timeout=120
        )
        self.assertEqual(run.returncode, 0, run.stderr)
        results = json.loads(run.stdout)

        self.assertGreater(len(results), 15)
        for scenario, failure in results.items():
            with self.subTest(scenario=scenario):
                self.assertIsNone(failure, failure)


class ResidentMarkupTests(unittest.TestCase):
    """What the resident's page says, and what it is able to ask for.

    The server refuses everything the page is not meant to do; these keep the
    page itself from offering it, and from telling a resident "Ready" when it
    has not heard from the robot.
    """

    def setUp(self) -> None:
        self.markup = (WEB_ROOT / "resident.html").read_text()
        self.script = (WEB_ROOT / "resident.js").read_text()
        self.styles = (WEB_ROOT / "resident.css").read_text()

    def test_there_is_no_reset_anywhere_on_the_page(self) -> None:
        # The one place the word may appear is the sentence sending the
        # resident to someone who can.
        script = self.script.replace("Please ask your carer to reset the robot.", "")
        for name, text in (("html", self.markup), ("js", script)):
            with self.subTest(file=name):
                self.assertNotIn("reset", text.lower())

    def test_the_page_asks_only_for_what_the_server_allows(self) -> None:
        called = set(re.findall(r'"(/api/[a-z_/]*)"', self.script))

        self.assertEqual(
            called,
            {"/api/locations", "/api/status", "/api/goals",
             "/api/goals/cancel", "/api/emergency_stop"},
        )

    def test_the_page_does_not_say_ready_before_a_status_read(self) -> None:
        status = self.markup.split('id="status"')[1].split("</")[0]

        self.assertNotIn("Ready", status)
        self.assertNotIn("Ready", self.markup)

    def test_the_status_is_announced(self) -> None:
        before, after = self.markup.split('id="status"')
        tag = before.rsplit("<", 1)[1] + after.split(">")[0]

        self.assertIn('aria-live="polite"', tag)

    def test_the_stop_button_is_always_shown(self) -> None:
        stop = self.markup.split('id="stop-button"')[1].split(">")[0]
        before = self.markup.split('id="stop-button"')[0].rsplit("<", 1)[1]

        self.assertNotIn("hidden", stop + before)
        self.assertNotIn("stopButton.hidden", self.script)

    def test_cancel_starts_hidden(self) -> None:
        cancel = self.markup.split('id="cancel-button"')[1].split(">")[0]

        self.assertIn("hidden", cancel)

    def test_a_silent_robot_is_noticed(self) -> None:
        unreachable = self.script.split("function showUnreachable")[1].split("\n}\n")[0]

        self.assertIn("say(SAY.unreachable, true)", unreachable)
        self.assertIn('unreachable: "Can\'t reach the robot. Please ask your carer."', self.script)
        self.assertIn("setInterval(checkStatusFreshness", self.script)

    def test_every_request_gives_up_eventually(self) -> None:
        request = self.script.split("async function request")[1].split("\n}\n")[0]

        self.assertIn("new AbortController()", request)
        self.assertIn("signal", request)

    def test_urgent_messages_have_their_own_alert_regions(self) -> None:
        for element in ("alert", "stop-alert"):
            with self.subTest(element=element):
                before, after = self.markup.split(f'id="{element}"')
                tag = before.rsplit("<", 1)[1] + after.split(">")[0]
                self.assertIn('role="alert"', tag)

    def test_stop_comes_before_the_rooms_for_keyboard_and_screen_reader(self) -> None:
        self.assertLess(self.markup.index('id="stop-button"'), self.markup.index('id="rooms"'))
        self.assertIn('<section class="stop-panel" aria-label="Stop the robot">', self.markup)

    def test_the_question_is_a_labelled_group(self) -> None:
        confirm = self.markup.split('id="confirm"')[1].split(">")[0]

        self.assertIn('role="group"', confirm)
        self.assertIn('aria-labelledby="confirm-question"', confirm)

    def test_focus_has_somewhere_to_land_when_the_question_closes(self) -> None:
        question = self.markup.split('id="question"')[1].split(">")[0]

        self.assertIn('tabindex="-1"', question)

    def test_the_focus_ring_is_two_tone(self) -> None:
        # Amber alone on white is about 1.6:1; a dark ring inside it shows on
        # any background, the amber halo outside it shows on the dark ones.
        focus = self.styles.split("button:focus-visible")[1].split("}")[0]

        self.assertIn("outline: 4px solid var(--focus-dark)", focus)
        self.assertIn("var(--focus)", focus.split("box-shadow")[1])

    def test_a_short_screen_keeps_the_stop_note_for_screen_readers(self) -> None:
        short = self.styles.split("@media (max-height: 520px)")[1]
        note = short.split(".stop-note")[1].split("}")[0]

        self.assertNotIn("display: none", note)
        self.assertIn("clip", note)

    def test_text_scales_with_the_browser_setting(self) -> None:
        self.assertIn("font-size: 150%", self.styles.split(":root")[1].split("}")[0])

    def test_a_missing_latch_field_is_treated_as_unknown(self) -> None:
        self.assertIn('typeof safety.emergency_stop !== "boolean"', self.script)

    def test_the_stopped_message_sends_the_resident_to_a_carer(self) -> None:
        self.assertIn("Please ask your carer to reset the robot.", self.script)

    def test_the_page_sends_json_wherever_the_server_requires_it(self) -> None:
        for path in ("/api/goals", "/api/goals/cancel"):
            with self.subTest(path=path):
                call = self.script.split(f'request("{path}"')[1].split("})")[0]
                self.assertIn('"Content-Type": "application/json"', call)

    def test_no_coordinates_are_shown(self) -> None:
        self.assertNotRegex(self.script, r"\.(x|y|yaw)\b")

    def test_touch_targets_are_at_least_64_pixels(self) -> None:
        # In rem, so they grow with the text; 1rem is 24px at the 150% root.
        sizes = [float(value) for value in re.findall(r"min-height: ([\d.]+)rem", self.styles)]

        self.assertTrue(sizes)
        self.assertGreaterEqual(min(sizes) * 24, 64)
        self.assertNotRegex(self.styles, r"min-height: \d+px")

    def test_text_meets_the_enhanced_contrast_minimum(self) -> None:
        # WCAG AAA, 7:1: this page is read by people whose eyesight is the
        # reason they have a robot fetching things.
        colours = dict(re.findall(r"--([a-z-]+): (#[0-9a-f]{6});", self.styles))
        pairs = [
            ("ink", "paper"), ("on-action", "action"), ("on-stop", "stop"),
            ("focus-dark", "paper"),
        ]

        for foreground, background in pairs:
            with self.subTest(pair=(foreground, background)):
                ratio = contrast(colours[foreground], colours[background])
                self.assertGreaterEqual(ratio, 7.0)
        # The two halves of the focus ring have to be told apart (3:1).
        self.assertGreaterEqual(contrast(colours["focus"], colours["focus-dark"]), 3.0)


if __name__ == "__main__":
    unittest.main()
