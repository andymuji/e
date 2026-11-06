from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
import re
import threading
import unittest

from robot_base.protocol import unseal
from robot_base.wheel_test import STOPPED
from robot_base.wheel_test_site import (
    HOLD_TIMEOUT,
    MAX_POWER,
    MOVES,
    PAGE,
    PAGE_HEADER,
    HeldLink,
    make_handler,
    move_command,
)


class FakePort:
    def __init__(self) -> None:
        self.written: list[str] = []

    def write(self, data: bytes) -> None:
        self.written.append(data.decode())

    def read(self, _size: int) -> bytes:
        return b""

    def close(self) -> None:
        pass


class MoveTests(unittest.TestCase):
    def test_each_move_turns_the_wheels_the_right_way(self) -> None:
        # Front left, front right, rear left, rear right, written out by hand
        # so a sign slip in the moves or the kinematics fails here.
        expected = {
            "forward": (200, 200, 200, 200),
            "backward": (-200, -200, -200, -200),
            "left": (-200, 200, 200, -200),
            "right": (200, -200, -200, 200),
            "forward-left": (0, 200, 200, 0),
            "forward-right": (200, 0, 0, 200),
            "backward-left": (-200, 0, 0, -200),
            "backward-right": (0, -200, -200, 0),
            "turn-left": (-200, 200, -200, 200),
            "turn-right": (200, -200, 200, -200),
        }
        self.assertEqual(set(expected), set(MOVES))
        for move, commands in expected.items():
            self.assertEqual(move_command(move, 20), commands, move)

    def test_the_page_offers_only_real_moves_and_all_of_them(self) -> None:
        self.assertEqual(set(re.findall(r'data-move="([^"]+)"', PAGE)), set(MOVES))


class HeldLinkTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = 100.0
        self.link = HeldLink(FakePort(), clock=lambda: self.now)

    def test_a_move_runs_out_unless_it_is_renewed(self) -> None:
        self.link.hold((200, 0, 0, 0))
        self.assertEqual(self.link.current(), (200, 0, 0, 0))

        self.now += HOLD_TIMEOUT - 0.01
        self.link.hold((200, 0, 0, 0))
        self.now += HOLD_TIMEOUT - 0.01
        self.assertEqual(self.link.current(), (200, 0, 0, 0))

        self.now += 0.01
        self.assertEqual(self.link.current(), STOPPED)

    def test_stop_is_immediate(self) -> None:
        self.link.hold((200, 0, 0, 0))
        self.link.stop()
        self.assertEqual(self.link.current(), STOPPED)

    def test_nothing_starts_before_the_first_move(self) -> None:
        self.assertEqual(self.link.current(), STOPPED)

    def test_a_lost_pico_reads_as_stopped(self) -> None:
        self.link.hold((200, 0, 0, 0))
        self.link.done.set()
        self.assertEqual(self.link.current(), STOPPED)


class ServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.link = HeldLink(FakePort())
        self.server = ThreadingHTTPServer(
            ("127.0.0.1", 0), make_handler(self.link, practice=False)
        )
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def request(self, method, path, body=None, headers=None):
        connection = HTTPConnection("127.0.0.1", self.server.server_address[1])
        self.addCleanup(connection.close)
        sent = {PAGE_HEADER: "1", "Host": "localhost:8090"}
        sent.update(headers or {})
        connection.request(method, path, body, {k: v for k, v in sent.items() if v})
        reply = connection.getresponse()
        return reply.status, reply.read()

    def drive(self, body=None, **headers):
        body = json.dumps(body or {"move": "left", "power": 20})
        return self.request("POST", "/drive", body, headers)[0]

    def test_a_held_move_reaches_the_wheels_and_stop_ends_it(self) -> None:
        self.assertEqual(self.drive(), 200)
        self.assertEqual(self.link.current(), (-200, 200, 200, -200))

        status, body = self.request("POST", "/stop", "{}")

        self.assertEqual(status, 200)
        self.assertEqual(self.link.current(), STOPPED)
        self.assertEqual(json.loads(body)["commands"], list(STOPPED))

    def test_another_site_cannot_start_the_wheels(self) -> None:
        self.assertEqual(self.drive(**{PAGE_HEADER: None}), 403)
        self.assertEqual(self.link.held[0], STOPPED)

    def test_only_this_computers_names_are_answered(self) -> None:
        for host in ("evil.example", "evil.example:8090", "192.168.1.5:8090", "["):
            self.assertEqual(self.drive(Host=host), 421, host)
            self.assertEqual(self.request("GET", "/", None, {"Host": host})[0], 421)
        self.assertEqual(self.link.held[0], STOPPED)

    def test_a_bad_request_stops_a_running_move(self) -> None:
        bad = [
            {"move": "moonwalk", "power": 20},
            {"move": ["left"], "power": 20},
            {"move": "left", "power": MAX_POWER + 1},
            {"move": "left", "power": 0},
            {"move": "left", "power": 20.5},
            {"move": "left", "power": True},
            {"move": "left"},
            [1, 2],
        ]
        for body in bad:
            self.assertEqual(self.drive(), 200)
            self.assertEqual(self.drive(body), 400, body)
            self.assertEqual(self.link.current(), STOPPED, body)
        self.assertEqual(self.drive(), 200)
        self.assertEqual(self.request("POST", "/drive", "not json")[0], 400)
        self.assertEqual(self.link.current(), STOPPED)

    def test_the_page_is_served_with_its_power_limit_filled_in(self) -> None:
        status, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(f'max="{MAX_POWER}"', body.decode())


class SendingTests(unittest.TestCase):
    def test_the_line_to_the_pico_goes_quiet_when_a_hold_runs_out(self) -> None:
        now = [0.0]
        port = FakePort()
        link = HeldLink(port, clock=lambda: now[0])
        link.hold((0, 0, 0, 200))
        link._send(link.current())
        now[0] = HOLD_TIMEOUT
        link._send(link.current())

        self.assertEqual(unseal(port.written[0]), "M 0 0 0 200")
        self.assertEqual(unseal(port.written[1]), "M 0 0 0 0")


if __name__ == "__main__":
    unittest.main()
