"""A web page for trying the base's moves: hold a button, the wheels turn.

From the repository root, on whichever computer the Pico is plugged into:

    PYTHONPATH=robot/ros2_ws/src/robot_base python3 -m robot_base.wheel_test_site
    ... -m robot_base.wheel_test_site --practice     # no Pico, nothing moves

then open http://localhost:8090. If the Pico is on the Pi, reach the page from
another computer through ssh rather than opening it to the network:

    ssh -L 8090:127.0.0.1:8090 <user>@<pi>

BENCH TEST ONLY, wheels in the air. Like wheel_test.py this talks straight to
the Pico and does NOT go through robot_safety: the owner's deliberate
exception, recorded in AGENTS.md. Do not start it alongside base_node.

There is no login, so it listens on this computer only, answers only to
"localhost", and takes a command only from its own page. Do not widen any of
that: anyone who can reach it can spin the motors with no safety gate.

A move lasts HOLD_TIMEOUT unless the page renews it, so a closed tab or a
dropped connection stops the wheels. The Pico's own timeout cannot do this
job, because the Link goes on sending the current command for as long as it
runs.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import time
from urllib.parse import urlsplit

from robot_base.kinematics import MecanumGeometry, wheel_commands
from robot_base.wheel_test import STOPPED, Link, find_port, open_pico

PAGE_PORT = 8090
HOLD_TIMEOUT = 0.3  # The page renews a move every 0.1 s: three chances.
# Four motors at once draw far more than one, and the battery gives only 10 A
# steadily, 15 A for 10 seconds.
# This is a guess, not a measurement; raise it only once the current is known.
MAX_POWER = 50
LOCAL_NAMES = ("localhost", "127.0.0.1")
PAGE_HEADER = "X-Wheel-Test"  # Another site's page cannot add this without our OK.
MAX_BODY = 1024

# Each move as a direction of the body: forward, left, anticlockwise.
MOVES = {
    "forward": (1, 0, 0),
    "backward": (-1, 0, 0),
    "left": (0, 1, 0),
    "right": (0, -1, 0),
    "forward-left": (1, 1, 0),
    "forward-right": (1, -1, 0),
    "backward-left": (-1, 1, 0),
    "backward-right": (-1, -1, 0),
    "turn-left": (0, 0, 1),
    "turn-right": (0, 0, -1),
}
# Only the pattern matters here, so any geometry will do: not the measured one.
_UNIT_BASE = MecanumGeometry(1.0, 0.5, 0.5)


def move_command(move: str, power: int) -> tuple[int, ...]:
    """The four wheel commands for `move`, the busiest wheel at `power` percent."""
    speeds = _UNIT_BASE.inverse(*MOVES[move])
    return wheel_commands(speeds, max(map(abs, speeds)) * 100 / power)


class HeldLink(Link):
    """A Link whose command runs out unless it is renewed."""

    def __init__(self, port, clock=time.monotonic) -> None:
        super().__init__(port)
        self.clock = clock
        self.message = ""
        # One tuple, so a send can never pair a command with another's deadline.
        self.held = (STOPPED, 0.0)

    def hold(self, commands: tuple[int, ...]) -> None:
        self.held = (commands, self.clock() + HOLD_TIMEOUT)

    def stop(self) -> None:
        self.held = (STOPPED, 0.0)

    def current(self) -> tuple[int, ...]:
        commands, deadline = self.held
        if self.done.is_set() or self.clock() >= deadline:
            return STOPPED
        return commands

    def report(self, message: str) -> None:
        super().report(message)
        self.message = f"{time.strftime('%H:%M:%S')} {message}"


class FakePico:
    """Stands in for the serial port in --practice."""

    def write(self, _data: bytes) -> None:
        pass

    def read(self, _size: int) -> bytes:
        return b""

    def close(self) -> None:
        pass


def parse_drive(body: bytes) -> tuple[int, ...]:
    """The wheel commands a /drive body asks for, or ValueError."""
    request = json.loads(body)  # JSONDecodeError is a ValueError.
    if not isinstance(request, dict):
        raise ValueError("expected an object")
    move, power = request.get("move"), request.get("power")
    if not isinstance(move, str) or move not in MOVES:
        raise ValueError("unknown move")
    if type(power) is not int or not 1 <= power <= MAX_POWER:
        raise ValueError(f"power must be a whole number from 1 to {MAX_POWER}")
    return move_command(move, power)


def make_handler(link: HeldLink, practice: bool):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if not self._host_is_ours():
                return
            if self.path == "/":
                html = PAGE.replace("__MAX_POWER__", str(MAX_POWER))
                self._send(200, html.encode(), "text/html; charset=utf-8")
            elif self.path == "/state":
                self._send_state()
            else:
                self._send_json(404, {"error": "not found"})

        def do_POST(self) -> None:
            if not self._host_is_ours():
                return
            if PAGE_HEADER not in self.headers:
                self._send_json(403, {"error": "not from the wheel test page"})
            elif self.path == "/stop":
                link.stop()
                self._send_state()
            elif self.path == "/drive":
                self._drive()
            else:
                self._send_json(404, {"error": "not found"})

        def _drive(self) -> None:
            try:
                length = int(self.headers.get("Content-Length", ""))
                if not 0 < length <= MAX_BODY:
                    raise ValueError("bad body size")
                commands = parse_drive(self.rfile.read(length))
            except ValueError as error:
                link.stop()  # Whatever was meant, it was not "keep going".
                self._send_json(400, {"error": str(error)})
                return
            link.hold(commands)
            self._send_state()

        def _host_is_ours(self) -> bool:
            """Refuse any other name: that is how DNS rebinding arrives."""
            try:
                name = urlsplit(f"//{self.headers.get('Host', '')}").hostname
            except ValueError:
                name = None
            if name in LOCAL_NAMES:
                return True
            self._send_json(421, {"error": "unrecognised host name"})
            return False

        def _send_state(self) -> None:
            self._send_json(
                200,
                {
                    "connected": not link.done.is_set(),
                    "practice": practice,
                    "commands": link.current(),
                    "message": link.message,
                },
            )

        def _send_json(self, status: int, payload: dict) -> None:
            self._send(status, json.dumps(payload).encode(), "application/json")

        def _send(self, status: int, body: bytes, kind: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_args) -> None:
            pass  # Ten requests a second while a button is held.

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Try the base's moves from a web page.")
    parser.add_argument("--port", help="the Pico's serial port (default: find it)")
    parser.add_argument(
        "--practice", action="store_true", help="no Pico: try the page, nothing moves"
    )
    args = parser.parse_args()

    port = FakePico() if args.practice else open_pico(args.port or find_port())
    link = HeldLink(port)
    server = ThreadingHTTPServer(
        ("127.0.0.1", PAGE_PORT), make_handler(link, args.practice)
    )
    link.start()
    print(f"Open http://localhost:{PAGE_PORT} - Ctrl-C here stops everything.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.server_close()
        link.close()


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wheel test</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0 auto; max-width: 760px; padding: 16px; color: #1c2024;
         background: #f4f5f6; font: 16px/1.4 system-ui, sans-serif; }
  h1 { margin: 0; font-size: 1.5rem; }
  .warn { margin: 4px 0 12px; font-weight: 600; color: #a11a1a; }
  #status { padding: 10px 12px; border-radius: 8px; background: #dfe3e6; }
  #status.ok { background: #d5efdc; }
  #status.bad { background: #f8d7d7; }
  #message { min-height: 1.4em; margin: 6px 0; font-size: .9rem; }
  .arm { display: block; margin: 12px 0; padding: 10px 12px; font-weight: 600;
         background: #fff3cd; border-radius: 8px; cursor: pointer; }
  .panel { display: flex; flex-wrap: wrap; gap: 24px; align-items: flex-start; }
  .panel > div:last-child { width: 272px; }
  .pad { display: grid; grid-template-columns: repeat(3, 84px);
         grid-auto-rows: 84px; gap: 6px; }
  .turns { display: flex; gap: 6px; margin-top: 6px; }
  .turns button { flex: 1; height: 60px; }
  button { font: inherit; font-weight: 600; border: 1px solid #8a9299;
           border-radius: 10px; background: #fff; color: inherit; cursor: pointer;
           touch-action: none; user-select: none; -webkit-user-select: none; }
  button span { display: block; font-size: 1.5rem; line-height: 1.1; }
  button.small { font-size: .75rem; }
  button:disabled { opacity: .45; cursor: not-allowed; }
  button.held { background: #1c5fbf; border-color: #1c5fbf; color: #fff; }
  #stop { background: #c62020; border-color: #c62020; color: #fff;
          font-size: 1.1rem; }
  .robot { display: grid; grid-template-columns: 96px 96px; gap: 48px 40px;
           padding: 18px 14px; background: #dfe3e6; border-radius: 14px;
           text-align: center; }
  .wheel { padding: 6px 2px; background: #fff; border-radius: 8px; font-size: .8rem; }
  .wheel b { display: block; min-height: 1.3em; font-size: 1.6rem; line-height: 1.3; }
  .front { margin: 0 0 6px; font-size: .85rem; text-align: center; }
  label.power { display: block; margin-top: 16px; }
  input[type=range] { width: 100%; max-width: 300px; display: block; }
  .keys { font-size: .85rem; color: #4a5258; }
</style>
</head>
<body>
<h1>Wheel test</h1>
<p class="warn">Bench test: wheels in the air only. This skips the safety gate,
and STOP here is not an emergency stop. Keep a hand on the battery connection.</p>
<div id="status" role="status">Connecting…</div>
<div id="message"></div>
<label class="arm"><input type="checkbox" id="armed">
  The robot is raised and no wheel can touch anything</label>
<div class="panel">
  <div>
    <div class="pad">
      <button class="small" data-move="forward-left"><span>↖</span>Forward left</button>
      <button data-move="forward"><span>↑</span>Forward</button>
      <button class="small" data-move="forward-right"><span>↗</span>Forward right</button>
      <button data-move="left"><span>←</span>Left</button>
      <button id="stop">STOP</button>
      <button data-move="right"><span>→</span>Right</button>
      <button class="small" data-move="backward-left"><span>↙</span>Back left</button>
      <button data-move="backward"><span>↓</span>Back</button>
      <button class="small" data-move="backward-right"><span>↘</span>Back right</button>
    </div>
    <div class="turns">
      <button data-move="turn-left">⟲ Turn left</button>
      <button data-move="turn-right">⟳ Turn right</button>
    </div>
    <label class="power">Power: <output id="powerText">20</output>%
      <input type="range" id="power" min="5" max="__MAX_POWER__" step="5" value="20">
    </label>
  </div>
  <div>
    <p class="front">↑ Front of the robot · how each wheel should turn</p>
    <div class="robot" id="robot">
      <div class="wheel"><b></b>Front left</div>
      <div class="wheel"><b></b>Front right</div>
      <div class="wheel"><b></b>Rear left</div>
      <div class="wheel"><b></b>Rear right</div>
    </div>
    <p class="keys">Keyboard: arrow keys move, Q and E turn, Space stops.
    ▲ means the wheel turns as it would to drive forward.</p>
  </div>
</div>
<script>
const armed = document.getElementById("armed");
const power = document.getElementById("power");
const statusBox = document.getElementById("status");
const moveButtons = [...document.querySelectorAll("[data-move]")];
const wheels = [...document.querySelectorAll("#robot b")];
const KEYS = { ArrowUp: "forward", ArrowDown: "backward", ArrowLeft: "left",
               ArrowRight: "right", q: "turn-left", e: "turn-right" };
let held = null, timer = null, driving = null, usable = false;

function setStatus(text, kind) {
  statusBox.textContent = text;
  statusBox.className = kind;
}

function render(state) {
  usable = state.connected;
  if (!state.connected) {
    setStatus("Lost the Pico. Wheels are stopped. Check the cable, then restart "
              + "the program.", "bad");
  } else if (state.practice) {
    setStatus("Practice mode: no Pico, nothing will move.", "");
  } else {
    setStatus("Connected to the Pico.", "ok");
  }
  document.getElementById("message").textContent =
    state.connected ? state.message : "";
  state.commands.forEach((command, index) => {
    wheels[index].textContent = command > 0 ? "▲" : command < 0 ? "▼" : "";
  });
  refresh();
}

function refresh() {
  moveButtons.forEach((button) => {
    button.disabled = !armed.checked || !usable;
    button.classList.toggle("held", button.dataset.move === held);
  });
  if (held && (!armed.checked || !usable)) stop();
}

function lostServer() {
  usable = false;
  setStatus("Can't reach the test program. The wheels stop by themselves.", "bad");
  wheels.forEach((wheel) => { wheel.textContent = ""; });
  refresh();
}

async function post(path, body) {
  const reply = await fetch(path, {
    method: "POST",
    headers: { "X-Wheel-Test": "1", "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  if (!reply.ok) throw new Error(reply.status);
  render(await reply.json());
}

function drive() {
  if (driving) return;  // A slow reply lets the hold run out, which is right.
  driving = post("/drive", { move: held, power: Number(power.value) })
    .catch(lostServer)
    .finally(() => { driving = null; });
}

function start(move) {
  if (held || !armed.checked || !usable) return;
  held = move;
  refresh();
  drive();
  timer = setInterval(drive, 100);
}

function stop() {
  clearInterval(timer);
  held = null;
  refresh();
  post("/stop").catch(lostServer);
  // A drive already on its way could land after that stop.
  if (driving) driving.finally(() => {
    if (!held) post("/stop").catch(lostServer);
  });
}

moveButtons.forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    button.setPointerCapture(event.pointerId);
    start(button.dataset.move);
  });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) {
    button.addEventListener(name, () => { if (held) stop(); });
  }
  button.addEventListener("contextmenu", (event) => event.preventDefault());
});
document.getElementById("stop").addEventListener("pointerdown", stop);
document.addEventListener("keydown", (event) => {
  if (event.key === " " || event.key === "Escape") {
    event.preventDefault();
    stop();
  } else if (KEYS[event.key]
             && !(event.target === power && event.key.startsWith("Arrow"))) {
    event.preventDefault();
    if (!event.repeat) start(KEYS[event.key]);
  }
});
document.addEventListener("keyup", (event) => {
  if (held && KEYS[event.key] === held) stop();
});
window.addEventListener("blur", () => { if (held) stop(); });
document.addEventListener("visibilitychange", () => { if (held) stop(); });
armed.addEventListener("change", refresh);
power.addEventListener("input", () => {
  document.getElementById("powerText").textContent = power.value;
});

// Let go of the slider with the mouse and the arrow keys drive again.
power.addEventListener("pointerup", () => power.blur());

function poll() {
  fetch("/state").then((reply) => reply.json()).then(render).catch(lostServer);
}
poll();
setInterval(poll, 500);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
