"""Spin one wheel, or all four: pick them and a direction, press Enter to stop.

From the repository root, on whichever computer the Pico is plugged into:

    PYTHONPATH=robot/ros2_ws/src/robot_base python3 -m robot_base.wheel_test
    ... -m robot_base.wheel_test --power 40 --port /dev/ttyACM0

BENCH TEST ONLY, wheels in the air. This talks straight to the Pico and does
NOT go through robot_safety: the owner's deliberate exception, recorded in
AGENTS.md. Nothing here checks for obstacles or that the wheels are raised.
Do not start it alongside base_node; the two would fight over the port.

The Pico stops the motors after 0.25 s without a command, so a background
thread keeps sending the current one. It also keeps reading: a Pico whose
reports go unread blocks, and its watchdog then resets it.
"""

import argparse
import glob
import threading

from robot_base.protocol import (
    FULL_SCALE,
    WHEEL_COUNT,
    BootReport,
    Refusal,
    format_motor_line,
    parse_pico_line,
)

WHEELS = ("front left", "front right", "rear left", "rear right")
CHOICES = (*WHEELS, "all four wheels")
ALL = len(WHEELS)  # Index of "all four wheels" in CHOICES.
DIRECTIONS = {"f": 1, "b": -1}
STOPPED = (0,) * WHEEL_COUNT
SEND_PERIOD = 0.05  # Same 20 Hz as base_node: five chances before the timeout.


def wheel_command(wheel: int, direction: int, power: int) -> tuple[int, ...]:
    """The four wheel commands that spin `wheel` (or ALL) at `power` percent."""
    speed = direction * power * FULL_SCALE // 100
    return tuple(
        speed if wheel in (index, ALL) else 0 for index in range(WHEEL_COUNT)
    )


def pico_messages(data: bytes) -> list[str]:
    """Anything worth telling the user in complete lines from the Pico."""
    messages = []
    for line in data.decode(errors="replace").splitlines():
        try:
            report = parse_pico_line(line)
        except ValueError:
            continue  # A line cut off by the read, or start-up noise.
        if isinstance(report, Refusal):
            messages.append(f"Pico refused a command: {report.reason}")
        elif isinstance(report, BootReport):
            messages.append(f"Pico restarted ({report.cause}); motors off")
    return messages


class Link:
    """Keeps the current command flowing to the Pico from a background thread."""

    def __init__(self, port) -> None:
        self.port = port
        self.commands = STOPPED
        self.done = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self.done.set()
        self._thread.join()
        try:
            self._send(STOPPED)
        except OSError:
            pass  # Already gone; the Pico's own timeout stops the motors.
        self.port.close()

    def _send(self, commands: tuple[int, ...]) -> None:
        self.port.write((format_motor_line(commands) + "\n").encode())

    def _run(self) -> None:
        try:
            while not self.done.is_set():
                self._send(self.commands)
                for message in pico_messages(self.port.read(4096)):
                    print(f"\n{message}")
                self.done.wait(SEND_PERIOD)
        except OSError as error:  # pyserial's SerialException is an OSError.
            print(f"\nLost the Pico: {error}. Press Enter to quit.")
            self.done.set()


def find_port() -> str:
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise SystemExit("No Pico found. Is its USB cable plugged in?")
    return ports[0]


def ask(prompt: str, answers: list[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in answers:
            return answer
        print(f"Please type one of: {', '.join(answers)}")


def run(link: Link, power: int) -> None:
    while not link.done.is_set():
        print()
        for number, name in enumerate(CHOICES, start=1):
            print(f"  {number}  {name}")
        wheel = ask("Which wheel? (1-5, q to quit) ", ["1", "2", "3", "4", "5", "q"])
        if wheel == "q":
            return
        index = int(wheel) - 1
        direction = ask("Which way? (f forward, b backward) ", list(DIRECTIONS))
        link.commands = wheel_command(index, DIRECTIONS[direction], power)
        try:
            input(f"Spinning {CHOICES[index]} at {power}%. Press Enter to stop. ")
        finally:
            link.commands = STOPPED


def main() -> None:
    parser = argparse.ArgumentParser(description="Spin one wheel, or all four.")
    parser.add_argument("--port", help="the Pico's serial port (default: find it)")
    parser.add_argument(
        "--power",
        type=int,
        default=20,
        choices=range(1, 101),
        metavar="1-100",
        help="percent of full motor power (default: 20)",
    )
    args = parser.parse_args()

    import serial  # Here, so the tests need no pyserial.

    port = args.port or find_port()
    link = Link(serial.Serial(port, 115200, timeout=0, write_timeout=0.1))
    link.start()
    print(f"Connected on {port}. Ctrl-C stops at any time.")
    try:
        run(link, args.power)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        link.close()


if __name__ == "__main__":
    main()
