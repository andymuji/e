"""The Pi's half of the serial line protocol spoken with the Pico.

The Pico's half is robot/firmware/pico_base/base_logic.py, which documents
the format. The two are separate files because one runs on a microcontroller
and one in a ROS package, and test_firmware_contract.py feeds each side's
output to the other's parser so they cannot drift apart unnoticed.
"""

from collections.abc import Sequence
from dataclasses import dataclass

WHEEL_COUNT = 4
FULL_SCALE = 1000

# The Pico's ticks_ms() wraps at 2**30.
TICKS_PERIOD = 1 << 30


@dataclass(frozen=True)
class EncoderReport:
    """The Pico's periodic report: its clock, driver state, encoder totals."""

    ms: int
    enabled: bool
    totals: tuple[int, ...]


@dataclass(frozen=True)
class BootReport:
    """The Pico has just started. `cause` is why it last reset."""

    cause: str


@dataclass(frozen=True)
class Refusal:
    """The Pico refused a command line and turned the motors off."""

    reason: str


def checksum(payload: str) -> int:
    total = 0
    for character in payload:
        total ^= ord(character)
    return total


def seal(payload: str) -> str:
    return f"{payload}*{checksum(payload):02X}"


def unseal(line: str) -> str:
    line = line.strip()
    if len(line) < 4 or line[-3] != "*":
        raise ValueError(f"no checksum: {line!r}")
    payload, digits = line[:-3], line[-2:]
    if any(digit not in "0123456789ABCDEF" for digit in digits):
        raise ValueError(f"bad checksum: {line!r}")
    if int(digits, 16) != checksum(payload):
        raise ValueError(f"checksum mismatch: {line!r}")
    return payload


def format_motor_line(commands: Sequence[int]) -> str:
    """A command line for the Pico, without its newline.

    Refuses rather than clips: an out-of-range value here is a bug on the Pi,
    and the Pico would refuse it anyway.
    """
    if len(commands) != WHEEL_COUNT:
        raise ValueError(f"expected {WHEEL_COUNT} wheel commands, got {len(commands)}")
    for command in commands:
        if not isinstance(command, int) or isinstance(command, bool):
            raise ValueError(f"wheel command {command!r} is not an integer")
        if not -FULL_SCALE <= command <= FULL_SCALE:
            raise ValueError(f"wheel command {command} is outside +-{FULL_SCALE}")
    return seal("M " + " ".join(str(command) for command in commands))


def parse_pico_line(line: str) -> EncoderReport | BootReport | Refusal:
    """One line from the Pico, or ValueError if it is not a protocol line.

    A crashing MicroPython program prints a traceback, which arrives here as
    lines that are not protocol lines; the caller logs them.
    """
    fields = unseal(line).split(" ")
    kind = fields[0]
    if kind == "E" and len(fields) == 3 + WHEEL_COUNT:
        try:
            ms, enabled, *totals = (int(field) for field in fields[1:])
        except ValueError as error:
            raise ValueError(f"unreadable report: {line!r}") from error
        if enabled not in (0, 1):
            raise ValueError(f"unreadable report: {line!r}")
        return EncoderReport(ms, bool(enabled), tuple(totals))
    if kind == "B" and len(fields) == 2:
        return BootReport(fields[1])
    if kind == "X" and len(fields) == 2:
        return Refusal(fields[1])
    raise ValueError(f"not a protocol line: {line!r}")


def count_delta(new: int, old: int) -> int:
    """Encoder counts between two reports, across the 32-bit wrap."""
    return ((new - old + (1 << 31)) % (1 << 32)) - (1 << 31)


def ticks_diff(new: int, old: int) -> int:
    """Milliseconds between two of the Pico's ticks, across their wrap."""
    half = TICKS_PERIOD // 2
    return ((new - old + half) % TICKS_PERIOD) - half
