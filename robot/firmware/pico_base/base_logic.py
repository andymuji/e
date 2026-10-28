"""Everything the Pico base controller decides, with no hardware in it.

main.py owns the pins; this module owns the rules. It imports nothing, so it
runs unchanged under MicroPython on the Pico and under CPython in the tests,
which is the only way the rules below get checked before a wheel turns.

The rule everything here serves: a wheel turns only while a fresh, valid
command asks it to. Anything else - no command yet, a command older than
COMMAND_TIMEOUT_MS, a line that is garbled, out of range or too long - leaves
the motor drivers disabled.

Line protocol, over the Pico's USB serial, one ASCII line each, newline ended.
Every line carries an NMEA-style checksum: `*` and the XOR of every character
before it, as two upper-case hex digits.

    Pi -> Pico   M <fl> <fr> <rl> <rr>*HH
                 Wheel commands, front-left, front-right, rear-left,
                 rear-right, each an integer from -1000 to 1000: thousandths
                 of full motor voltage, sign is direction. All four zero
                 means motors off.

    Pico -> Pi   E <ms> <enabled> <c_fl> <c_fr> <c_rl> <c_rr>*HH
                 Every REPORT_PERIOD_MS: the Pico's millisecond tick, whether
                 the motor drivers are enabled (0/1), and each wheel's total
                 encoder count since boot, wrapped to a signed 32-bit integer.
                 B <cause>*HH
                 Once at boot: why the Pico last reset (power, watchdog, ...).
                 X <reason>*HH
                 A command line was refused, and the motors were turned off.

Wheel order is the same everywhere, on both sides: FL, FR, RL, RR.

MicroPython note: no dataclasses, enums or type annotations here, because
the Pico's interpreter does not have them all.
"""

WHEEL_COUNT = 4
FULL_SCALE = 1000
DUTY_MAX = 65535

# How long a command keeps the wheels turning. The Pi sends one every 50 ms,
# so this is five missed lines. It must stay at or below 0.25 s and at or
# below robot_safety's command_timeout; robot_base's tests check both.
COMMAND_TIMEOUT_MS = 250
REPORT_PERIOD_MS = 50

# Longest line accepted. A valid command is at most 30 characters.
LINE_MAX = 64

# MicroPython's ticks_ms() wraps at 2**30 on the RP2040.
TICKS_PERIOD = 1 << 30


def ticks_diff(new, old):
    """new - old in ms, correct across the tick counter's wrap."""
    half = TICKS_PERIOD // 2
    return ((new - old + half) % TICKS_PERIOD) - half


def wrap_int32(value):
    """An encoder total as the signed 32-bit number the report carries."""
    return ((value + (1 << 31)) % (1 << 32)) - (1 << 31)


def checksum(payload):
    total = 0
    for character in payload:
        total ^= ord(character)
    return total


def seal(payload):
    """A line body with its checksum appended, ready to send."""
    return f"{payload}*{checksum(payload):02X}"


def unseal(line):
    """The body of a received line, or ValueError if the checksum is wrong."""
    line = line.strip()
    if len(line) < 4 or line[-3] != "*":
        raise ValueError("checksum")
    payload, digits = line[:-3], line[-2:]
    for digit in digits:
        if digit not in "0123456789ABCDEF":
            raise ValueError("checksum")
    if int(digits, 16) != checksum(payload):
        raise ValueError("checksum")
    return payload


def _small_int(token):
    """A plain decimal integer, optionally negative, at most four digits.

    Stricter than int(): no '+', no spaces, no underscores, no other scripts'
    digits. Anything unusual is a garbled line, not a number.
    """
    digits = token[1:] if token.startswith("-") else token
    if not 1 <= len(digits) <= 4:
        raise ValueError("syntax")
    for digit in digits:
        if digit not in "0123456789":
            raise ValueError("syntax")
    return int(token)


def parse_motor_line(line):
    """The four wheel commands in a line, or ValueError naming what is wrong."""
    fields = unseal(line).split(" ")
    if not fields or fields[0] != "M":
        raise ValueError("syntax")
    if len(fields) != 1 + WHEEL_COUNT:
        raise ValueError("fields")
    values = tuple(_small_int(field) for field in fields[1:])
    for value in values:
        if not -FULL_SCALE <= value <= FULL_SCALE:
            raise ValueError("range")
    return values


def format_report(ms, enabled, totals):
    counts = " ".join(str(wrap_int32(total)) for total in totals)
    return seal(f"E {ms} {1 if enabled else 0} {counts}")


def format_boot(cause):
    return seal(f"B {cause}")


def format_error(reason):
    return seal(f"X {reason}")


def duty_pair(command):
    """(forward, reverse) PWM duty for one BTS7960, from -1000..1000.

    The BTS7960 has one PWM input per direction. Only one is ever non-zero.
    Both zero with the driver enabled shorts the motor through the low side,
    which brakes that wheel; with the driver disabled it coasts.
    """
    duty = abs(command) * DUTY_MAX // FULL_SCALE
    if command > 0:
        return duty, 0
    if command < 0:
        return 0, duty
    return 0, 0


def quadrature_step(a, b):
    """+1 or -1 for one edge on encoder channel A, read just after the edge.

    Which of the two is "forward" depends on how the motor is mounted and
    wired; main.py's pin table has a per-wheel flag to flip it.
    """
    return 1 if a != b else -1


class CommandWatchdog:
    """Fresh only while the last valid command is younger than the timeout."""

    def __init__(self, timeout_ms=COMMAND_TIMEOUT_MS):
        if not 0 < timeout_ms <= COMMAND_TIMEOUT_MS:
            raise ValueError(f"timeout_ms must be in (0, {COMMAND_TIMEOUT_MS}]")
        self.timeout_ms = timeout_ms
        self._last = None

    def feed(self, now_ms):
        self._last = now_ms

    def starve(self):
        """Forget the last command, so only a new one can turn a wheel."""
        self._last = None

    def fresh(self, now_ms):
        if self._last is None:
            return False
        # A negative age means the clock went backwards: not fresh.
        return 0 <= ticks_diff(now_ms, self._last) <= self.timeout_ms


class LineAssembler:
    """Bytes from the serial port in, complete lines out.

    A line that is too long, or holds anything but printable ASCII, comes out
    as None so the caller can refuse it: dropping it silently would leave the
    previous command running.
    """

    def __init__(self, limit=LINE_MAX):
        self.limit = limit
        self._buffer = bytearray()
        self._bad = False

    def feed(self, data):
        lines = []
        for byte in data:
            if byte == 10:  # "\n"
                if self._bad:
                    lines.append(None)
                else:
                    # bytes() first: MicroPython's bytearray has no decode().
                    lines.append(bytes(self._buffer).decode())
                self._buffer = bytearray()
                self._bad = False
            elif byte == 13:  # "\r", from a terminal typing at the Pico
                continue
            elif self._bad:
                continue
            elif byte < 32 or byte > 126 or len(self._buffer) >= self.limit:
                self._bad = True
                self._buffer = bytearray()
            else:
                self._buffer.append(byte)
        return lines


class BaseLogic:
    """Command lines in, what the motor drivers should do out."""

    STOPPED = (0,) * WHEEL_COUNT

    def __init__(self, timeout_ms=COMMAND_TIMEOUT_MS):
        self.watchdog = CommandWatchdog(timeout_ms)
        self.commands = self.STOPPED

    def stop(self):
        self.commands = self.STOPPED
        self.watchdog.starve()

    def on_line(self, line, now_ms):
        """Take one received line. Returns None, or why it was refused.

        A refused line stops the motors: the command before it is not kept,
        because the Pi meant to send something else and nobody knows what.
        """
        if line is None:
            self.stop()
            return "overlong"
        try:
            commands = parse_motor_line(line)
        except ValueError as error:
            self.stop()
            return str(error) or "syntax"
        self.commands = commands
        self.watchdog.feed(now_ms)
        return None

    def outputs(self, now_ms):
        """(enabled, commands) to apply now.

        Disabled unless a fresh command asks for motion. All four zero is a
        request to stop, and a stopped base has its drivers off rather than
        held in braking, so a stuck output stage cannot turn a wheel.
        """
        if not self.watchdog.fresh(now_ms):
            self.stop()
            return False, self.STOPPED
        if self.commands == self.STOPPED:
            return False, self.STOPPED
        return True, self.commands


class EncoderTotals:
    """Running totals built from the counts the interrupt handlers collect.

    The handlers only ever add +-1 to a small count that main.py empties every
    report, so they never grow a big integer (which would allocate memory,
    and a hard interrupt must not). The totals, which can grow, live here.
    """

    def __init__(self, reversed_flags):
        if len(reversed_flags) != WHEEL_COUNT:
            raise ValueError("one reversed flag per wheel")
        self.signs = tuple(-1 if flag else 1 for flag in reversed_flags)
        self.totals = [0] * WHEEL_COUNT

    def absorb(self, deltas):
        for index in range(WHEEL_COUNT):
            self.totals[index] = wrap_int32(
                self.totals[index] + self.signs[index] * deltas[index]
            )
        return tuple(self.totals)
