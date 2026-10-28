"""Pico base controller: four BTS7960 motor drivers and four wheel encoders.

Runs at power-up on a Raspberry Pi Pico H under MicroPython. Takes wheel
commands from the Pi over USB serial and reports encoder counts back; the
line protocol and every rule about when a wheel may turn are in
base_logic.py, which is tested on a PC. This file only touches pins.

FAIL CLOSED. The motor drivers are enabled only while a fresh, valid command
asks for motion. They are disabled:
  - at power-up and after ANY reset, including a hardware-watchdog reset,
    because the enable pin is the first thing this file drives, and it is
    driven low;
  - when no valid command has arrived for base_logic.COMMAND_TIMEOUT_MS;
  - when a line is garbled, out of range or too long;
  - on any exception, including Ctrl-C;
  - if this loop stops running: the hardware watchdog then resets the Pico
    within WATCHDOG_MS, which lands back at the first point above.

This is the SOFTWARE side of stopping. It is not an emergency stop. The
physical emergency stop must cut the 12 V motor supply itself, upstream of
the BTS7960 boards, and must not depend on this Pico or on the Pi.

WIRING NOTES
  - Pico GPIO is 3.3 V ONLY and not 5 V tolerant. Encoders powered from 5 V
    put 5 V on their A/B outputs: those need a level shifter (or a divider)
    before they reach a Pico pin, or they will damage it. Encoders that run
    from 3.3 V can be wired straight in.
  - Each BTS7960 board's VCC pin goes to the Pico's 3V3 (OUT) pin, pin 36,
    NOT to 5 V: VCC powers the board's input buffer, and at 3.3 V its logic
    levels match the Pico's. All grounds are common: Pico GND, every board's
    GND, and the battery negative.
  - R_EN and L_EN on every board are tied together and to ENABLE_PIN, with
    one 10 kOhm resistor from that line to GND. The Pico's own pull-downs
    hold the line low while it boots; the resistor keeps it low if the wire
    to the Pico comes loose or the Pico is unplugged.
  - Nothing else may drive the BTS7960 inputs. See AGENTS.md: no other
    node, board or remote connects to the motor controller.

SERVICE MODE. Once the hardware watchdog is started it cannot be stopped,
and it resets the Pico whenever this program is interrupted - which is what
it is for, but it also means Thonny or mpremote cannot copy new files over a
running controller. Put a jumper from SERVICE_PIN to GND and power-cycle:
the controller then keeps the motors off, never starts the watchdog, and
drops to the prompt. Remove the jumper and power-cycle to drive again.
"""

from machine import Pin

# ---------------------------------------------------------------------------
# PIN TABLE. The only place a pin number appears. Change pins here and only
# here. GPnn numbers, not the physical pin numbers printed on the board edge.
# ---------------------------------------------------------------------------

# R_EN + L_EN of all four BTS7960 boards, with a 10 kOhm pull-down to GND.
# High = drivers allowed to drive. Low = every motor output off.
ENABLE_PIN = 22

# Jumper to GND at power-up for service mode (see above).
SERVICE_PIN = 21

# On-board LED: lit while the motor drivers are enabled.
LED_PIN = 25

# One row per wheel, in protocol order FL, FR, RL, RR.
#   RPWM, LPWM      the BTS7960's two PWM inputs (forward, reverse). Each
#                   pair shares one RP2040 PWM slice (GP0/1 is slice 0, ...).
#   ENC_A, ENC_B    the encoder's two channels. 3.3 V signals only.
#   motor_rev       True if a positive command turns this wheel backwards.
#   encoder_rev     True if this wheel counts down when turning forwards.
# The *_rev flags are PLACEHOLDERS until the wheels are tested on the bench,
# raised off the floor: see README.md.
WHEELS = (
    # name,          RPWM, LPWM, ENC_A, ENC_B, motor_rev, encoder_rev
    ("front_left",   0,    1,    10,    11,    False,     False),
    ("front_right",  2,    3,    12,    13,    False,     False),
    ("rear_left",    4,    5,    14,    15,    False,     False),
    ("rear_right",   6,    7,    16,    17,    False,     False),
)

# GP26, GP27 and GP28 are left free: they are the ADC inputs, and GP26 is
# where a battery-voltage divider would go (see README.md).

PWM_FREQUENCY_HZ = 20_000  # Above hearing; the BTS7960 is rated to 25 kHz.

# Hardware watchdog. If the loop below stops for this long, the Pico resets
# and comes back with the motors off.
WATCHDOG_MS = 300

# ---------------------------------------------------------------------------
# Motors off before anything else happens, including importing the rest.
# ---------------------------------------------------------------------------
_enable = Pin(ENABLE_PIN, Pin.OUT, value=0)

import array  # noqa: E402
import select  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402

import base_logic  # noqa: E402
import machine  # noqa: E402
from machine import PWM  # noqa: E402

_led = Pin(LED_PIN, Pin.OUT, value=0)
_pwms = []
for _row in WHEELS:
    for _pin in (_row[1], _row[2]):
        _pwm = PWM(Pin(_pin, Pin.OUT, value=0))
        _pwm.freq(PWM_FREQUENCY_HZ)
        _pwm.duty_u16(0)
        _pwms.append(_pwm)


def motors_off():
    """Disable first, then zero the duties: the order that never glitches."""
    _enable.value(0)
    for pwm in _pwms:
        pwm.duty_u16(0)
    _led.value(0)


def apply(enabled, commands):
    if not enabled:
        motors_off()
        return
    for index, row in enumerate(WHEELS):
        command = -commands[index] if row[5] else commands[index]
        forward, reverse = base_logic.duty_pair(command)
        _pwms[2 * index].duty_u16(forward)
        _pwms[2 * index + 1].duty_u16(reverse)
    # Duties first, enable last, so no wheel starts on a stale duty.
    _enable.value(1)
    _led.value(1)


# Encoder counts since the last report, added to by the interrupt handlers.
# Small integers only: see base_logic.EncoderTotals.
_counts = array.array("i", [0] * base_logic.WHEEL_COUNT)
_encoder_pins = []


def _make_handler(index, pin_b):
    counts = _counts
    step = base_logic.quadrature_step

    def handler(pin_a):
        counts[index] += step(pin_a.value(), pin_b.value())

    return handler


def start_encoders():
    for index, row in enumerate(WHEELS):
        pin_a = Pin(row[3], Pin.IN, Pin.PULL_UP)
        pin_b = Pin(row[4], Pin.IN, Pin.PULL_UP)
        pin_a.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
            handler=_make_handler(index, pin_b),
            hard=True,
        )
        _encoder_pins.append((pin_a, pin_b))


def take_counts():
    """The counts since the last call, read and zeroed with interrupts off."""
    state = machine.disable_irq()
    try:
        deltas = tuple(_counts)
        for index in range(base_logic.WHEEL_COUNT):
            _counts[index] = 0
    finally:
        machine.enable_irq(state)
    return deltas


def send(line):
    sys.stdout.write(line + "\n")


def reset_cause():
    cause = machine.reset_cause()
    if cause == machine.WDT_RESET:
        return "watchdog"
    if cause == machine.PWRON_RESET:
        return "power"
    return f"other-{cause}"


def run():
    watchdog = machine.WDT(timeout=WATCHDOG_MS)
    logic = base_logic.BaseLogic()
    lines = base_logic.LineAssembler()
    totals = base_logic.EncoderTotals([row[6] for row in WHEELS])
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    start_encoders()

    send(base_logic.format_boot(reset_cause()))
    next_report = time.ticks_ms()

    while True:
        watchdog.feed()

        # Read what has arrived, but a bounded amount per pass, so a flood
        # of input cannot starve the watchdog or the timeout check below.
        for _ in range(128):
            if not poll.poll(0):
                break
            now = time.ticks_ms()
            for line in lines.feed(sys.stdin.buffer.read(1)):
                refused = logic.on_line(line, now)
                if refused is not None:
                    apply(False, logic.STOPPED)
                    send(base_logic.format_error(refused))

        enabled, commands = logic.outputs(time.ticks_ms())
        apply(enabled, commands)

        now = time.ticks_ms()
        if base_logic.ticks_diff(now, next_report) >= 0:
            next_report = time.ticks_add(next_report, base_logic.REPORT_PERIOD_MS)
            if base_logic.ticks_diff(now, next_report) > 0:
                next_report = now  # Fell behind; do not burst to catch up.
            send(base_logic.format_report(now, enabled, totals.absorb(take_counts())))


def main():
    service = Pin(SERVICE_PIN, Pin.IN, Pin.PULL_UP)
    if service.value() == 0:
        motors_off()
        send(base_logic.format_boot("service"))
        return  # To the prompt, motors off, no watchdog.
    try:
        run()
    finally:
        # Every way out of run() - an exception, Ctrl-C - ends here with the
        # motors off. The watchdog then resets the Pico, which starts again
        # with the motors off and waits for a fresh command.
        motors_off()


main()
