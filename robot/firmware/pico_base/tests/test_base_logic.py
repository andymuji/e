"""The Pico's rules, checked on a PC before anything is flashed.

base_logic.py imports nothing, so it runs here exactly as it runs on the
Pico. main.py needs `machine` and is not imported: it only moves what these
rules decide onto pins.
"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import base_logic  # noqa: E402
from base_logic import (  # noqa: E402
    COMMAND_TIMEOUT_MS,
    BaseLogic,
    CommandWatchdog,
    EncoderTotals,
    LineAssembler,
    duty_pair,
    format_report,
    parse_motor_line,
    seal,
    ticks_diff,
    unseal,
    wrap_int32,
)

FORWARD = seal("M 500 500 500 500")


class ChecksumTests(unittest.TestCase):
    def test_a_sealed_line_opens_to_its_payload(self) -> None:
        self.assertEqual(unseal(seal("M 1 2 3 4")), "M 1 2 3 4")

    def test_a_single_changed_character_is_caught(self) -> None:
        line = seal("M 100 100 100 100")
        garbled = line.replace("100", "900", 1)

        with self.assertRaises(ValueError):
            unseal(garbled)

    def test_a_line_without_a_checksum_is_refused(self) -> None:
        for line in ["M 0 0 0 0", "M 0 0 0 0*", "M 0 0 0 0*zz", "", "*00"]:
            with self.subTest(line=line), self.assertRaises(ValueError):
                unseal(line)


class MotorLineTests(unittest.TestCase):
    def test_four_wheel_commands_are_read_in_order(self) -> None:
        self.assertEqual(parse_motor_line(seal("M 1 -2 30 -1000")), (1, -2, 30, -1000))

    def test_out_of_range_is_refused_not_clipped(self) -> None:
        # Clipping would still drive, at a speed nobody asked for.
        with self.assertRaisesRegex(ValueError, "range"):
            parse_motor_line(seal("M 1001 0 0 0"))

    def test_anything_but_plain_integers_is_refused(self) -> None:
        for body in [
            "M 1 2 3",
            "M 1 2 3 4 5",
            "M 1.0 0 0 0",
            "M +1 0 0 0",
            "M 1_0 0 0 0",
            "M 10000 0 0 0",
            "M  1 0 0 0",
            "M nan 0 0 0",
            "S 0 0 0 0",
            "M - 0 0 0",
        ]:
            with self.subTest(body=body), self.assertRaises(ValueError):
                parse_motor_line(seal(body))


class WatchdogTests(unittest.TestCase):
    def test_the_timeout_is_no_longer_than_a_quarter_second(self) -> None:
        self.assertLessEqual(COMMAND_TIMEOUT_MS, 250)
        with self.assertRaises(ValueError):
            CommandWatchdog(COMMAND_TIMEOUT_MS + 1)

    def test_nothing_is_fresh_before_the_first_command(self) -> None:
        self.assertFalse(CommandWatchdog().fresh(0))

    def test_a_command_goes_stale_after_the_timeout(self) -> None:
        watchdog = CommandWatchdog()
        watchdog.feed(1000)

        self.assertTrue(watchdog.fresh(1000 + COMMAND_TIMEOUT_MS))
        self.assertFalse(watchdog.fresh(1000 + COMMAND_TIMEOUT_MS + 1))

    def test_a_clock_that_went_backwards_is_not_fresh(self) -> None:
        watchdog = CommandWatchdog()
        watchdog.feed(1000)

        self.assertFalse(watchdog.fresh(990))

    def test_the_tick_counter_wrapping_does_not_freeze_a_command(self) -> None:
        # ticks_ms() wraps every 12.4 days. Plain subtraction would give a
        # huge negative age across the wrap - here refused, but the danger is
        # the mirror image: an age that never grows.
        last = base_logic.TICKS_PERIOD - 10
        watchdog = CommandWatchdog()
        watchdog.feed(last)

        self.assertEqual(ticks_diff(20, last), 30)
        self.assertTrue(watchdog.fresh(20))
        self.assertFalse(watchdog.fresh(COMMAND_TIMEOUT_MS))


class BaseLogicTests(unittest.TestCase):
    def setUp(self) -> None:
        self.logic = BaseLogic()

    def test_motors_are_off_at_boot(self) -> None:
        # Also what the Pico is after a hardware-watchdog reset: a fresh
        # BaseLogic, with no command it may act on.
        self.assertEqual(self.logic.outputs(0), (False, (0, 0, 0, 0)))

    def test_a_fresh_valid_command_enables_the_motors(self) -> None:
        self.assertIsNone(self.logic.on_line(FORWARD, 0))

        self.assertEqual(self.logic.outputs(10), (True, (500, 500, 500, 500)))

    def test_a_silent_link_turns_the_motors_off(self) -> None:
        self.logic.on_line(FORWARD, 0)

        self.assertFalse(self.logic.outputs(COMMAND_TIMEOUT_MS + 1)[0])

    def test_a_stale_command_is_not_replayed_when_the_clock_looks_fresh_again(
        self,
    ) -> None:
        self.logic.on_line(FORWARD, 0)
        self.logic.outputs(COMMAND_TIMEOUT_MS + 1)

        self.assertFalse(self.logic.outputs(0)[0])

    def test_a_garbled_line_stops_the_motors_rather_than_keeping_the_last_command(
        self,
    ) -> None:
        self.logic.on_line(FORWARD, 0)

        reason = self.logic.on_line(FORWARD[:-1] + "0", 10)

        self.assertEqual(reason, "checksum")
        self.assertEqual(self.logic.outputs(11), (False, (0, 0, 0, 0)))

    def test_an_out_of_range_line_stops_the_motors(self) -> None:
        self.logic.on_line(FORWARD, 0)

        self.assertEqual(self.logic.on_line(seal("M 2000 0 0 0"), 10), "range")
        self.assertFalse(self.logic.outputs(11)[0])

    def test_an_overlong_line_stops_the_motors(self) -> None:
        self.logic.on_line(FORWARD, 0)

        self.assertEqual(self.logic.on_line(None, 10), "overlong")
        self.assertFalse(self.logic.outputs(11)[0])

    def test_after_a_refusal_only_a_new_command_moves_the_wheels(self) -> None:
        self.logic.on_line(FORWARD, 0)
        self.logic.on_line("junk", 10)
        self.logic.on_line(seal("M 0 0 0 100"), 20)

        self.assertEqual(self.logic.outputs(21), (True, (0, 0, 0, 100)))

    def test_all_zero_turns_the_drivers_off_rather_than_braking(self) -> None:
        self.logic.on_line(seal("M 0 0 0 0"), 0)

        self.assertEqual(self.logic.outputs(1), (False, (0, 0, 0, 0)))


class LineAssemblerTests(unittest.TestCase):
    def test_lines_are_split_on_newlines_and_carriage_returns_ignored(self) -> None:
        lines = LineAssembler()

        self.assertEqual(lines.feed(b"M 1"), [])
        self.assertEqual(lines.feed(b" 2\r\nX\n"), ["M 1 2", "X"])

    def test_a_line_that_is_too_long_is_reported_not_dropped(self) -> None:
        lines = LineAssembler(limit=8)

        self.assertEqual(lines.feed(b"123456789\nok\n"), [None, "ok"])

    def test_a_non_ascii_line_is_reported_not_dropped(self) -> None:
        self.assertEqual(LineAssembler().feed(b"M \xff\n"), [None])

    def test_a_byte_at_a_time_gives_the_same_lines(self) -> None:
        lines = LineAssembler()
        found = []
        for byte in FORWARD.encode() + b"\n":
            found += lines.feed(bytes([byte]))

        self.assertEqual(found, [FORWARD])


class DutyTests(unittest.TestCase):
    def test_direction_picks_which_input_gets_the_duty(self) -> None:
        self.assertEqual(duty_pair(1000), (65535, 0))
        self.assertEqual(duty_pair(-1000), (0, 65535))
        self.assertEqual(duty_pair(500), (32767, 0))

    def test_zero_drives_neither_input(self) -> None:
        self.assertEqual(duty_pair(0), (0, 0))

    def test_never_both_inputs_at_once(self) -> None:
        # Both high on a BTS7960 shorts the motor through the high side.
        for command in range(-1000, 1001, 7):
            forward, reverse = duty_pair(command)
            self.assertTrue(forward == 0 or reverse == 0, command)


class EncoderTests(unittest.TestCase):
    def test_quadrature_direction_follows_the_phase_between_channels(self) -> None:
        self.assertEqual(base_logic.quadrature_step(1, 0), 1)
        self.assertEqual(base_logic.quadrature_step(0, 1), 1)
        self.assertEqual(base_logic.quadrature_step(1, 1), -1)
        self.assertEqual(base_logic.quadrature_step(0, 0), -1)

    def test_totals_accumulate_and_honour_the_reversed_flags(self) -> None:
        totals = EncoderTotals([False, True, False, False])

        totals.absorb((5, 5, -3, 0))

        self.assertEqual(totals.absorb((1, 1, 1, 1)), (6, -6, -2, 1))

    def test_totals_wrap_as_signed_32_bit(self) -> None:
        totals = EncoderTotals([False] * 4)
        totals.totals[0] = 2**31 - 1

        self.assertEqual(totals.absorb((1, 0, 0, 0))[0], -(2**31))
        self.assertEqual(wrap_int32(-(2**31) - 1), 2**31 - 1)

    def test_the_report_carries_time_state_and_totals(self) -> None:
        line = format_report(1234, True, (1, -2, 3, -4))

        self.assertEqual(unseal(line), "E 1234 1 1 -2 3 -4")


if __name__ == "__main__":
    unittest.main()
