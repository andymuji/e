import unittest

from robot_base.protocol import (
    BootReport,
    EncoderReport,
    Refusal,
    count_delta,
    format_motor_line,
    parse_pico_line,
    seal,
    ticks_diff,
    unseal,
)


class MotorLineTests(unittest.TestCase):
    def test_a_command_line_carries_its_checksum(self) -> None:
        line = format_motor_line((1, -2, 3, -1000))

        self.assertEqual(unseal(line), "M 1 -2 3 -1000")

    def test_the_pi_refuses_to_send_what_the_pico_would_refuse(self) -> None:
        for bad in [(1, 2, 3), (0, 0, 0, 1001), (0.5, 0, 0, 0), (True, 0, 0, 0)]:
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                format_motor_line(bad)


class PicoLineTests(unittest.TestCase):
    def test_an_encoder_report_is_read(self) -> None:
        report = parse_pico_line(seal("E 1234 1 5 -6 7 -8"))

        self.assertEqual(report, EncoderReport(1234, True, (5, -6, 7, -8)))

    def test_boot_and_refusal_lines_are_read(self) -> None:
        self.assertEqual(parse_pico_line(seal("B watchdog")), BootReport("watchdog"))
        self.assertEqual(parse_pico_line(seal("X range")), Refusal("range"))

    def test_anything_else_is_not_a_protocol_line(self) -> None:
        for line in [
            "Traceback (most recent call last):",
            seal("E 1 2 3"),
            seal("E 1 2 3 4 5 6"),
            seal("E x 1 1 1 1 1"),
            seal("Q hello"),
            seal("E 1 1 1 1 1 1")[:-1] + "0",
        ]:
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_pico_line(line)


class WrapTests(unittest.TestCase):
    def test_encoder_deltas_survive_the_32_bit_wrap(self) -> None:
        self.assertEqual(count_delta(-(2**31), 2**31 - 1), 1)
        self.assertEqual(count_delta(2**31 - 1, -(2**31)), -1)
        self.assertEqual(count_delta(10, 3), 7)

    def test_pico_time_survives_its_tick_wrap(self) -> None:
        self.assertEqual(ticks_diff(20, (1 << 30) - 30), 50)


if __name__ == "__main__":
    unittest.main()
