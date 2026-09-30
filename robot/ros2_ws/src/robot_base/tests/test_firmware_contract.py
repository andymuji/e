"""The Pi and the Pico speak the same protocol, and agree on when to stop.

The two halves live in different places - a ROS package and a
microcontroller's files - so each side's output is fed to the other side's
parser here. A change to one that the other does not follow fails this test
instead of failing on the robot, where a refused line means a stopped base
and nobody sure why.
"""

import importlib.util
from pathlib import Path
import unittest

from robot_base.protocol import (
    BootReport,
    EncoderReport,
    Refusal,
    format_motor_line,
    parse_pico_line,
)
import yaml

REPO = Path(__file__).resolve().parents[5]
FIRMWARE = REPO / "robot" / "firmware" / "pico_base" / "base_logic.py"
CONFIG = REPO / "robot" / "ros2_ws" / "src" / "robot_bringup" / "config"


def load_firmware():
    spec = importlib.util.spec_from_file_location("pico_base_logic", FIRMWARE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ProtocolContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.pico = load_firmware()

    def test_the_pico_reads_what_the_pi_sends(self) -> None:
        for commands in [(0, 0, 0, 0), (1000, -1000, 1, -1), (250, 500, -750, 0)]:
            with self.subTest(commands=commands):
                self.assertEqual(
                    self.pico.parse_motor_line(format_motor_line(commands)), commands
                )

    def test_the_pi_reads_what_the_pico_sends(self) -> None:
        report = self.pico.format_report(99, True, (1, -2, 2**31, 4))

        self.assertEqual(
            parse_pico_line(report), EncoderReport(99, True, (1, -2, -(2**31), 4))
        )
        self.assertEqual(
            parse_pico_line(self.pico.format_boot("watchdog")), BootReport("watchdog")
        )
        self.assertEqual(
            parse_pico_line(self.pico.format_error("checksum")), Refusal("checksum")
        )

    def test_both_sides_agree_on_full_scale_and_wheel_count(self) -> None:
        from robot_base import protocol

        self.assertEqual(protocol.FULL_SCALE, self.pico.FULL_SCALE)
        self.assertEqual(protocol.WHEEL_COUNT, self.pico.WHEEL_COUNT)
        self.assertEqual(protocol.TICKS_PERIOD, self.pico.TICKS_PERIOD)


class StopTimingContractTests(unittest.TestCase):
    """The Pico must give up on a silent link no later than the gate would."""

    def setUp(self) -> None:
        self.pico = load_firmware()
        self.gate = yaml.safe_load((CONFIG / "safety.yaml").read_text())[
            "safety_controller"
        ]["ros__parameters"]

    def test_the_pico_stops_within_a_quarter_second(self) -> None:
        self.assertLessEqual(self.pico.COMMAND_TIMEOUT_MS, 250)

    def test_the_pico_stops_no_later_than_the_gates_command_timeout(self) -> None:
        self.assertLessEqual(
            self.pico.COMMAND_TIMEOUT_MS / 1000.0, self.gate["command_timeout"]
        )

    def test_the_pi_sends_often_enough_to_keep_the_pico_fed(self) -> None:
        # At least three lines per Pico timeout, so one late line is not a stop.
        base = yaml.safe_load((CONFIG / "base.yaml").read_text())["base_driver"][
            "ros__parameters"
        ]
        period_ms = 1000.0 / base["control_rate_hz"]

        self.assertLessEqual(3 * period_ms, self.pico.COMMAND_TIMEOUT_MS)


if __name__ == "__main__":
    unittest.main()
