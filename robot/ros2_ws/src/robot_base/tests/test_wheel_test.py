import time
import unittest

from robot_base.protocol import seal, unseal
from robot_base.wheel_test import (
    STOPPED,
    Link,
    pico_messages,
    wheel_command,
)


class FakePort:
    def __init__(self) -> None:
        self.written: list[str] = []
        self.reads = 0
        self.closed = False

    def write(self, data: bytes) -> None:
        self.written.append(data.decode())

    def read(self, _size: int) -> bytes:
        self.reads += 1
        return b""

    def close(self) -> None:
        self.closed = True


class WheelCommandTests(unittest.TestCase):
    def test_only_the_chosen_wheel_turns(self) -> None:
        self.assertEqual(wheel_command(0, 1, 20), (200, 0, 0, 0))
        self.assertEqual(wheel_command(3, -1, 100), (0, 0, 0, -1000))


class PicoMessageTests(unittest.TestCase):
    def test_refusals_and_restarts_are_reported_and_the_rest_is_not(self) -> None:
        data = "\n".join(
            [seal("E 1 0 0 0 0 0"), seal("X range"), seal("B watchdog"), "junk"]
        ).encode()

        messages = pico_messages(data)

        self.assertEqual(len(messages), 2)
        self.assertIn("range", messages[0])
        self.assertIn("watchdog", messages[1])


class LinkTests(unittest.TestCase):
    def test_it_keeps_sending_and_reading_then_ends_on_a_stop(self) -> None:
        port = FakePort()
        link = Link(port)
        link.commands = (200, 0, 0, 0)
        link.start()
        deadline = time.monotonic() + 2.0
        while len(port.written) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        link.close()

        self.assertGreaterEqual(len(port.written), 3)
        self.assertGreaterEqual(port.reads, 3)
        self.assertEqual(unseal(port.written[0]), "M 200 0 0 0")
        self.assertEqual(
            unseal(port.written[-1]), "M " + " ".join(map(str, STOPPED))
        )
        self.assertTrue(port.closed)


if __name__ == "__main__":
    unittest.main()
