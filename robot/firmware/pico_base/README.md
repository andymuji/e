# Pico base controller

The program on the Raspberry Pi Pico H that turns the four mecanum wheels.
The Pi tells it how fast each wheel should turn, about 20 times a second, and
it reports back how far each wheel has turned. It is the last link in the
chain every motion command follows:

    keyboard -> /cmd_vel_requested -> robot_safety (the gate) -> /cmd_vel
             -> robot_base (on the Pi) -> USB cable -> this Pico -> BTS7960 boards -> motors

## What it will and will not do

The motors can only turn while the Pi keeps sending fresh, valid commands.
The motor drivers are switched **off**:

- when the Pico powers up or resets, for any reason;
- when no valid command has arrived for 0.25 s (the Pi sends one every 0.05 s);
- when a command is garbled, out of range, or too long;
- if the program crashes or is interrupted;
- if the program freezes: a hardware watchdog then resets the Pico within
  0.3 s, and it comes back with the motors off.

**This is not an emergency stop.** It is software, and software stopping
software. Do not put the wheels on the floor until the physical emergency
stop is wired so that pressing it cuts the 12 V supply to all four BTS7960
boards, independently of the Pico and the Pi - and then only with the tether,
the marked exclusion zone and the second person holding the stop that
`AGENTS.md` and `docs/safety-test-procedure.md` require. Until then, run it
with the chassis raised so no wheel touches anything.

## Files

| File | What it is |
|---|---|
| `main.py` | Runs at power-up. The **pin table** is at the top - the only place pin numbers appear. Touches the hardware and nothing else. |
| `base_logic.py` | Every rule: the message format, the 0.25 s timeout, speed to motor-power conversion, encoder counting. Tested on a PC. |
| `tests/` | Those tests. `.claude/skills/run-checks/check.sh` runs them. |

## Wiring

The pin table at the top of `main.py` is the reference; this is a summary.

| Pico pin | Goes to |
|---|---|
| GP0, GP1 | front-left BTS7960 RPWM, LPWM |
| GP2, GP3 | front-right BTS7960 RPWM, LPWM |
| GP4, GP5 | rear-left BTS7960 RPWM, LPWM |
| GP6, GP7 | rear-right BTS7960 RPWM, LPWM |
| GP10, GP11 | front-left encoder A, B |
| GP12, GP13 | front-right encoder A, B |
| GP14, GP15 | rear-left encoder A, B |
| GP16, GP17 | rear-right encoder A, B |
| GP22 | R_EN **and** L_EN of all four boards, plus a 10 kOhm resistor to GND |
| GP21 | service-mode jumper (to GND only when updating files) |
| 3V3 (OUT), pin 36 | VCC of all four boards |
| GND | GND of all four boards, the encoders, and the battery negative |

Three things that will damage parts if missed:

1. **The Pico's pins take 3.3 V at most.** If the encoders are powered from
   5 V, their A and B outputs are 5 V signals and need a level shifter (or a
   resistor divider) before they reach the Pico. Powered from 3.3 V, they
   can go straight in. Check which before connecting.
2. **Each BTS7960 board's VCC goes to the Pico's 3.3 V, not to 5 V.** VCC only
   powers the board's input chip; at 3.3 V its signals match the Pico's.
3. **All grounds are joined**: Pico, the four boards, the encoders, and the
   battery negative. The 12 V battery goes to the boards' B+/B- through the
   inline fuse (and, once wired, the emergency stop), never near the Pico.

## Putting it on the Pico

1. Install MicroPython for the Pico: hold BOOTSEL, plug the Pico in, and copy
   the Raspberry Pi Pico `.uf2` from micropython.org onto the drive that
   appears.
2. Copy `base_logic.py` and `main.py` onto the Pico, with Thonny or with
   `mpremote cp base_logic.py main.py :`.
3. Unplug and replug. The on-board LED stays dark: motors off, waiting.

To update the files later, put a jumper from GP21 to GND and replug. The
program then keeps the motors off, does not start the watchdog, and leaves
the Pico free for Thonny or `mpremote`. Remove the jumper and replug to drive
again. (Without the jumper, the watchdog resets the Pico as soon as anything
interrupts the program - which is its job, and also why the jumper exists.)

## First test, wheels in the air

On the Pi, with the chassis raised so no wheel touches anything, start the
base (see `robot_bringup/launch/base_mapping.launch.py`), then drive gently
from the keyboard. For each wheel in turn:

1. Press forward. If that wheel turns backwards, set its `motor_rev` to
   `True` in the pin table.
2. Watch `/odom` in Foxglove, or `ros2 topic echo /odom`. Driving forward
   should make `x` grow. If one wheel counts the wrong way, set its
   `encoder_rev` to `True`.

## Numbers still to measure

These live in `robot/ros2_ws/src/robot_bringup/config/base.yaml` and are
placeholders until measured:

- the wheel radius;
- the distances from the centre to the front axle and to each wheel's middle;
- the encoder counts per wheel turn. This program counts both edges of
  channel A, so it is **2 x (encoder pulses per motor turn) x (gear ratio)**;
- the wheel speed at full power, raised off the floor, in radians per second.

## Honest limits

- The Pico counts encoders in software. Very fast encoders (more than about
  5,000 edges a second per wheel) could be missed; at 0.5 m/s these motors
  should be well under that. A PIO counter would lift the limit.
- If the Pi holds the USB port open but stops reading it, the Pico's writes
  can block for up to half a second; the watchdog then resets it, with the
  motors off. That is the right outcome but it looks like a reboot.
- There is no battery-voltage reading yet. GP26 (an analog input) is kept
  free for one: a resistor divider from the 12 V battery down to under 3.3 V.
  Until that exists, the safety gate's low-battery stop cannot be switched on
  honestly, and it stays off.
