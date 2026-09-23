# Proof of concept: drive the robot around one room and map it

**Goal:** a person drives the real robot around one room by remote control,
and the robot builds a map of that room from its lidar.

**Done means:** a saved map of the real room is committed to
`robot/ros2_ws/src/robot_bringup/maps/`, together with the recording report
from the mapping run in `docs/runs/`. Nothing else counts.

**Not in this proof of concept:** driving to a goal by itself (Nav2), voice,
the operator console, and sideways driving. They all wait.

Written 2026-09-23. Parts total: $291.99 before shipping, emergency stop
bought later.

---

## The parts and what each one does

| Part | Job |
|---|---|
| Raspberry Pi 4 (already owned) | The robot's computer. Runs the lidar, the safety gate, and the motor driver program. |
| LD14P lidar kit | Spins and measures the distance to every wall and object around it. This is what the map is made from. Plugs into the Pi by USB. |
| Mecanum chassis, 4 motors with wheel sensors | The body and wheels. The wheel sensors count how far each wheel turned. |
| 4 × BTS7960 motor drivers | One per motor. They turn small control signals into the heavy current the motors need. |
| Raspberry Pi Pico H | A small chip that does the split-second work: sends the control signals to the four drivers and counts the wheel sensor pulses. Talks to the Pi over USB. |
| 12 V 10 Ah LiFePO4 battery + charger | Powers the motors. |
| USB power bank (already owned, must supply 5 V 3 A) | Powers the Pi, kept separate from the motor battery. |
| Wire, 30 A fuse holder, terminals | Connect the battery to the drivers. The fuse sits right at the battery. |
| **Later:** emergency stop button + 40 A relay | Cuts power to the motors when pressed, whatever the software is doing. |
| A laptop | Where a person drives the robot from and watches the map form. |

## How it fits together

```
Motor power:
  Battery + ──[30 A fuse]──[e-stop relay, later]──> 4 × BTS7960 ──> 4 motors
  Battery − ─────────────────────────────────────> 4 × BTS7960

Control:
  Laptop  ~~Wi-Fi~~>  Pi 4 ──USB──> Pico ──control wires──> 4 × BTS7960
                        │             ^
                        │             └── wheel sensor wires (through a voltage
                        │                  adapter if the sensors run on 5 V)
                        └──USB──> lidar
  Power bank ──USB-C──> Pi 4
```

The Pico's ground must be wired to the drivers' ground, or the control
signals have nothing to be measured against.

### Where each program runs, and why

- **On the Pi:** the lidar program, the **safety gate** (`robot_safety`), and
  the motor driver program. The safety gate has to be on the robot, not the
  laptop: if Wi-Fi drops, the commands stop arriving, and the gate stops the
  robot. That only works if the gate isn't on the far side of the lost link.
- **On the laptop:** the driving keys (`teleop`), the map builder
  (SLAM Toolbox), and the map display (RViz). The map builder is heavy, and
  this keeps the Pi free.
- **On the Pico:** motor signals and wheel counting. If it hears nothing from
  the Pi for 0.2 seconds, it stops the motors on its own. Losing the command
  stream is a stop condition, never a reason to keep going.

The motion path is the same as in simulation, and the project's rules apply
unchanged: driving keys → `/cmd_vel_requested` → safety gate → `/cmd_vel` →
motor driver program → Pico. Nothing else sends `/cmd_vel`.

### One simplification: drive it like a tank

Mecanum wheels can drive sideways. For this proof of concept they won't: the
two left wheels always turn together, and the two right wheels together, like
a tank. That matches what the existing software, safety gate and robot
description already assume, and a room can be mapped without sideways
driving. Each wheel still has its own driver, so sideways driving can be added
later without new parts.

---

## The stages

Each stage ends in something you can see working. Don't start a stage until
the one before it works.

### Stage 0 — Before the parts arrive (software)

- [ ] Answer the [open questions](#open-questions) below.
- [ ] Set up the laptop with ROS 2 Jazzy (see [getting-started.md](getting-started.md)).
- [ ] Write the Pico program: motor signals, wheel counting, and the
      0.2-second stop if the Pi goes quiet.
- [ ] Write the motor driver program for the Pi: takes `/cmd_vel` from the
      safety gate, passes it to the Pico, and reports how far the wheels have
      turned, so the map builder knows how the robot moved.
- [ ] Add a launch file for the real robot. The existing ones assume the
      simulator's clock.

### Stage 1 — The Pi sees the room

Needs: Pi, lidar, power bank, laptop.

- [ ] Install Ubuntu 24.04 Server (64-bit) and ROS 2 Jazzy on the Pi, then
      build this repository on it.
- [ ] Put the Pi and the laptop on the same Wi-Fi and the same
      `ROS_DOMAIN_ID`.
- [ ] Install LDRobot's ROS 2 lidar program, and confirm it supports the LD14P.
- [ ] **Check:** the laptop shows the outline of the room, live, as you walk
      around carrying the Pi and lidar.

### Stage 2 — The wheels turn, in the air

Needs: everything except the emergency stop. **The robot sits on a box with
all four wheels off the ground for this whole stage.**

- [ ] Check what voltage the wheel sensors use. If it's 5 V, add a voltage
      adapter (level shifter) before connecting them to the Pico.
- [ ] Wire battery → fuse → drivers → motors. Wire the Pico to the drivers
      and the wheel sensors.
- [ ] **Check:** each wheel spins in the right direction when you press the
      driving keys.
- [ ] **Check:** stopping the driver program or unplugging the Pico's USB
      stops every wheel within a fraction of a second.
- [ ] **Check:** turning a wheel by hand one full turn changes its count by
      the amount the motor's spec says it should.

### Stage 3 — Measure the real robot

The robot description and the safety distances are placeholders right now.
They must be replaced before the robot touches the floor.

- [ ] Measure the body (length, width, height), the wheel size, the gap
      between the left and right wheels, and where the lidar sits. Put them
      into the robot description (`robot_description/urdf`).
- [ ] Measure the top speed, the braking distance, and how long the wheels
      take to react to a command. Put them into
      `robot_bringup/config/base_dynamics.yaml` and set `measured: true`.
- [ ] Run the checks. The stop and caution distances recalculate themselves
      from these numbers. Don't type them in by hand.

### Stage 4 — Emergency stop, then the safety test

- [ ] Buy and fit the emergency stop button and relay. The button switches
      the relay, and the relay cuts motor power.
- [ ] Carry out every step of [safety-test-procedure.md](safety-test-procedure.md),
      starting with the wheels in the air, while recording the run
      (`robot_telemetry`).
- [ ] **Nothing drives on the floor until this stage passes.**

### Stage 5 — Map the room

- [ ] Clear the room of people who aren't taking part. Two people: one
      drives, the other holds the emergency stop. Tether the robot as the
      safety procedure says.
- [ ] Record the run. Start the map builder on the laptop. Drive slowly around
      the room until the map on screen covers all of it.
- [ ] Save the map, commit it to `robot_bringup/maps/`, and commit the
      recording report to `docs/runs/`.
- [ ] **The proof of concept is done.**

---

## Open questions

These change what gets built or bought. Answer them before Stage 0 finishes.

1. **What does the laptop run: Windows, Mac or Linux?** ROS 2 works best on
   Ubuntu Linux. On Windows or Mac the laptop needs extra setup, or the Pi
   does more of the work.
2. **How much memory does the Pi 4 have (2, 4 or 8 GB)?** It's printed on the
   board. 2 GB will be tight.
3. **What voltage do the wheel sensors use?** Check the chassis listing or ask
   the seller. If it's 5 V, buy a level shifter (about $3–5).
4. **What current does the battery's protection circuit (BMS) allow?** It
   needs to be 20 A or more, or the battery may cut out when all four motors
   start at once.
5. **Does the power bank really supply 5 V 3 A?** If not, the Pi will slow
   itself down or reboot. The fix is a 12 V → 5 V converter from the motor
   battery (about $20).
6. **Which room gets mapped?** Somewhere with space to clear, a door that
   closes, and no stairs.
