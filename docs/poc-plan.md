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
| Raspberry Pi 4 (already owned) | The robot's computer. Runs all the ROS software, including the map builder. |
| LD14P lidar kit | Spins and measures the distance to every wall and object around it. This is what the map is made from. Plugs into the Pi by USB. |
| Mecanum chassis, 4 motors with wheel sensors | The body and wheels. The wheel sensors count how far each wheel turned. |
| 4 × BTS7960 motor drivers | One per motor. They turn small control signals into the heavy current the motors need. |
| Raspberry Pi Pico H | A small chip that does the split-second work: sends the control signals to the four drivers and counts the wheel sensor pulses. Talks to the Pi over USB. |
| 12 V 10 Ah LiFePO4 battery + charger | Powers the motors. |
| USB power bank (already owned, must supply 5 V 3 A) | Powers the Pi, kept separate from the motor battery. |
| Wire, 30 A fuse holder, terminals | Connect the battery to the drivers. The fuse sits right at the battery. |
| **Later:** emergency stop button + 40 A relay | Cuts power to the motors when pressed, whatever the software is doing. |
| A Mac laptop | Where a person drives the robot from and watches the map form. It doesn't need ROS installed. |

## How it fits together

```
Motor power:
  Battery + ──[30 A fuse]──[e-stop relay, later]──> 4 × BTS7960 ──> 4 motors
  Battery − ─────────────────────────────────────> 4 × BTS7960

Control:
  Mac     ~~Wi-Fi~~>  Pi 4 ──USB──> Pico ──control wires──> 4 × BTS7960
                        │             ^
                        │             └── wheel sensor wires (through a voltage
                        │                  adapter if the sensors run on 5 V)
                        └──USB──> lidar
  Power bank ──USB-C──> Pi 4
```

The Pico's ground must be wired to the drivers' ground, or the control
signals have nothing to be measured against.

### Where each program runs, and why

- **On the Pi: all the ROS software.** That's the lidar program, the
  **safety gate** (`robot_safety`), the motor driver program, the map builder
  (SLAM Toolbox), the driving keys (`teleop`), and `foxglove_bridge`, which
  lets the Mac see what the robot sees. ROS 2 doesn't run reliably on macOS,
  so none of it goes on the Mac. The safety gate would belong on the robot
  anyway: if Wi-Fi drops, the commands stop arriving, and the gate stops the
  robot. That only works if the gate isn't on the far side of the lost link.
- **On the Mac: two windows, no ROS.** The Terminal app, logged into the Pi
  (`ssh`), where the driving keys are pressed. The keys are read on the Pi,
  so a dropped connection means no more commands, which means a stop. And the
  Foxglove app, which shows the lidar scan and the map as it forms.
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
- [ ] Install the Foxglove app on the Mac.
- [ ] Write the Pico program: motor signals, wheel counting, and the
      0.2-second stop if the Pi goes quiet.
- [ ] Write the motor driver program for the Pi: takes `/cmd_vel` from the
      safety gate, passes it to the Pico, and reports how far the wheels have
      turned, so the map builder knows how the robot moved.
- [ ] Add a launch file for the real robot. The existing ones assume the
      simulator's clock, and the driving-keys one opens a window, which a
      Pi with no screen can't do.

### Stage 1 — The Pi sees the room

Needs: Pi, lidar, power bank, Mac.

- [ ] Install Ubuntu 24.04 Server (64-bit) and ROS 2 Jazzy on the Pi, then
      build this repository on it.
- [ ] Put the Pi and the Mac on the same Wi-Fi, check the Mac can log in
      with `ssh`, and give the Pi its own `ROS_DOMAIN_ID`.
- [ ] Find out how much memory the Pi has: run `free -h` on it.
- [ ] Install LDRobot's ROS 2 lidar program, and confirm it supports the LD14P.
- [ ] **Check:** Foxglove on the Mac shows the outline of the room, live, as
      you walk around carrying the Pi and lidar.

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
- [ ] Record the run. Start the map builder on the Pi, and watch it in Foxglove. Drive slowly around
      the room until the map on screen covers all of it.
- [ ] Save the map, commit it to `robot_bringup/maps/`, and commit the
      recording report to `docs/runs/`.
- [ ] **The proof of concept is done.**

---

## Open questions

These change what gets built or bought. Answer them before Stage 0 finishes.

1. ~~What does the laptop run?~~ **A Mac** (answered 2026-09-23). That's why
   all the ROS software runs on the Pi.
2. **How much memory does the Pi 4 have (1, 2, 4 or 8 GB)?** Check the box,
   or run `free -h` once it's set up (Stage 1). Now that the map builder runs
   on the Pi, this matters more: 4 GB or more is comfortable, 2 GB should
   manage one room, and 1 GB is probably too little.
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
