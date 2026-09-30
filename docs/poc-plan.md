# Proof of concept: drive the robot's own base around one room and map it

**Goal:** a person drives the mecanum base that was bought on 2026-09-26
around one room with the keyboard, and the robot builds a map of that room
from its lidar. Every drive command passes through the safety gate
(`robot_safety`), exactly as it will on the real robot.

**Done means:** a saved map of the real room is committed to
`robot/ros2_ws/src/robot_bringup/maps/`, together with, in `docs/runs/`:

- the recording of the mapping run and its `robot_telemetry` report, which
  must say **`VERDICT: PASS`** (see
  [what the report should say](#what-the-recording-report-should-say));
- the filled-in [stop-test form](runs/stop-test-form.md) from Stage 5, every
  row a pass.

**Not in this proof of concept:** driving to a goal by itself (Nav2), voice,
the operator console, and sideways driving. They all wait.

Written 2026-09-23. Rewritten 2026-09-30 for the parts that were actually
bought. **Still to buy: about $78 before tax, plus two shipping charges not
yet known**, almost all of it the emergency stop. See
[still to buy](#still-to-buy).

---

## Read this first: there is no physical emergency stop yet

**None of the parts bought is an emergency stop.** The base has a battery,
four motors and four motor drivers, and nothing that cuts the motors' power
when a person presses a button.

The software stop exists and works, but the project's rules say it is
**secondary**: if the Pi freezes, or a motor driver sticks, software cannot
stop the wheels. Only a stop that physically cuts power can.

So, until the stop on the [still-to-buy list](#still-to-buy) has been bought,
wired and passed its tests in Stage 5:

- **The base never drives on the floor.** It sits on a box with all four
  wheels in the air.
- **The battery is only connected while someone is watching**, and the fuse
  is pulled out whenever you walk away.

And once the stop works, every floor run still needs all three of these
([safety-test-procedure.md](safety-test-procedure.md)):

- **a tether** — the robot is on a leash, and pulling it hard cuts the
  motors;
- **a marked exclusion zone** — tape on the floor, and nobody inside it;
- **a second person** holding the stop button, whose only job is the button.

**Nobody who isn't taking part is in the room during a floor run.** That
includes any elder or other vulnerable person.

## The parts and what each one does

| Part | Job |
|---|---|
| Raspberry Pi 4 (owned) | The robot's computer. Runs all the ROS software: the lidar program, the safety gate, the motor driver program, the map builder and the recorder. |
| LD14P lidar kit (D200) (bought) | Spins and measures the distance to every wall and object around it. The map is made from this, and the safety gate watches it. Plugs into the Pi by USB. |
| Mecanum chassis, 4 motors with wheel sensors (bought) | The body and wheels. The wheel sensors count how far each wheel turned. |
| 4 × BTS7960 motor drivers (bought) | One per motor. They turn the Pico's small control signals into the heavy current the motors need. |
| Raspberry Pi Pico H (bought) | A small chip that does the split-second work: sends the control signals to the four drivers and counts the wheel sensor pulses. Talks to the Pi over USB. |
| ExpertPower 12 V 10 Ah LiFePO4 battery + 14.6 V charger (bought) | Powers the motors, and nothing else. See [the battery](#the-battery-can-only-supply-10-a) — it's weaker than the plan asked for. |
| 12-AWG wire, 30 A fuse holder, ring terminals (bought) | Connect the battery to the drivers. The fuse sits right at the battery. |
| USB power bank (owned, must supply 5 V 3 A) | Powers the Pi, kept separate from the motor battery. The Pi powers the Pico and the lidar. |
| **To buy:** stop button, relay, tether, fuses | The physical emergency stop. Pressing the button, or pulling the tether, cuts power to all four motor drivers. |
| A Mac laptop (owned) | Where a person drives from (the Terminal app, logged into the Pi) and watches the map form (the Foxglove app). It doesn't need ROS. |

## Parts

### Already owned

Bought on 2026-09-26, from [the 09-26 list](#what-changed-on-2026-09-30-and-why):

| Part | Notes |
|---|---|
| LD14P lidar kit (D200) | Includes the lidar's USB adapter |
| Mecanum chassis + 4 motors with wheel sensors | Which variant arrived, and its wheel-sensor voltage, are [still to check](#still-to-check-and-by-whom) |
| BTS7960 motor drivers, pack of 4 | |
| ExpertPower 12 V 10 Ah LiFePO4 battery | Model EP1210. Its terminals are "F2" flat tabs, 0.25 inch wide |
| 14.6 V 2 A LiFePO4 charger | |
| Raspberry Pi Pico H | Pins already soldered on |
| 12-AWG red and black wire, 25 ft | |
| 30 A inline fuse holder (ATC) | Takes the ordinary flat car fuses. **No fuse in it yet** |
| Ring terminal / heat-shrink kit | **Ring terminals don't fit the battery's F2 tabs.** Check whether the kit also has 0.25-inch female spade connectors |

Owned before:

| Item | Notes |
|---|---|
| Raspberry Pi 4 | Memory size still unknown (see open questions) |
| USB power bank, 5 V 3 A | Powers the Pi |
| microSD card, 32 GB+ | For the Pi |
| USB cables | Includes the lidar's cable and a micro-USB cable for the Pico |
| Wire stripper, crimper, multimeter, screwdrivers | |
| Zip ties, double-sided tape, a flat scrap of plastic or wood | Mounting plate for the Pi and lidar |
| Mac laptop | Runs the Foxglove app and the Terminal |

No RC car was bought, and none will be.

### Still to buy

#### The answer

**You still have to buy the emergency stop and a few small parts: about
$77.74 before tax, plus shipping from two small shops that only shows at their
checkouts.** With Los Altos sales tax (9.75%) that's about $85, before those
two shipping charges.

- **Nothing drives on the floor until the stop has arrived, been wired, and
  passed Stage 5.** Stages 1 to 4 can all happen before then, with the wheels
  in the air.
- **Amazon won't let its prices be read automatically any more.** Six of the
  parts are the same Amazon listings the 09-28 plan checked, and their prices
  are from that check, two days old. Put them in one basket with ZIP 94022
  and check before ordering.
- **Four prices come from search results, not from the shop's own page.**
  They're marked "search result". Check them in the shop.

#### Why a relay, and not one big switch

There are two ways to build a stop that cuts the motors' power.

1. **A big switch in the battery wire.** Pressing it breaks the motor current
   directly. Simple, but a switch that can safely break 15 A of battery
   current is a large industrial part (the ones found cost $146–196), the
   tether would have to carry the full motor current, and a broken wire in it
   would do nothing.
2. **A small switch that holds a relay on (chosen).** A relay is a switch
   worked by an electromagnet. Its heavy contacts sit in the battery wire and
   are **open unless the relay is held on.** A thin wire loop runs from the
   relay, down the tether, through the stop button and back, and while that
   loop is complete it holds the relay on. Only a tiny current flows in the
   loop, so an ordinary stop button and thin tether wire will do.

The relay design is **fail-safe**: anything that breaks the loop opens the
relay and cuts the motors, and nothing that goes wrong can close it.

- pressing the button;
- pulling the tether until its plug comes apart;
- a wire breaking or falling out;
- the battery going flat;
- the relay's own coil burning out.

The only failure it doesn't cover is the relay's contacts welding shut, which
is why its contacts are rated 40 A, well above anything this battery can
deliver (15 A at most, see below), and why the stop is tested at the start of
every session.

The button **latches**: once pressed it stays down until you twist it. The
motors can't get power back until someone deliberately twists it.

#### The battery can only supply 10 A

The 09-26 plan said this battery was acceptable **only if** its protection
circuit (the BMS, the circuit inside that switches the battery off to protect
it) allowed 20 A or more. **It allows 10 A continuously, and 15 A for up to
10 seconds** ([ExpertPower's own page](https://www.expertpower.us/products/ep1210-10ah),
read 2026-09-30).

What that means:

- **It's probably enough for this proof of concept.** Driving slowly on a
  flat floor with nothing on board, four small motors draw a few amps.
- **It can cut out.** Starting hard, spinning on the spot, or a wheel jammed
  against something can make all four motors together pull more than 15 A.
  The battery then switches itself off: the wheels lose power and the robot
  rolls to a stop. The Pi and lidar keep running, because they're on the
  power bank. That isn't dangerous at walking speed, but it looks exactly
  like a fault, so **if the wheels suddenly die, suspect this first.**
- **It is not enough for the real robot** with a 9 kg payload.
- **If it keeps cutting out,** the 09-26 plan's alternative was a 12.8 V
  20 Ah battery rated 20 A continuous ($56.99 at Walmart then, not checked
  since).
- **It sets the fuse size: 15 A, not 20 A.** A fuse only helps if it blows
  before something else gives up, and the battery switches itself off above
  15 A anyway. A 15 A fuse is well within what the 12-AWG wire can carry.

#### What to buy

Prices checked 2026-09-30 unless marked otherwise.

| Part / job | Exact product | Qty | Price | Seller | What's confirmed, and what isn't |
|---|---|---:|---:|---|---|
| Stop button | [BAOMAIN e-stop in a box](https://www.amazon.com/dp/B00NTT91Y0), 1 NC + 1 NO, twist to reset | 1 | $9.99 | BAOMAIN, Amazon-shipped | **Price from 09-28**, not re-readable. Title confirmed today: "1NO1NC 660V 10A". Only the NC contact is used |
| Motor power cut | [12 V 40 A 4-pin relay](https://www.superbrightleds.com/arx-4pin-40a-x), normally open | 2 | $5.98 | Super Bright LEDs | Read today: $2.99 each, in stock, 12 V DC coil. Second one is a spare. **Shipping not shown**; free only over $300 |
| Fuses: 15 A main, 2 A for the stop loop | [ZIPCCI 80-piece ATC assortment](https://www.amazon.com/dp/B0B18NVT5Z) | 1 | $8.99 | Amazon-shipped | **Price from 09-28.** Check the listing includes 15 A and 2 A; if not, any auto parts store sells both |
| Fuse holder for the stop loop | [VANTRONIK inline ATC holder, 14 AWG, 3-pack](https://www.amazon.com/dp/B0DKJJJYX1) | 1 | $4.89 | Amazon-shipped | **Price from 09-28** |
| Tether wire (stop loop) | [HiFind 22 AWG 2-core, 50 ft, tinned copper](https://www.amazon.com/dp/B0F384B5QY) | 1 | $9.49 | Amazon-shipped | **Price from 09-28.** Real copper |
| Pull-apart plug on the robot | [CENTROPOWER 5.5 × 2.1 mm DC plug pairs, 10 pairs](https://www.amazon.com/dp/B07C7VSRBG) | 1 | $9.39 | CENTROPOWER, Amazon-shipped | **Price from 09-28** |
| Leash | [HERCULES 550 paracord, 50 ft](https://www.amazon.com/dp/B0BD2J2DQV) | 1 | $4.99 | Amazon-shipped | **Price from 09-28** |
| Joins without soldering, and splitting the power four ways | [WAGO 221-415 five-way lever connectors, 10-pack](https://www.homedepot.com/p/WAGO-221-415K006-000-5-Wire-Lever-Nuts-Conductor-Compact-Splicing-Connectors-12-24-AWG-10-Pack-221-415K006-000/334555610) | 1 | $10.47 | Home Depot | **Search result**, not read from the page. Takes wire from 24 to 12 AWG |
| Battery and relay connectors | 0.25-inch insulated female spade connectors: 10 yellow (12–10 AWG) and 4 red (22–18 AWG) | 1 set | ~$2.20 | Any hardware store; e.g. [Jameco](https://www.jameco.com/z/19017-0050-Molex-Quick-Connect-0-250-Female-12-10-AWG-Crimp-Non-Mating-End-Insulated_460761.html) | **Search result**: $0.22 each for 10+ yellow. Red not priced. **Skip if the ring terminal kit already has them** |
| Signal wires, Pico to drivers | [Female-to-female jumper wires, 40 × 20 cm](https://www.pishop.us/product/female-to-female-jumper-cable-x-40-20cm/) | 1 | $2.95 | PiShop.us | Read today: in stock. Signal only, never motor or battery current |
| Signal wires, to the breadboard | [Male-to-female jumper wires, 40 × 20 cm](https://www.pishop.us/product/male-to-female-jumper-cable-x-40-20cm/) | 1 | $2.95 | PiShop.us | Read today: in stock |
| Sharing the Pico's one 3.3 V pin | [Half-size breadboard, 400 points](https://www.pishop.us/product/half-size-400-pin-diy-breadboard-clear/) | 1 | $5.45 | PiShop.us | Read today: in stock. Four drivers and four wheel sensors all need 3.3 V and ground from the Pico, which has one 3.3 V pin |
| **Total** | | | **$77.74** | | Before tax. Plus shipping from Super Bright LEDs and PiShop.us, shown only at their checkouts |

**By seller:**

| Seller | Items | Shipping |
|---|---:|---|
| Amazon-shipped (6 items) | $47.74 | $0: over $35, free by Amazon's policy. Not confirmed at a checkout for 94022 |
| Super Bright LEDs | $5.98 | Unknown until checkout |
| PiShop.us | $11.35 | Unknown until checkout: set by weight |
| Hardware store, in person (WAGO, spade connectors) | $12.67 | None |
| **All** | **$77.74** | Plus two unknown charges |

**Alternatives found, not counted:**

- **Relay:** [Jameco's 12 V 40 A relay](https://www.jameco.com/z/171-1c-12dm-r-jameco-valuepro-relay-enclosed-style-single-pole-double-throw-spdt-12vdc-12vdc-40a_171433.html)
  (search result: $2.95, 1,177 in stock) would put the relay and the spade
  connectors in one order. Amazon also sells 5-packs of 40 A relays with
  plug-in sockets and wires, which avoid crimping but whose prices couldn't
  be read.
- **Stop button:** industrial stop stations at AutomationDirect cost
  [$146](https://www.automationdirect.com/adc/shopping/catalog/pushbuttons_-z-_switches_-z-_indicators/idem_emergency_stop_control_stations/esl-ssp-232014)
  and [$196](https://www.automationdirect.com/adc/shopping/catalog/pushbuttons_-z-_switches_-z-_indicators/emergency_stop_control_stations/e22asb106).
  Better-built, but not needed to switch a relay.

#### Only if the wheel sensors run on 5 V

The Pico's pins take **3.3 V only**; 5 V damages them. Look at the label on
a motor or the chassis listing: if the wheel sensors say they work on 3.3 V,
power them from the Pico's 3.3 V and buy nothing. If they need 5 V (or say
"3.5–5 V"), buy two of these, one board per two sensors:

| Part | Qty | Price | Seller | Notes |
|---|---:|---:|---|---|
| [BSS138 4-channel level shifter](https://www.adafruit.com/product/757) | 2 | $7.90 | Adafruit | Read today: $3.95 each, in stock. Plus Adafruit's shipping |

#### Wiring note (from the 09-26 plan, still true)

Each BTS7960 board has a small VCC pin that powers its input chips. Wire it
to the Pico's **3.3 V** pin, not 5 V. Then the drivers read the Pico's 3.3 V
signals reliably, with no level shifters in between.

Every battery-current wire is 12 AWG. Every step of the wiring, with a check
after each, is in [wiring-checklist.md](wiring-checklist.md).

## How it fits together

```
Motor power (12 V):
  Battery + ──[15 A fuse]──┬──[relay contacts: open unless the loop holds them]──> 4 × BTS7960 ──> 4 motors
                           │
  The stop loop:           └──[2 A fuse]──> pull-apart plug ──> tether ──> stop button (NC)
                                                                               │  (second person)
                           ┌── relay coil <── pull-apart plug <── tether <─────┘
  Battery − ───────────────┴──────────────────────────────────> 4 × BTS7960

Control:
  Mac ~~Wi-Fi~~> Pi 4 ──USB──> Pico H ──signal wires──> 4 × BTS7960
                   │              ^
                   │              └── wheel sensor wires (through level
                   │                  shifters only if they run on 5 V)
                   └── powered by: power bank ──USB-C──> Pi 4

Sensing:
  lidar ──USB──> Pi 4      (the map is made from this, and the gate watches it)

Viewing:
  Pi 4 ~~Wi-Fi (5 GHz)~~> Mac: Terminal app (driving keys, over ssh)
                               Foxglove app (the scan and the map, watch-only)
```

**The two halves are separate.** The battery powers only the motor drivers
(and holds the relay on). The Pi, the Pico and the lidar run from the power
bank. So cutting the motors never switches off the computer, which keeps
recording what happened.

The Pico's ground must be wired to the drivers' ground, or the control
signals have nothing to be measured against.

### Where each program runs

- **On the Pi: all the ROS software.** `base_mapping.launch.py` starts all of
  it at once: the lidar program, the **safety gate**, the motor driver program
  (`robot_base`), the map builder (Cartographer), the recorder, and
  `foxglove_bridge` for the Mac to watch through. The gate belongs on the
  robot: if Wi-Fi drops, the commands stop arriving and the gate stops the
  robot. That only works if the gate isn't on the far side of the lost link.
- **On the Mac: two windows, no ROS.** The Terminal app, logged into the Pi,
  where the driving keys are pressed. The keys are read on the Pi, so a
  dropped connection means no more commands, which means a stop. And the
  Foxglove app, showing the scan and the map as it forms.
- **On the Pico:** motor signals and wheel counting. If it hears nothing from
  the Pi for a moment, it stops the motors by itself. Losing the command
  stream is a stop, never a reason to keep going.

The motion path is the project's rule, unchanged:

```
driving keys ──> /cmd_vel_requested ──> safety gate ──> /cmd_vel ──> robot_base ──USB──> Pico ──> drivers
```

Nothing else sends `/cmd_vel`. The physical stop sits outside all of it, in
the battery wire.

### One simplification: drive it like a tank

Mecanum wheels can drive sideways. For this proof of concept they won't: the
two left wheels always turn together, and the two right wheels together, like
a tank. That matches what the safety gate and the robot description already
assume, and a room can be mapped without sideways driving.

### Two things the gate will do that look like faults

- **It stops for anything close on any side**, behind and beside as well as
  in front. The gate looks at the whole lidar circle. Near a wall, in a
  narrow gap, or with someone standing close, the robot won't move. That is
  the gate working. Keep more than the stop distance from everything, and
  have the second person stand back.
- **It stops the moment you let go of the keys.** A drive command older than
  half a second is thrown away, not repeated.

### What the recording report should say

`base_mapping.launch.py` records the run. Afterwards, `analyse_run` checks the
recording ([what the report checks](safety-test-procedure.md#what-the-report-checks))
and gives one verdict. With the gate in the path, all four of its checks
can be answered, so this run can reach **`PASS`**, unlike the old RC-car plan.

| Check | What this run will show |
|---|---|
| The recording contains something to judge | **Passed**, because the gate is running and publishing |
| Only `/safety_controller` published `/cmd_vel` | **Passed**, as long as nothing but the gate drives the wheels. `robot_base` only listens to `/cmd_vel`. Anything else here is a **fail** |
| Nothing moved while the safety gate said stop | **Passed**: the gate says stop at the start, before the first key, and every time you let go of the keys |
| The emergency stop stayed latched until an operator reset it | **Only if you use the software stop during the run.** The physical button cuts power without telling the software, so the recording can't see it |

So, during the mapping run, **once**, while driving slowly: press the
software stop, check the wheels stop and stay stopped, then send the reset.
The commands are in [getting-started.md](getting-started.md). Then:

- **`PASS`** (exit 0) is what "done" needs.
- **`INCOMPLETE`** (exit 4) almost certainly means the software stop wasn't
  used during the recording. Not a pass: record the run again.
- **`FAIL`** (exit 1) means something drove the wheels that shouldn't have.
  **Stop all hardware work until it's explained.**

After Stage 4, pass the new stop distance to `analyse_run` with
`--stop-distance`. Without it, the report measures stopping times against the
old placeholder (0.45 m). The verdict doesn't depend on it, but the stopping
times do.

### Mapping

- **Drive slowly:** about a slow walk (0.3 m/s or less), and turn slowly. The
  lidar only scans about six times a second.
- **Expect drift** along long bare walls and near glass.
- **The recording is the backup.** If the live map goes wrong, the "rebuild
  the map from a recording" command makes it again from the recorded lidar
  data. Both it and the "save the map" command are in
  [getting-started.md](getting-started.md).

---

## The stages

Each stage ends in something you can see working. Don't start a stage until
the one before it works. **The wheels stay in the air until the stop has
passed its tests in Stage 5.**

### Stage 0 — Before the parts arrive (software)

- [ ] Answer the [open questions](#open-questions) below.
- [ ] Install the Foxglove app on the Mac.
- [x] Add a launch file for mapping with the lidar alone, on the real clock.
      **Done 2026-09-30:** `car_mapping.launch.py`. Written for the RC car;
      Stage 1 still uses it, carrying the Pi and lidar by hand.
- [x] Decide which one program publishes the lidar's position on the robot.
      **Decided:** this repository's launch file, `base_link` → `lidar_link`.
      LDRobot's own launch file isn't used.
- [ ] Write the Pico program (`robot/firmware/pico_base/`): motor signals,
      wheel counting, and stopping by itself if the Pi goes quiet. **Being
      written 2026-09-30.**
- [ ] Write the motor driver program for the Pi (`robot_base`): takes
      `/cmd_vel` from the gate, passes it to the Pico, and reports how far the
      wheels have turned. **Being written 2026-09-30.**
- [ ] Add the launch file for the base, `base_mapping.launch.py`: gate,
      driver, lidar, map builder, recorder and Foxglove, on the real clock.
      **Being written 2026-09-30.**
- [ ] Add the "save the map" and "rebuild the map from a recording"
      commands, documented in [getting-started.md](getting-started.md).
      **Being written 2026-09-30.**
- [ ] Check the driving keys work in a plain ssh terminal on the Pi. The
      existing `teleop.launch.py` opens a window, which a Pi with no screen
      can't do.

### Stage 1 — The Pi sees the room

Needs: Pi, lidar, power bank, Mac. No motors, no battery.

Follow "Set up the Pi" in [getting-started.md](getting-started.md). It runs
`robot/scripts/setup-pi.sh`, which does most of this stage. Written, not yet
run on a Pi.

- [ ] Install Ubuntu 24.04 Server (64-bit) and ROS 2 Jazzy on the Pi, then
      build this repository on it.
- [ ] Put the Pi and the Mac on the same **5 GHz** Wi-Fi, check the Mac can
      log in with `ssh`, and give the Pi its own `ROS_DOMAIN_ID`.
- [ ] Find out how much memory the Pi has: run `free -h` on it.
- [ ] Build LDRobot's lidar program (`ldlidar_ros2`), which lists the LD14P.
      Nobody has confirmed it builds on Jazzy on a Pi, so this step proves it.
- [ ] **Check:** Foxglove on the Mac shows the outline of the room, live,
      as you carry the Pi and lidar around.
- [ ] **Check:** carry them slowly round the whole room and watch a map
      form. This proves the mapping works before any motor is involved.

### Stage 2 — Power and the emergency stop, wheels in the air

Needs: the stop parts from the still-to-buy list. **The base sits on a box
with all four wheels off the ground for this whole stage.** The Pico isn't
connected yet, so nothing can command the motors.

Follow [wiring-checklist.md](wiring-checklist.md), sections 1 to 4.

- [ ] Do the on-arrival checks: battery terminals, fuse sizes, relay coil
      voltage, stop button contacts.
- [ ] Wire battery → 15 A fuse → relay → drivers, and the stop loop through
      the 2 A fuse, the tether, the pull-apart plug and the button.
- [ ] **Check:** with the loop closed, the relay clicks on and the drivers
      get 12 V. Press the button: it clicks off and they get 0 V. Pull the
      plug apart: the same.
- [ ] **Check:** nothing on the Pi's side has any connection to the battery
      except the shared ground through the drivers.

### Stage 3 — Pico and drivers: every wheel turns the right way, in the air

Needs: the Pico program, `robot_base`, `base_mapping.launch.py`. **Wheels
still in the air.** The second person holds the stop button throughout.

Follow [wiring-checklist.md](wiring-checklist.md), sections 5 and 6.

- [ ] Check what voltage the wheel sensors use. If they need 5 V, fit the
      level shifters first.
- [ ] Wire the Pico to the drivers and the wheel sensors, using the pin table
      that comes with the Pico program.
- [ ] Start `base_mapping.launch.py` on the Pi and drive from the Mac's
      Terminal, through the gate.
- [ ] **Check:** forward key → all four wheels turn forward. Turn keys → left
      and right sides turn opposite ways. A wheel turning backwards gets its
      two motor wires swapped at its driver (battery disconnected first).
- [ ] **Check:** let go of the keys → every wheel stops.
- [ ] **Check:** the stop button stops every wheel while a key is held.
- [ ] **Check:** turning a wheel by hand one full turn changes its count by
      the amount the motor's spec says it should.
- [ ] **Check:** with the robot still and nothing near it, the gate says
      clear. If it says stop, the lidar is seeing part of the robot: move the
      lidar up or the part down.

### Stage 4 — Measure the base

The robot description and the safety distances are placeholders. **They must
be replaced with this base's real numbers before it touches the floor.** The
stop and caution distances are calculated from these numbers; if the numbers
are wrong, every safety margin is wrong in the dangerous direction at once.

Everything here is measured **with the wheels in the air**, except braking,
which can only be measured on the floor. Braking is the first floor run of
all, in Stage 5, with every floor-run rule in force. Until then it keeps its
assumed value.

Write each number down with how you measured it:

- [ ] **Wheel radius:** measure the wheel's diameter across the rollers, and
      halve it.
- [ ] **Wheel spacing, left to right:** centre of the left wheels to centre
      of the right wheels.
- [ ] **Wheel spacing, front to back:** centre of the front axle to centre of
      the back axle.
- [ ] **Footprint:** the whole robot's length, width and height, including
      anything that sticks out (the lidar, the plug, the relay).
- [ ] **Where the lidar sits:** how far behind the robot's front edge, and
      how high.
- [ ] **How often the lidar scans:** `ros2 topic hz /scan` on the Pi. The
      current number assumes 10 times a second; the LD14P is expected to be
      about 6.
- [ ] **Top speed:** hold the forward key at the speed you'll map at, count
      how many turns one wheel makes in 10 seconds, and multiply by the
      wheel's distance round. A wheel in the air turns a little faster than
      one on the floor, so this errs on the safe side.
- [ ] **Reaction time:** how long from a key to the wheels moving. A phone
      video of the Mac's screen and a wheel together shows it. If you can't
      measure it, leave the assumed value and write that down.

Then put them in, **as inputs only**:

- [ ] The body and wheel sizes go into the robot description,
      `robot_description/urdf/robot.urdf.xacro`.
- [ ] Top speed, reaction time, scan rate and the lidar's distance behind the
      front edge go into `robot_bringup/config/base_dynamics.yaml`. Leave
      `measured: false` until braking is measured too.
- [ ] Run `.claude/skills/run-checks/check.sh`. The checks calculate the new
      stop and caution distances (and the other sizes that follow from the
      body) and **fail until the configuration matches what they
      calculated.** Copy exactly the numbers the checks give. **Never type in
      a safety distance you chose yourself**, and never loosen one to make a
      check pass.
- [ ] Write down the new stop and caution distances. Stage 5 tests them.

The robot description is drawn for a two-wheel robot with a caster, and has
no setting for front-to-back wheel spacing. How it describes a four-wheel base
is [still to decide](#open-questions); measure the spacing anyway.

### Stage 5 — Stop tests

Every step of [safety-test-procedure.md](safety-test-procedure.md), and the
physical stop's own tests. **All of them with the wheels in the air first,
then on the floor.** Print [the stop-test form](runs/stop-test-form.md) and
fill it in as you go.

- [ ] Record every test (`robot_telemetry`).
- [ ] Do the in-the-air tests (part A of the form). **Any fail: stop and fix
      before going on.**
- [ ] **Measure braking, the first floor run.** Exclusion zone marked, tether
      attached, the second person holding the button. Drive at mapping speed
      past a mark, press the stop button at the mark, and measure how far it
      rolls. Three times; use the worst. Work out the deceleration (the form
      shows how), put it into `base_dynamics.yaml`, set `measured: true`, and
      run the checks again as in Stage 4.
- [ ] If the stop or caution distance changed, redo the obstacle test in the
      air (row A10) with the new distances.
- [ ] Then the floor tests (part B): slowest speed first, the same three
      rules.
- [ ] Commit the filled-in form (a scan or photo is fine) and the reports to
      `docs/runs/`.
- [ ] **Nothing drives on the floor, except for these tests, until every row
      passes.**

### Stage 6 — Map the room

- [ ] Clear the room of people who aren't taking part. Two people: one
      drives, the other holds the stop button. Tether the robot, mark the
      exclusion zone.
- [ ] Test the stop at the start of every session: press the button while
      driving slowly, check the wheels stop, twist it to release.
- [ ] Start `base_mapping.launch.py` inside `tmux` on the Pi (so a dropped
      ssh connection doesn't end it) and watch the map form in Foxglove.
- [ ] Drive slowly round the room until the map on screen covers all of it.
      **Once**, while driving slowly, press the software stop, check the
      robot stays stopped, and reset it.
- [ ] Save the map with the "save the map" command. If the live map went
      wrong, rebuild it from the recording with the "rebuild the map"
      command.
- [ ] Stop the launch with a single Ctrl-C and wait for it to finish, or the
      recording can be left unfinished.
- [ ] Check the report says `PASS`.
- [ ] Commit the map to `robot_bringup/maps/`, and the report to
      `docs/runs/`.
- [ ] **The proof of concept is done.**

---

## Open questions

These change what gets built or bought. Answer them before Stage 2.

1. ~~What does the laptop run?~~ **A Mac** (answered 2026-09-23). That's why
   all the ROS software runs on the Pi.
2. **How much memory does the Pi 4 have (1, 2, 4 or 8 GB)?** Check the box,
   or run `free -h` in Stage 1.
   - **2 GB** should manage one room.
   - **4 GB** is comfortable.
   - **1 GB** is probably too little to build the software on the Pi, unless
     you add swap space and build one part at a time.
3. **What voltage do the wheel sensors use?** Check the motor's label or the
   chassis listing. 3.3 V: buy nothing. 5 V only: buy the two level
   shifters.
4. **Does the power bank really supply 5 V 3 A?** If not, the Pi will slow
   itself down or reboot. On the Pi, `vcgencmd get_throttled` shows whether
   that has happened.
5. **Which room gets mapped?** Somewhere with space to clear, a door that
   closes, and no stairs.
6. **How does the robot description describe a four-wheel base?** It's drawn
   for two wheels and a caster. Whoever changes it in Stage 4 decides, and
   writes down why.
7. **Should the Pico tell the Pi when the motor power is cut?** Today the
   physical stop is invisible to the software: after it's released, a held
   key would move the robot at once. Stage 5 tests that it doesn't, but a
   wire from the relay's output to a Pico pin (through a voltage divider)
   would let the gate latch its own stop too. For whoever writes the Pico
   program.
8. **Does the battery cut out when the robot starts or turns?** See
   [the battery](#the-battery-can-only-supply-10-a). If it does, a gentler
   start in the Pico program may be enough; if not, the 20 Ah battery.

### Still to check, and by whom

| What | Who checks, and when | Why it matters |
|---|---|---|
| Prices and total at checkout to 94022 | You, before ordering | Amazon prices are from 09-28; two shops' shipping only shows at checkout |
| Fuse assortment includes 15 A and 2 A | You, before ordering | If not, buy those two at an auto parts store |
| Battery's F2 tabs vs the connectors you have | Whoever wires it, on arrival | Ring terminals don't fit F2 tabs. 0.25-inch female spades do |
| Relay says 12 V on it, and 40 A | Whoever wires it, on arrival | A 5 V or 24 V coil won't work from this battery |
| Relay's tab widths | Whoever wires it, on arrival | Sets which spade connectors fit it |
| Stop button: NC contact, and latches when pressed | Whoever wires it, on arrival | With a multimeter: closed when out, open when pressed, stays pressed until twisted |
| Wheel-sensor voltage | On arrival | Decides the level shifters. The chassis listing couldn't be read (AliExpress needs a login) |
| Motors' stall current | On arrival, from the motor label or listing | Four together must stay under the battery's 15 A, and well under the relay's 40 A |
| Which chassis variant arrived | On arrival | The 09-26 plan said to confirm it before ordering; nobody has recorded which one came |
| Driver program and Pico program exist and pass their tests | Stage 3 | Being written 2026-09-30 |
| Lidar program builds on Jazzy | Stage 1 | Builds on a PC (2026-09-30) only after two fixes, which `setup-pi.sh` applies. Still unconfirmed on the Pi |
| Driving keys work in a plain ssh terminal | Stage 0 or 3 | `teleop.launch.py` opens a window, which a Pi with no screen can't do |
| Battery charger's voltage suits this battery | You, on arrival | The charger gives 14.6 V; ExpertPower's page lists a 14.4 V charge voltage. Check the battery's manual |
| WAGO connectors are genuine | On arrival | |

## What changed on 2026-09-30, and why

- **The parts that were bought are the 09-26 list, not the 09-28 one.** The
  owner bought the mecanum chassis, motors, drivers, Pico, battery, charger,
  wire, fuse holder and terminals. No RC car was bought, and none will be.
- **The RC-car plan is dropped.** The robot is now driven by the Pi, not a
  radio remote, so the whole plan is back on the robot's own base.
- **The safety gate is back in the path.** Every drive command goes keys →
  gate → driver → Pico → motors, and the recording report can now reach
  `PASS`. The old plan expected `INCONCLUSIVE`, because the car had no gate.
- **All of safety-test-procedure.md applies again,** including the gate
  tests (steps 4 to 6) the RC-car plan had to skip.
- **The emergency stop still has to be bought.** Neither list that was
  actually bought included one. It's redesigned for this battery: a 12 V
  40 A relay held on by the stop loop, instead of the car's 5 V relay module
  powered from the power bank.
- **The battery is weaker than the 09-26 plan asked for.** The plan accepted
  the 10 Ah battery only with a 20 A BMS. Its maker says 10 A continuous,
  15 A for 10 seconds. So the main fuse is 15 A, not 20 A.
- **Ring terminals don't fit this battery.** It has 0.25-inch F2 tabs, so
  spade connectors are added.
- **Measuring the base is back** (Stage 4), before any floor driving. On the
  RC car it was forbidden, because a toy car's numbers would have become the
  real robot's safety distances. This base is the robot, so its numbers are
  the right ones.
- **New:** the [stop-test form](runs/stop-test-form.md) and the
  [wiring checklist](wiring-checklist.md).

## What changed on 2026-09-28, and why

Kept as history. None of this was bought, and the plan above replaces it.

- **Car instead of the custom base.** Chosen by the user. That removed:
  - the mecanum chassis, the four motor drivers and the Pico;
  - the 12 V battery and its charger;
  - the heavy wire, ring terminals, level shifters and jumper wires.
- **The emergency stop is bought now, not "later".** The plan never allowed
  floor driving without it, so a list without it could never finish the
  job.
- **Shipping is counted.** The old list said "before tax and shipping", so
  its ~$323 was never the real cost.
- **Estimated prices ("~") are replaced** by prices read from the listings,
  each marked with its seller.
- **The lidar source changed.** The $65.58 RobotShop price couldn't be
  confirmed: its page blocks automated checks, and search results show
  $77.45. The Amazon listing from Waveshare, the kit's maker, was checked
  directly.
- **The level-shifter question is gone.** The old list said none were needed
  for the motor drivers but listed two for the wheel sensors "only if 5 V".
  This car has neither.
- **"Done" is redefined.** The old goal needed a recording report that
  can't pass on a car with no safety gate.
- **Map builder changed** from SLAM Toolbox to Cartographer, because the car
  has no wheel sensors.
- **Stage 3 no longer changes `base_dynamics.yaml` or the robot
  description.** Numbers measured on a toy car would have turned into the
  real robot's safety distances.

---

## Future design, not included: the real robot

The real robot must travel between rooms and carry about 20 lb (9 kg). None
of that is in this proof of concept, and **a small base driven slowly round
one room says little about whether a robot can do it.**

**Probably carries over:** the lidar (at least as a second, short-range
one), the Pi if it has 4 GB or more, the Pico program, the driver program,
the emergency-stop button, tether and relay design, the Foxglove setup, and
what the map-building work teaches.

**Doesn't carry over:** this chassis and its motors, the 10 Ah battery
(10 A is far too little for a 9 kg payload), and the relay and fuse sizes,
which are chosen for this battery.

**Has to be worked out from measurements, not from listings:**

- **Motor torque and gearing** for the whole weight: payload, battery and
  body together.
- **Braking distance and top speed**, with the payload on, on each kind of
  floor. These go into `base_dynamics.yaml`, and the stop and caution
  distances are calculated from that.
- **Getting over door thresholds and rug edges**, and not tipping over when
  braking hard with the payload raised.
- **Battery and fuse sizing** from the motors' measured peak and stall
  current, with margin.
- **The full [safety-test-procedure.md](safety-test-procedure.md)**, again,
  on the real robot.
