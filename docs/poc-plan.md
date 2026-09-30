# Proof of concept: drive an RC car around one room and map it

**Goal:** a person drives a hobby RC car around one room with its own radio
remote. The car carries the Pi and the lidar, and the Pi builds a map of that
room from the lidar.

**Done means:** a saved map of the real room is committed to
`robot/ros2_ws/src/robot_bringup/maps/`, together with, in `docs/runs/`, the
recording of the mapping run and the written results of the stop tests in
Stage 4. The `robot_telemetry` report for this run will say `INCONCLUSIVE` or
`INCOMPLETE`, because this car has no safety gate for it to check. That is
expected, and it is **not** a pass.

**Not in this proof of concept:** driving to a goal by itself (Nav2), voice,
the operator console, and the safety gate. They all wait for the real robot.

Written 2026-09-23. Changed to an RC car and shopping list re-checked
2026-09-28. **Total: $221.41 before tax, shipping included, emergency stop
included** — under the $310 limit, but not yet confirmed at a checkout. See
[the shopping list](#shopping-list).

---

## Read this first: the car is outside the robot's safety system

The project's rule is that every drive command passes through the safety gate
(`robot_safety`). **This car breaks that rule, on purpose, for this proof of
concept only** (decided 2026-09-28). Its radio remote talks straight to the
car's own motor controller. So none of these can stop it:

- the safety gate,
- the software emergency stop,
- the "stop if commands stop arriving" rule.

Only two things can stop it:

1. **The emergency stop** on the shopping list. It cuts the car's motor power
   with a relay, whatever the remote is doing.
2. **The remote's own behaviour when its signal is lost.** No listing or
   manual for this car says what it does. Stage 4 tests it before the car
   touches the floor.

Because of this:

- **No ROS program may ever connect to the car's motor controller or radio
  receiver.** The Pi only listens to the lidar.
- **Nothing learned from this car counts as safety evidence for the real
  robot.** [safety-test-procedure.md](safety-test-procedure.md) still applies
  in full to the real robot.
- **Nobody who isn't taking part is in the room during a floor run.** That
  includes any elder or other vulnerable person. Every floor run has a
  tether, a marked exclusion zone, and a second person holding the emergency
  stop.

## The parts and what each one does

| Part | Job |
|---|---|
| Raspberry Pi 4 (already owned) | The computer on the car. Runs the lidar program and the map builder. |
| LD14P lidar kit (D200) | Spins and measures the distance to every wall and object around it. This is what the map is made from. Plugs into the Pi by USB. |
| ARRIS MN82S RC crawler (1/12 scale) | The car, with its own remote, battery and charger. A person drives it by hand. |
| Emergency stop box, relay, fuse | The physical stop: pressing the button cuts the car's motor power. |
| Tether wire, pull-apart plug, paracord | Carries the stop button's signal to the second person, and gives the car a leash. |
| USB power bank (already owned, must supply 5 V 3 A) | Powers the Pi and the lidar, and keeps the stop relay switched on. |
| A Mac laptop (already owned) | Where you watch the map form, in the Foxglove app. It doesn't need ROS. |

## Shopping list

### The answer

**Yes:** a complete list, including the emergency stop, comes to **$221.41
before tax** with shipping. That is $88.59 under the $310 limit.

- Two things aren't proven yet:
  - **Shipping to 94022 hasn't been confirmed at a checkout.** Amazon quoted
    its default address (Phoenix), not 94022.
  - **The car's behaviour when its radio signal is lost** is not documented
    anywhere. Stage 4 tests it with the wheels in the air. If it fails,
    return the car.
- **Before ordering:** put everything in one Amazon basket with ZIP 94022 and
  check the total is still under $310.
- **Tax:** Los Altos sales tax is 9.75%
  ([CDTFA](https://cdtfa.ca.gov/taxes-and-fees/rates.aspx), from July 2026).
  On $217.17 of parts that's about **$21.17**, so about **$242.58** with tax.

### What to buy

All prices checked on 2026-09-28. "Amazon-shipped" means the item qualifies
for Amazon's free delivery on Amazon-shipped orders over $35 (stated on each
listing).

| Part / job | Exact product | Qty | Item price | Shipping | Line total | Seller | What the listing confirms |
|---|---|---:|---:|---:|---:|---|---|
| Lidar | [Waveshare D200 kit with LD14P](https://www.amazon.com/dp/B0C4FGRLJX) | 1 | $71.99 | $4.24 * | $76.23 | waveshare, on Amazon (ships its own orders) | "Only 7 left". Maker's page: lidar, 9 cm cable, USB adapter; 5 V, ≤300 mA, 8 m range, Class 1 laser ([wiki](https://www.waveshare.com/wiki/D200_LiDAR_Kit)) |
| Car, remote, battery, charger | [ARRIS MN82S LC79 1/12 crawler](https://www.amazon.com/dp/B0DXFBZDHH), silver | 1 | $79.98 | $0 † | $79.98 | ARRIS, Amazon-shipped | "Only 10 left". 7.4 V 1200 mAh battery and USB charger included; 280 motor. Remote needs 2 × AA, not included |
| Emergency stop button | [BAOMAIN e-stop in a box](https://www.amazon.com/dp/B00NTT91Y0), 1 NC + 1 NO, twist to reset | 1 | $9.99 | $0 † | $9.99 | BAOMAIN, Amazon-shipped | In stock. Latching; only the NC contact is used |
| Motor power cut | [HiLetgo 5 V 30 A relay module, 2-pack](https://www.amazon.com/dp/B0CHFJSNP6) | 1 | $9.99 | $0 † | $9.99 | HiLetgo, Amazon-shipped | In stock. 30 A at 30 V DC contact; about 190 mA coil; jumper for high/low trigger. Second one is a spare |
| Fuse holder | [VANTRONIK inline ATC holder, 14 AWG, 3-pack](https://www.amazon.com/dp/B0DKJJJYX1) | 1 | $4.89 | $0 † | $4.89 | Amazon-shipped | In stock |
| Fuses | [ZIPCCI 80-piece ATC assortment](https://www.amazon.com/dp/B0B18NVT5Z) | 1 | $8.99 | $0 † | $8.99 | Amazon-shipped | In stock. Cheaper than any small fuse pack found |
| Tether wire (stop signal) | [HiFind 22 AWG 2-core, 50 ft, tinned copper](https://www.amazon.com/dp/B0F384B5QY) | 1 | $9.49 | $0 † | $9.49 | Amazon-shipped | In stock. Real copper, not copper-clad aluminium |
| Joins without soldering | [WAGO 221-412, bag of 10](https://www.amazon.com/dp/B072PT3JNL) | 1 | $7.47 | $0 † | $7.47 | Peppy Products, Amazon-shipped | In stock. Takes any wire from 24 to 12 AWG |
| Pull-apart plug on the car | [CENTROPOWER 5.5 × 2.1 mm DC plug pairs, 10 pairs](https://www.amazon.com/dp/B07C7VSRBG) | 1 | $9.39 | $0 † | $9.39 | CENTROPOWER, Amazon-shipped | In stock. 18 AWG, 9.5 in leads |
| Leash | [HERCULES 550 paracord, 50 ft](https://www.amazon.com/dp/B0BD2J2DQV) | 1 | $4.99 | $0 † | $4.99 | Amazon-shipped | In stock |
| **Total** | | | **$217.17** | **$4.24** | **$221.41** | | |

\* This fee was quoted for Amazon's default address (Phoenix 85001), not 94022.\
† Amazon's published rule: free delivery on Amazon-shipped orders over $35.
It has not been checked at a checkout for 94022.

### By seller

| Seller | Items | Shipping | Total |
|---|---:|---:|---:|
| waveshare (on Amazon, ships its own orders) | $71.99 | $4.24, fee quoted for Phoenix | $76.23 |
| Amazon-shipped (9 items) | $145.18 | $0 (over $35: free by policy) | $145.18 |
| **All** | **$217.17** | **$4.24** | **$221.41** |

No other fees, surcharges or import duties apply. Everything ships from
within the US.

### Already owned (not in the total)

| Item | Notes |
|---|---|
| Raspberry Pi 4 | Memory size still unknown (see open questions) |
| USB power bank, 5 V 3 A | Must also have a **second USB port**, which powers the stop relay |
| microSD card, 32 GB+ | For the Pi |
| USB cables | Includes the cable for the lidar's USB adapter. **One spare USB-A cable gets cut open** to power the stop relay |
| Wire stripper, crimper, multimeter, screwdrivers | |
| Zip ties, double-sided tape, a flat scrap of plastic or wood | Mounting plate for the Pi and lidar |
| Mac laptop | Runs the Foxglove app |
| 2 × AA batteries | For the car's remote. **Not confirmed as owned**; not included with the car |

### Alternatives checked, not counted

- **Lidar:** [Waveshare direct](https://www.waveshare.com/d200-lidar-kit.htm),
  $54.99. It ships from China, and the shipping cost and US import duties
  only show at checkout. [RobotShop](https://www.robotshop.com/products/360-omni-directional-triangulation-lidar-8m-d200-developer-kit-w-ld14p-lidar)
  blocks automated checks, so its price and shipping couldn't be confirmed.
- **Car:** [Traxxas TRX-4M Bronco](https://www.rcsuperstore.com/traxxas-1-18-trx-4m-ford-bronco-body-4x4-rtr-crawler-w-id-battery-usb-charger/),
  $179.95 with free shipping at RC Superstore. It's a better-known brand, but
  still with no written promise to stop when the signal is lost, and no slow
  mode. With the same stop parts the total is **$321.38, over the limit**.

## How it fits together

```
Motor power (inside the car):
  Car battery + ──[fuse]──[relay contact]──> car's motor controller ──> motor
  Car battery − ────────────────────────────> car's motor controller

The stop loop (keeps the relay switched on):
  Power bank 2nd USB port ──(cut USB cable)── +5 V ──┐
                                                     │
  on the car:   [pull-apart plug] <── 50 ft tether wire ──> [stop button, NC]
                       │                                    (second person)
                       └──> relay module power
  Power bank ground ─────────────────────────────────> relay module ground

Mapping:
  Power bank ──USB-C──> Pi 4 ──USB──> lidar
  Pi 4 ~~Wi-Fi (5 GHz)~~> Mac, Foxglove app

Driving:
  Car's remote ~~2.4 GHz radio~~> car's receiver  (no connection to the Pi)
```

**How the stop works:** the relay only lets power reach the motor while
current flows round the stop loop. Any of these opens the relay and cuts the
motor:

- pressing the button;
- pulling the tether until the plug comes apart;
- a broken wire;
- the power bank running flat.

Nothing in that list can make the motor start. The joins are made with the
WAGO connectors, so nothing needs soldering. Set the relay module's jumper to
"high trigger" and connect its signal pin to its own +5 V, so the relay is on
whenever the loop is closed.

**Where the relay goes:** cut the red wire between the car's battery plug and
its motor controller, and put the fuse and relay in that gap. The car then
cannot drive at all without the stop circuit connected, which is the point.

### Where each program runs

- **On the Pi:** the lidar program, the map builder, the recorder, and
  `foxglove_bridge`. None of them can move the car.
- **On the Mac:** the Foxglove app, showing the scan and the map as it
  forms. No ROS.
- **In the driver's hands:** the car's remote, which the Pi knows nothing
  about.

### Mapping without wheel sensors

The car has no wheel sensors, so the map builder has to work out how the car
moved from the lidar alone.

- **The current map builder can't do that.** SLAM Toolbox ignores every scan
  after the first unless something reports movement, so the map never grows
  ([maintainer's note](https://github.com/SteveMacenski/slam_toolbox/issues/221)).
- **Use Cartographer instead.** It has a ready-made Jazzy package for the Pi
  ([ROS index](https://index.ros.org/p/cartographer_ros/)) and can map from the
  lidar alone. Waveshare documents the same setup for a sibling lidar
  ([guide](https://www.waveshare.com/wiki/Cartographer_Map_Building)).
- **Drive slowly:** about a slow walk (0.3 m/s or less), and turn slowly. The
  lidar only scans about six times a second.
- **Expect drift** along long bare walls and near glass.
- **Record the lidar data** as well, so the map can be rebuilt afterwards if
  the live one goes wrong.

---

## The stages

Each stage ends in something you can see working. Don't start a stage until
the one before it works.

### Stage 0 — Before the parts arrive (software)

- [ ] Answer the [open questions](#open-questions) below.
- [ ] Install the Foxglove app on the Mac.
- [ ] Add a launch file for the real car. It starts the lidar program,
      Cartographer set to use the lidar only, and `foxglove_bridge`, all on
      the real clock. The existing `slam.launch.py` is fixed to the
      simulator's clock, so on the Pi it would wait forever.
- [ ] Decide which one program publishes the lidar's position on the car.
      LDRobot's launch file publishes its own, and it doesn't match this
      repository's names.

### Stage 1 — The Pi sees the room

Needs: Pi, lidar, power bank, Mac.

- [ ] Install Ubuntu 24.04 Server (64-bit) and ROS 2 Jazzy on the Pi, then
      build this repository on it.
- [ ] Put the Pi and the Mac on the same **5 GHz** Wi-Fi, check the Mac can
      log in with `ssh`, and give the Pi its own `ROS_DOMAIN_ID`. The car's
      remote uses 2.4 GHz.
- [ ] Find out how much memory the Pi has: run `free -h` on it.
- [ ] Build LDRobot's lidar program (`ldlidar_ros2`), which lists the LD14P.
      Nobody has confirmed it builds on Jazzy, so this step proves it.
- [ ] **Check:** Foxglove on the Mac shows the outline of the room, live,
      as you carry the Pi and lidar around.
- [ ] **Check:** carry them slowly round the whole room and watch a map
      form. This proves the mapping works before any car is involved.

### Stage 2 — Build the stop, wheels in the air

**The car sits on a box with all four wheels off the ground for this whole
stage.**

- [ ] Check the car's battery plug and wire size. Choose the fuse from the
      kit: the smallest one that doesn't blow while driving, and never
      bigger than the car's own battery wire can carry.
- [ ] Check the relay module says **05VDC** on the relay itself. The listing
      contradicts itself about this.
- [ ] Wire the fuse and relay into the car's red battery wire, and the stop
      loop through the tether and the button.
- [ ] **Check:** with the loop closed, the wheels turn from the remote. With
      it open, they don't.

### Stage 3 — Mount the lidar

- [ ] Fix the mounting plate to the car's **frame**, not only to its plastic
      body. Put the lidar on top, level, higher than everything else on the
      car. A tilt of 3° moves the scan about 15 cm up or down at 3 m.
- [ ] Strap the Pi and power bank down so nothing moves on a bump.
- [ ] Measure where the lidar sits relative to the middle of the car, and
      use it only for the lidar's position in the new launch file.
- [ ] **Don't** put the car's numbers into the robot description or
      `base_dynamics.yaml`. Those describe the real robot, and the safety
      distances are calculated from them.

### Stage 4 — Stop tests

All of these are done with the wheels in the air first. Two people take
part, and each result is written down.

- [ ] Press the button while driving slowly → the wheels stop. Time it.
- [ ] Pull the tether until the plug comes apart → the wheels stop.
- [ ] Unplug the power bank → the wheels stop.
- [ ] **Turn the remote off while driving slowly → the wheels stop.** This is
      the car's own signal-loss behaviour, which no manual confirms. **If the
      wheels keep turning, stop here: the car fails and goes back.**
- [ ] Twist the button to reset it, with the driver's hands off the remote →
      the wheels stay still.
- [ ] Repeat the button and tether tests on the floor, at the slowest
      speed. Mark an exclusion zone with nobody inside it, attach the
      tether, and have the second person holding the button.
- [ ] Record the run (`robot_telemetry`), and put the written results in
      `docs/runs/`.

These tests replace only what this car can be given. Steps 4–6 of
[safety-test-procedure.md](safety-test-procedure.md) test the safety gate,
which this car doesn't have. Those steps still apply to the real robot.

### Stage 5 — Map the room

- [ ] Clear the room of people who aren't taking part. Two people: one
      drives, the other holds the emergency stop. Tether the car.
- [ ] Record the run. Start the map builder on the Pi and watch it in
      Foxglove. Drive slowly round the room until the map on screen covers
      all of it.
- [ ] Save the map, commit it to `robot_bringup/maps/`, and commit the
      recording and the stop-test notes to `docs/runs/`.
- [ ] **The proof of concept is done.**

---

## Open questions

These change what gets built or bought. Answer them before Stage 0 finishes.

1. ~~What does the laptop run?~~ **A Mac** (answered 2026-09-23). That's why
   all the ROS software runs on the Pi.
2. **How much memory does the Pi 4 have (1, 2, 4 or 8 GB)?** Check the box,
   or run `free -h` in Stage 1.
   - **2 GB** should manage one room.
   - **4 GB** is comfortable.
   - **1 GB** is probably too little to build the software on the Pi, unless
     you add swap space and build one part at a time.
3. **Does the power bank really supply 5 V 3 A, and does it have a second
   USB port?** If not, the Pi will slow itself down or reboot. On the Pi,
   `vcgencmd get_throttled` shows whether that has happened.
4. ~~Which chassis?~~ **A hobby RC car driven by its own remote** (decided
   2026-09-28). See the warning at the top.
5. **Which room gets mapped?** Somewhere with space to clear, a door that
   closes, and no stairs.

### Still to check, and by whom

| What | Who checks, and when | Why it matters |
|---|---|---|
| Total at checkout to 94022 | You, before ordering | Nothing's been confirmed at a checkout yet |
| Car stops when its remote is turned off | You and the second person, Stage 4 | No manual says it does. It's one of only two ways to stop the car |
| Car's battery plug and wire size | Whoever wires it, Stage 2 | Sets the fuse size. Not stated anywhere; probably a small "SM" plug |
| Car can carry about 0.6 kg on top | You, Stage 3 | Only a reseller's "3 kg" claim, not the maker's |
| Car stays still when the stop is reset | You, Stage 4 | The listing doesn't say whether its motor controller waits for the throttle to be at rest |
| Relay really has a 5 V coil, and has a protection diode | Whoever wires it, on arrival | The listing contradicts itself |
| Lidar's USB adapter socket, and a cable to fit it | On arrival | The kit lists no USB cable |
| Lidar program builds on Jazzy | Stage 1 | Only "foxy and above" is claimed |
| WAGO connectors are genuine | On arrival | Sold by a reseller, not WAGO |

## What changed on 2026-09-28, and why

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
of that is in the $310 above, and **nothing about this RC car says anything
about whether a robot can do it.**

**Probably carries over:** the lidar (at least as a second, short-range
one), the Pi if it has 4 GB or more, the emergency-stop button and tether,
the Foxglove setup, and what the map-building work teaches.

**Doesn't carry over:** the car, its motor controller, the 5 V relay (the
real robot needs one sized for its own battery), and mapping from the lidar
alone. The real robot has wheel sensors.

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
- **The full [safety-test-procedure.md](safety-test-procedure.md)**,
  including the steps this car skips.
