# Wiring checklist: power, emergency stop, drivers and Pico

For Stages 2 and 3 of the [proof-of-concept plan](poc-plan.md). One check per
step: tick it, and don't go on until it's ticked. If a check fails, stop and
work out why before changing anything else.

**The base sits on a box with all four wheels off the ground for this whole
checklist.** Nothing here is done with the wheels on the floor.

If anyone who has wired a car or boat 12 V system can look over sections 2
and 3 before the battery is first connected, ask them.

## Words used here

- **Relay:** a switch worked by an electromagnet (its **coil**). Its heavy
  contacts, pins **30** and **87**, are open until 12 V reaches the coil pins,
  **85** and **86**. The numbers are printed next to the pins, usually with a
  small diagram on the side.
- **NC contact** ("normally closed"): a switch contact that is connected
  until the button is pressed.
- **Stop loop:** the thin wire circuit from the battery, through the tether
  and the stop button, to the relay's coil. While it's complete the relay
  holds the motor power on.
- **WAGO:** a lever connector. Strip 11 mm of wire, lift the lever, push the
  wire in fully, close the lever. Tug each wire to check.
- **B+ / B−** on a BTS7960 driver: its battery input. **M+ / M−:** its motor
  output.

## 0. Before every wiring session

- [ ] The main fuse is **out** of its holder.
- [ ] The Pico's USB cable is unplugged from the Pi.
- [ ] Rings, watches and metal bracelets off. A LiFePO4 battery shorted by a
      ring can burn a finger before its protection cuts in.
- [ ] Nothing metal lying across the battery's two terminals.

## 1. On arrival: check the parts before using them

- [ ] **Battery label** says 12 V (12.8 V), 10 Ah, LiFePO4. Multimeter across
      the terminals: between 12.8 and 13.6 V.
- [ ] **Battery terminals** are F2 flat tabs, 0.25 inch wide. **Ring terminals
      don't fit them.** Use yellow 0.25-inch female spade connectors
      (12–10 AWG) on the battery wires. Check whether the ring terminal kit
      has them before buying.
- [ ] **Battery protection (BMS):** 10 A continuous, 15 A for 10 seconds
      (ExpertPower's page). Write down if the label or manual says otherwise.
- [ ] **Main fuse is 15 A.** Not 20 A and not 30 A: the battery switches
      itself off above 15 A, so a bigger fuse would never blow. 15 A is also
      well within what 12-AWG wire carries.
- [ ] **Main fuse holder's own wires** are 12 AWG or thicker (printed on the
      insulation). If they're thinner, tell whoever is checking the plan
      before using it.
- [ ] **Stop-loop fuse is 2 A**, in the small 14-AWG holder.
- [ ] **Relay** says **12 V** (or 12 VDC) and **40 A** on its case. Not 5 V,
      not 24 V.
- [ ] **Relay coil**, measured with the multimeter on ohms between pins 85 and
      86: somewhere around 60–100 Ω. Write it down: ________ Ω.
- [ ] **Relay tab widths:** check which spade connectors fit pins 30 and 87
      (heavy wire) and 85 and 86 (thin wire).
- [ ] **Stop button, multimeter on the beep setting, across its NC
      terminals:** beeps with the button out; silent when pressed; stays
      pressed until twisted; beeps again after twisting. Mark the NC
      terminals with tape. (The other pair, NO, is not used.)
- [ ] **Wheel sensor voltage**, from the motor's label or the chassis
      listing: ________ V. 3.3 V allowed: no level shifters. 5 V only (or
      "3.5–5 V"): fit the level shifters in section 6.
- [ ] **Motor stall current**, if the label or listing gives it: ________ A
      each. Four together should stay under 15 A (the battery) and well under
      40 A (the relay). If they don't, say so before going on.
- [ ] **Which chassis variant arrived.** Write it down, and photograph the
      motor label.
- [ ] **Charger** says 14.6 V, 2 A, LiFePO4.
- [ ] **WAGO connectors** have the WAGO name moulded in.

## 2. Motor power path (battery disconnected throughout)

The whole path uses the 12-AWG wire. Red for positive, black for negative.

- [ ] Crimp a yellow spade connector onto a short red wire and a short black
      wire, for the battery. Tug-test each crimp.
- [ ] **Red:** battery + → main fuse holder, with the holder within 15 cm
      (6 inches) of the battery. **Fuse still out.**
- [ ] Fuse holder's other end → **WAGO A**. WAGO A will hold three wires: the
      fuse, relay pin 30, and the stop-loop fuse.
- [ ] WAGO A → relay pin **30** (red, yellow spade on the relay end).
- [ ] Relay pin **87** → **WAGO B** (red). WAGO B then feeds all four
      drivers.
- [ ] WAGO B → **B+** on each of the four drivers: four red wires.
- [ ] **Black:** battery − → **WAGO C**. WAGO C → **B−** on drivers 1 and 2,
      and one short black link wire → **WAGO D**. WAGO D → **B−** on drivers
      3 and 4.
- [ ] **Check (multimeter, beep setting, battery still disconnected):** relay
      pin 87 beeps to every driver's B+; the battery's − wire beeps to every
      driver's B−; relay pin 30 does **not** beep to pin 87; and **no red
      point beeps to any black point.**
- [ ] Fix the relay to the base plate by its tab, away from moving parts.
- [ ] Fix the battery to the chassis so it can't slide.

## 3. The stop loop (battery disconnected throughout)

Each half of the pull-apart plug has two wires, usually red and black. One
tether core carries the loop out to the button, the other brings it back, so
one plug carries both.

- [ ] Fix one half of the pull-apart plug to the robot's frame: the
      **robot half**. The other half goes on the tether: the **tether half**.
- [ ] Stop-loop fuse holder (2 A fuse fitted): one end into **WAGO A**, the
      other end → the robot half's **red** wire (a WAGO for the join).
- [ ] The robot half's **black** wire → relay pin **86** (red spade
      connector).
- [ ] Relay pin **85** → **WAGO C** (battery −), with a short black wire.
- [ ] 50 ft tether wire: at the robot end, its two cores → the tether half's
      red and black wires (a WAGO for each join).
- [ ] At the far end: the two tether cores → the stop button's two **NC**
      terminals.
- [ ] If the relay's diagram shows a diode across its coil, **86 must be the
      positive side**. If it shows a resistor or nothing, either way round
      works.
- [ ] Tie the paracord leash to the robot's **frame**, not to the plug or a
      wire. Make it about 30 cm longer than the tether wire, so a pull parts
      the plug before the leash goes tight.
- [ ] **Check (beep setting, battery still disconnected):** from WAGO A to
      relay pin 86 beeps with the button out and the plug together; silent
      with the button pressed; silent with the plug pulled apart.

## 4. First power-up: relay and drivers only (no Pico)

Drivers are wired to the battery; nothing is wired to their signal pins yet.

- [ ] Stop button **pressed**. Connect the battery spades. Fit the 15 A fuse.
- [ ] **Check:** relay silent. Multimeter (DC volts) between WAGO B and
      WAGO C: 0 V.
- [ ] Twist the button out. **Check:** the relay clicks. WAGO B to WAGO C:
      the battery's voltage, about 13 V. No motor moves. Nothing warms up.
- [ ] Press the button. **Check:** click, 0 V at WAGO B.
- [ ] Twist it out, then pull the tether plug apart. **Check:** click, 0 V.
- [ ] Plug together again. **Check:** click, about 13 V.
- [ ] Pull the 15 A fuse out. Tick Stage 2 in the plan.

## 5. The Pico and the drivers (motor power off)

**Main fuse out** for this whole section. The pin numbers come from the pin
table that comes with the Pico program in `robot/firmware/pico_base/`; this
checklist doesn't repeat them, so there's only one list to keep right.

- [ ] Stick the breadboard to the base plate. Pico's **3V3(OUT)** pin → the
      breadboard's red (+) rail. A Pico **GND** pin → the blue (−) rail.
- [ ] Each driver's **VCC** pin → the + rail (**3.3 V, never 5 V**). Each
      driver's **GND** pin → the − rail. Four of each.
- [ ] Each driver's **RPWM, LPWM, R_EN and L_EN** → the Pico pins the pin
      table gives for that wheel. Label each wire with its wheel (front left,
      front right, back left, back right).
- [ ] Each driver's **R_IS and L_IS** pins: leave unconnected unless the pin
      table uses them.
- [ ] Each motor's two power wires → its driver's **M+ and M−**.
- [ ] **Check (beep setting):** the − rail beeps to WAGO C (battery −),
      through the drivers' ground. That shared ground is required.
- [ ] **Check (beep setting):** the + rail does **not** beep to WAGO B or any
      driver's B+. If it does, stop: 12 V would reach the Pico.
- [ ] Plug the Pico's USB into the Pi, with the motor power still off.
      **Check:** the Pico's LED does whatever the Pico program's notes say it
      should.

## 6. Wheel sensors

- [ ] **If the sensors take 3.3 V:** each sensor's power wire → the + rail,
      ground → the − rail, its two signal wires → the Pico pins in the pin
      table.
- [ ] **If they need 5 V:** the sensors' power comes from the Pico's
      **VBUS** pin (5 V from USB) instead, and **each signal wire goes through
      a level shifter** before the Pico: shifter HV side to 5 V, LV side to
      3.3 V, both grounds to the − rail. **No 5 V wire ever touches a Pico
      pin directly.**
- [ ] **Check (volts, Pico powered, motors off):** each sensor signal pin
      measures between 0 and 3.3 V at the Pico, never more, as you turn its
      wheel slowly by hand.

## 7. Before driving anything (then go to Stage 3)

- [ ] Every wire is tied down clear of the wheels. Tug every WAGO wire and
      every crimp once more.
- [ ] Nothing bare can touch the chassis or another wire.
- [ ] The lidar sits on top, level, higher than everything else on the
      robot, including the relay and the plug.
- [ ] The Pi, the power bank and the battery are strapped down.
- [ ] **Power-on order, every time:** Pi and Pico first; stop button
      **pressed**; fit the 15 A fuse; second person ready; then twist the
      button out.
- [ ] **Power-off order, every time:** press the stop button, pull the 15 A
      fuse, then shut down the Pi. The relay's coil drains the battery slowly
      whenever the loop is closed, so the fuse always comes out at the end.
