# Hazard Analysis

- Status: First draft
- Date: 2026-09-22
- Covers: the software in this repository, as it stands today
- Companion documents: [safety-test-procedure.md](safety-test-procedure.md),
  [decisions/0001-ros-baseline.md](decisions/0001-ros-baseline.md)

## What this document is

This is the list of ways a robot driving around an elderly person's home could
hurt someone, and an honest account of which of them the software currently
catches.

It is deliberately uncomfortable reading. A hazard analysis that reassures is
worse than none at all, because the whole point of writing one is to find the
gaps while they are still cheap to fix. Where something is not handled, this
document says so plainly rather than describing the thing next to it that is.

**No robot exists yet.** Nothing has ever driven to a goal, in simulation or
anywhere else. Every distance in this system is calculated from a robot nobody
has measured. That fact colours every row below and is itself hazard H-14.

## The one thing everything rests on

There is a single rule the whole design hangs from:

> Every command that could turn a wheel passes through one piece of software,
> `robot_safety`, and that is the only thing allowed to talk to the wheels.

Navigation, the voice assistant and the operator console can all *ask* for
motion. None of them can *cause* it. They publish onto a topic called
`/cmd_vel_requested`, the gate reads it, decides how fast the robot may
actually go, and publishes the result onto `/cmd_vel`, which is what reaches
the motors.

If a second program ever publishes onto `/cmd_vel`, that one rule is broken
and most of the table below stops being true — because a command that took the
short cut was never slowed down, never stopped, and never checked. This has
been confirmed against a live running system three times by hand, and is now
also checked automatically on every change.

## How to read what follows

Each hazard has four parts:

- **How it happens** — the sequence of events, in plain language.
- **Caught by** — what in the system today would prevent or limit it.
- **Not caught** — what would still get through. This is the important part.
- **Severity** — how bad the outcome is for the person, not how likely it is.
  Likelihood is unknowable until the robot exists and has been observed.

Severity is one of:

| Level | Meaning |
|---|---|
| **Critical** | Could cause serious injury or death to a frail person |
| **High** | Could cause injury, or a fall |
| **Moderate** | Could distress, obstruct or trap without direct injury |
| **Low** | Nuisance, loss of function, or damage to the robot |

---

## A. The robot contacts a person

### H-01 The robot drives into a person in front of it — **Critical**

**How it happens.** The robot is moving forward under navigation. A person
steps into its path, or is already there and the robot did not stop in time.

**Caught by.** The safety gate measures the nearest obstacle on every scan. At
or inside `stop_distance` (currently 0.45 m) it commands zero speed. Between
`stop_distance` and `caution_distance` (0.85 m) it caps speed at 35%. Those
two numbers are not chosen by hand — they are calculated from the robot's top
speed, its braking ability, how old a sensor reading can be, how long the
motors take to respond, and a 1.5× safety factor, and a test fails if the
configuration drifts away from that calculation.

**Not caught.** The calculation is only as good as its inputs, and the inputs
are assumptions (H-14). Nobody has measured how hard this robot can brake, on
carpet, with a payload, on a worn drivetrain. The stop has never been
performed by a real robot at speed.

---

### H-02 The robot hits something the lidar cannot see — **Critical**

**How it happens.** The lidar is a single horizontal slice at one height
(0.15 m above the floor in the current placeholder design). It sees that slice
and nothing else. A person lying on the floor after a fall, a foot, a walking
frame's lower rail, a sleeping cat, a dropped walking stick, a low footstool,
a child — any of these can be entirely below the beam. A table top or a
person's outstretched arm can be entirely above it.

This is the single most dangerous limitation in the system, and it is a
property of the sensor, not a bug in the code. **A person who has fallen — the
exact event this robot exists to help with — is lying in the one place the
robot cannot see.**

**Caught by.** Nothing in software. The gate stops for what the lidar reports;
it has no way to know what the lidar missed.

**Not caught.** Everything below or above the scan plane. There is no depth
camera, no bumper, no contact sensor, and no motor-current monitoring in this
repository or in the robot description.

**What would close it.** A second sensing mode at a different height or in 3D,
plus a physical bumper that cuts motor power on contact rather than reporting
it to software. The hardware principles in `README.md` already call for bumper
and contact sensors; none is specified or wired yet.

---

### H-03 The robot turns into someone standing beside it — **High**

**How it happens.** The robot is stationary or moving slowly and rotates on
the spot. Its corners sweep further out than its sides. Someone standing close
beside it is struck by a corner.

**Caught by.** Partially, and by accident rather than design: because the gate
takes the nearest obstacle in *any* direction (see H-04), an obstacle beside
the robot does stop it. Nav2's inflation radius is also set from the
circumscribed radius — the circle that contains the robot however it is turned
— so the planner does not plan rotations into places the robot cannot rotate.

**Not caught.** The gate has no notion of which way the robot is being asked
to move, so its protection here is a side effect. A directional stop zone
(H-04's fix) must be designed so that it does not lose this.

---

### H-04 The robot cannot pass through a doorway, and the obvious fix is dangerous — **High**

**How it happens.** This one is not a collision; it is the pressure that leads
to one.

The gate stops on the nearest obstacle *in any direction*. The scan covers a
full circle, and `safety_node.py` takes the minimum over the whole circle with
no filtering by angle. So a wall 0.4 m to the robot's left stops the robot as
firmly as a person 0.4 m in front of it, even though the robot is driving
forward and away from that wall.

The consequence, worked through with the current numbers: the stop distance is
0.45 m and the robot is 0.30 m wide, so the robot needs a corridor wider than
**1.20 m** to move at all. An interior door is typically 0.76–0.81 m wide. A
domestic hallway is often under 1.1 m.

**As configured, this robot would stop dead in the doorway of the home it is
meant to work in, and stay stopped.**

**Caught by.** The behaviour is safe — it fails closed, and a robot that
refuses to move injures nobody. It has simply never been observed, because
nothing has ever navigated. The one room it has been simulated in is an open
6 × 5 m space with no doorways.

**Not caught.** The organisational hazard that follows. When an operator finds
the robot frozen in every doorway, the tempting fix is to reduce
`stop_distance` until it fits — which is precisely the number that decides
whether the robot stops in time for H-01. The configuration files and the
automated drift checks make that hard to do quietly, but they do not make it
impossible to do deliberately for the wrong reason.

**What would close it properly.** Stop zones shaped like the direction of
travel, rather than a single circle: a forward-facing stop polygon lets the
robot pass a wall it is driving parallel to while still stopping for what is
ahead of it, without touching the distance that protects H-01.

Nav2's Collision Monitor provides exactly that shape, and it has now been
added — but **adding it does not by itself close this hazard, and it would be
wrong to record that it has.** The Collision Monitor is an extra constraint
layered *in front of* the gate; it can only ever make the robot more cautious,
never less. The gate behind it still takes the nearest obstacle in every
direction, so the robot still stops in the doorway. What the Collision Monitor
does is establish the directional layer, derived from the same dimensions, so
that the remaining decision is a narrow one. As of 2026-09-23 that layer is no
longer only configuration: the node is installed, included by
`navigation.launch.py`, and has been run, reaching the active state and
publishing back a stop zone matching the derived geometry. It has still never
slowed or stopped a *moving* robot, because nothing has navigated yet, so its
behaviour against a real obstacle remains unproven - and H-04 is untouched
either way.

**That remaining decision is a change to `robot_safety`, which this round did
not make.** Narrowing the gate's field of view is a real safety change to the
one component everything else rests on. It should be made deliberately, with a
measured robot and a test that proves the narrowed gate still stops for H-01
and H-03, not as a side effect of tidying up. It is recorded here for that
decision.

---

### H-05 The robot pins a person against a wall or furniture — **Critical**

**How it happens.** A person is between the robot and a fixed obstacle. The
robot stops when the lidar sees the person, but stopping is not the same as
retreating, and a frail person trapped in a corner by a stationary robot
cannot necessarily push past it.

**Caught by.** The stop itself, which stops the squeeze getting worse.

**Not caught.** Everything after the stop. Nothing in the system backs the
robot away, and nothing tells anyone that a person is trapped. Worse, the
latched emergency stop (H-08) means that if the person or a bystander presses
stop, the robot stays exactly where it is until an operator resets it — which
is right for a runaway robot and wrong for a trapping one.

**What would close it.** A deliberate, slow, operator-authorised retreat
behaviour, and an alert path to a carer. Neither exists. This needs a design
decision before hardware, not after.

---

### H-06 The robot tips over onto someone — **Critical**

**How it happens.** A differential-drive base with a caster crosses a
threshold strip, a rug edge or a raised doorway sill at an angle, or
decelerates hard while carrying a raised payload, and topples.

**Caught by.** Nothing. The speed cap and the deceleration limit make it less
likely, but they were not chosen with tipping in mind.

**Not caught.** All of it. The URDF has placeholder masses and no analysis of
the centre of gravity, the wheel and caster geometry against real thresholds,
or the maximum safe deceleration with a payload. There is no tilt or
fall-detection stop condition, even though the robot description already
includes an IMU that could feed one.

**What would close it.** A stability analysis once the base is chosen, and a
tilt stop condition in the gate fed from the IMU. The gate's structure would
accommodate one; it does not have one.

---

## B. The robot fails to stop when it should

### H-07 The obstacle sensor fails, freezes or lies — **Critical**

**How it happens.** The lidar loses power, its cable works loose, the driver
crashes, or it keeps publishing the last scan it managed to take. The robot
keeps driving on a picture of a room that is no longer in front of it.

**Caught by.** The gate treats a scan older than `sensor_timeout` (0.5 s) as
no scan at all, and stops. It also stops on a scan with no usable readings,
and on individual readings that are missing, infinite or not-a-number. This is
the fail-closed rule: *the absence of information is a stop condition, never a
reason to continue.*

**Not caught.** A sensor that is working, publishing fresh data, and wrong —
for example a lidar pointing through a glass door, a mirror, or a dark
matt-black surface that returns no echo. To the gate, "nothing there" and "I
cannot see it" are the same reading. Glass and mirrors are common in homes.

---

### H-08 The emergency stop is pressed and the robot carries on — **Critical**

**How it happens.** The obvious worst case. Someone presses stop and the robot
does not.

**Caught by, in software.** The stop is latched: once engaged, the only thing
that releases it is an explicit reset message on `emergency_stop_reset`. A
`false` on the stop topic does not release it, a new destination does not
release it, and a restart of the navigation stack does not release it. This
was confirmed on a live run: after a spoken "stop", a subsequent "go to the
kitchen" did not move the robot.

It has also been confirmed on a live system that **nothing in a running graph
publishes `emergency_stop_reset` at all** — not the voice assistant, not the
recorder. The only way to release the latch is a deliberate operator action.

**Not caught: there is no physical emergency stop.** Everything above is
software stopping software. A normally-closed circuit that physically removes
motor power does not exist, is not specified, and is not wired. If the onboard
computer freezes, if the motor driver latches its last command, or if the
software stop message never arrives, none of the above helps.

**This is the largest single gap in the project.**
`docs/safety-test-procedure.md` already refuses to accept a software test as a
substitute, and `decisions/0001-ros-baseline.md` records the software stop as
explicitly secondary. The hardware it is secondary to has not been bought.

---

### H-09 The connection to the robot is lost mid-drive — **High**

**How it happens.** Wi-Fi drops, the navigation process dies, or the operator
console's machine goes to sleep, while the robot is moving.

**Caught by.** The gate stops. A motion request older than `command_timeout`
(0.5 s) is discarded rather than held, so losing the command stream stops the
robot instead of freezing it at its last speed. The gate runs on the robot,
not on the network, so it keeps running when the network does not.

**Not caught.** If the machine running the gate itself dies, nothing stops the
robot except the physical circuit that does not exist (H-08).

---

### H-10 The robot loses track of where it is and drives confidently to the wrong place — **High**

**How it happens.** Localization diverges — the robot slips, someone moves the
furniture the map was built from, or it is carried. Navigation keeps planning
against a pose that is wrong, so the robot drives purposefully into a wall, a
staircase, or a person.

**Caught by.** Under autonomous navigation the gate requires a fresh, credible
pose. `require_localization: true` in `safety_navigation.yaml` stops the robot
if the localization transform goes stale for more than a second, and
separately if the position uncertainty exceeds 0.25 m² — because the localizer
keeps publishing confidently after it has diverged, so freshness alone proves
nothing.

**Not caught.** *This stop has never fired.* Nothing has ever navigated, so a
check that exists in code and in configuration has never been observed
stopping a robot. A stop condition that has never triggered is a claim, not a
feature.

**On the asymmetry between the two configurations.** `safety.yaml`, used for
the plain teleoperated simulation, has `require_localization: false`.
`safety_navigation.yaml`, used when Nav2 drives, has it `true`. **This is
deliberate and correct.** Under teleoperation a human is looking at the robot
and choosing every movement; there is no pose to lose, and a gate demanding
one would refuse to move at all — which teaches operators to ignore the gate,
the worst outcome of all. Under Nav2 the pose *is* the thing choosing where to
drive, so a lost pose is a direct cause of H-10. The asymmetry is checked by a
test (`test_autonomous_navigation_requires_localization`) so it cannot be
flattened by accident.

---

### H-11 The battery dies mid-trip — **Moderate**

**How it happens.** The robot runs out of charge and stops, possibly in a
doorway, at the top of a stairwell, or across the path a person uses at night.

**Caught by.** In principle, a low-battery stop condition is implemented in
the gate and covered by tests.

**Not caught.** **It is switched off everywhere.** `low_battery_fraction` is
`-1.0` — meaning "disabled" — in both `safety.yaml` and
`safety_navigation.yaml`, because the simulated robot has no battery to
report. Nothing will turn it on automatically when a real battery appears; a
person has to remember.

There is also no layer above the gate that breaks off a trip and returns to a
charger while there is still charge to get there. The gate is the floor, not a
power policy: by the time it fires, the only safe thing left is to stop, which
is how the robot ends up stopped somewhere inconvenient. See also H-13.

---

### H-12 A staircase — **Critical**

**How it happens.** The robot drives over the top step. The lidar's horizontal
slice sees open air above a descending staircase exactly as it sees an empty
room.

**Caught by.** Nothing whatsoever. There is no cliff or drop-off sensor in the
robot description, no keep-out zone configured on the map, and no stop
condition for it in the gate.

**Not caught.** All of it. The map produced by SLAM marks walls, not floors;
an open stairwell is free space on the map and free space to the lidar.

**What would close it.** Downward-facing cliff sensors wired into the same
physical circuit as the emergency stop, plus Nav2 keep-out zones drawn on the
map around every drop. Both are absent. **Until at least one of them exists,
this robot must not be operated on any floor that has an unguarded staircase,
and the exclusion zone in the test procedure must be chosen with that in
mind.**

`README.md` also discusses a stair-climbing mechanism as a possible future
direction. That is a different machine with a different hazard analysis; it
must not be started before this row is closed.

---

## C. Something asks the robot to do the wrong thing

### H-13 The robot stops safely, in an unsafe place — **Moderate**

**How it happens.** Every stop condition above ends in the same action: stand
still, right here. In a corridor, a doorway, or on the route from a bed to a
bathroom at night, a stationary robot is an obstacle for someone who may be
unsteady on their feet and may not see it.

**Caught by.** Nothing. Stopping is treated throughout as unconditionally
safe.

**Not caught.** The entire question. There is no "stopped, and in the way"
state, no alert to a carer, no light or sound to mark the robot's position,
and no consideration of where a stop leaves the robot.

**Note.** This is the one hazard where the safety design works *against* the
person, and it deserves a design decision rather than a tweak. The right
answer is probably not "move after stopping" — that undermines H-08 — but
"make the stopped robot conspicuous and tell somebody".

---

### H-14 The robot is not the shape or speed the software thinks it is — **Critical**

**How it happens.** Every distance in this system is derived from two files of
assumptions: the placeholder dimensions in `robot.urdf.xacro` and the assumed
dynamics in `base_dynamics.yaml`, which says `measured: false` at the top. The
stop distance, the caution distance, the planner's footprint and the inflation
radius all come from those numbers. If the real robot is faster, heavier,
wider, or brakes worse than assumed, every margin in the system is wrong in
the dangerous direction at once.

**Caught by.** The derivation itself, which is the strongest structural
protection in the repository. The distances are computed, not typed; tests
recompute them and fail if the configuration drifts; a hook refuses edits that
touch them directly; and `base_dynamics.yaml` records honestly that it is
unmeasured. Change the robot's dimensions and the planner's footprint changes
with them, automatically.

**Not caught.** The fact that the inputs are guesses. The machinery guarantees
the numbers are *consistent*, not that they are *right*. Consistency with a
fiction is still fiction.

**What must happen.** Measure the base, set `measured: true`, re-derive, and
re-run the full procedure in `docs/safety-test-procedure.md`. This is the
gating item for all hardware work and the reason the project is not blocked on
anything else.

---

### H-15 Speech is misheard and the robot acts on it — **High**

**How it happens.** Speech recognition is unreliable by nature, and more so
with an elderly speaker, a hearing aid, a television in the background, or an
accent the model was not trained on. Someone says something the robot hears as
a destination.

**Caught by.** The voice path cannot cause motion; it can only request a named
destination from a fixed allow-list, and an unrecognised destination is
refused rather than guessed at. **Nothing in the voice package can publish
`emergency_stop_reset`** — a voice that can undo a stop is not a stop — and
this has been confirmed against a live graph, not only by reading the code.
Everything the voice path asks for still passes through navigation and the
gate.

**Not caught.** A correctly-recognised command from the wrong person — a
visitor, a television, a grandchild. There is no speaker identification and no
confirmation step for a destination.

---

### H-16 Speech is misheard *when someone is trying to stop the robot* — **Critical**

**How it happens.** The urgent case, and the reverse of H-15. Someone shouts
"stop" and the recogniser returns "sto", "hal" or "wait".

**Caught by.** This was a real defect, found by a live run and now fixed. A
mangled stop word used to be treated as an unrecognised command — a refusal —
and a refusal does not cancel a trip that is already running. It is now a
`halt`: the trip is cancelled, while the emergency-stop latch is deliberately
left untouched, because a half-heard word should not be able to engage a latch
that only an operator can release. The same handling now applies in the
operator console. The reasoning is recorded in
[decisions/0002-voice-provider.md](decisions/0002-voice-provider.md).

**Not caught.** A stop that is not heard at all — over a television, through a
closed door, or from a person who has fallen and cannot raise their voice.
**Voice is not an emergency stop and must never be presented to a family as
one.** The emergency stop is the physical button that does not exist yet
(H-08).

---

### H-17 Something bypasses the gate — **Critical**

**How it happens.** A new feature, a debugging session, a well-meant
contributor, or a copied launch file publishes directly onto `/cmd_vel`, or
plugs into the motor driver's velocity input. Every protection in sections A
and B is bypassed in one line.

**Caught by.** Quite a lot, now.

- Every Nav2 component that can emit a velocity is redirected onto the request
  topic, including `behavior_server` — whose recovery behaviours drive the
  robot, and which run precisely when something has already gone wrong. A live
  run found three velocity publishers under `behavior_server`, not one.
- The gate is deliberately **not** placed under lifecycle management, because
  the lifecycle manager shuts its nodes down on failure and the gate is the
  thing that must still be running then. Confirmed on a live system: the gate
  has no lifecycle interface for the manager to shut it down through.
- Three separate live runs each found exactly one publisher on `/cmd_vel`.
- Automated tests now read every launch file in the repository and fail if any
  of the above changes.

**Not caught.** Code that is not a launch file. Someone running
`ros2 topic pub /cmd_vel ...` by hand, or wiring a second board to the motor
controller, is outside what any test can see. That is why it is also written
as a rule in `AGENTS.md`, and why `robot_telemetry` — the flight recorder — is
built with no publisher of any kind: a witness that can act is not a witness.

---

### H-18 A stop that was never tested is reported as working — **High**

**How it happens.** Not a robot hazard; a *process* hazard, and the way the
other seventeen rows get quietly downgraded. A recorded run is attached to a
change as evidence. The run never happened to exercise the emergency stop, but
the report's headline says PASS and its exit code says success, so a reviewer
reads the headline and moves on.

**Caught by.** The report distinguishes a check that passed from one the run
never put to the test, and the test procedure states in writing that the
latter is deliberately not a pass.

**Not caught, until this round.** The headline and the exit code — the two
things a reviewer and an automated check actually read — did not honour that
distinction. This was found on a real recording in `docs/runs/` and has been
fixed; see `docs/runs/README.md`.

---

## What this analysis concludes

Three things are true at once, and all three need saying.

**The structure is sound.** One gate, fail-closed on every missing input, a
latched stop, distances derived rather than typed, and a flight recorder that
cannot act. Those are the right bones, and the live runs have confirmed the
topology rather than merely asserting it.

**The sensing is not.** A single horizontal lidar slice cannot see a fallen
person, a footstool, a staircase or a glass door, and no amount of careful
software fixes that. H-02 and H-12 are the two rows that should decide the
hardware budget.

**Nothing has been tested on a robot, because there is no robot.** Five rows
above (H-01, H-06, H-08, H-10, H-12) depend on hardware that has not been
bought, and one (H-14) says every number in the system is a guess. The
software is genuinely as complete as it can be without the machine; it is not
in any sense proven.

### The five things to do next, in order

1. **Buy and wire a physical emergency stop** — a normally-closed circuit that
   removes motor power independently of any software (H-08).
2. **Decide the drop-off answer before the robot touches a floor with stairs
   on it** — cliff sensors, keep-out zones, or a hard restriction to one level
   (H-12).
3. **Measure the base** and set `measured: true`, which unblocks everything
   downstream (H-14).
4. **Add a second sensing mode** at a different height, and a physical bumper
   (H-02).
5. **Decide whether the gate should keep looking in every direction at once**,
   now that the Collision Monitor provides a directional layer — because until
   that is decided the robot cannot leave the room (H-04).

The first four need hardware. The fifth needs a measured robot and a
deliberate decision about the gate, so it is not something to slip into a
tidying-up pass either. What *has* been done in software is the groundwork for
it: the Collision Monitor now exists as a second, independent, directional
layer in front of the gate, derived from the same dimensions as everything
else.

### Deliberately out of scope

- The stair-climbing mechanism discussed in `README.md`. A different machine,
  a different analysis.
- Privacy, data protection and recording consent for the voice and camera
  paths. Real and necessary, but not a physical-injury hazard, and it deserves
  its own document.
- Medical decisions of any kind. This robot fetches things and moves around; it
  is not a medical device and must not be described as one.
