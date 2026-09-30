# Stop-test results form

Print this, fill it in by hand during Stage 5 of the
[proof-of-concept plan](../poc-plan.md), and commit a scan or photo of it to
`docs/runs/` next to the recording reports. One form per session. It covers
every step of [safety-test-procedure.md](../safety-test-procedure.md) (marked
"Step 1" to "Step 7") and the physical stop's own tests.

## The rule

**If any row fails: stop here.** Press the stop button, write down exactly
what happened in "What happened", and don't do the next row. Fix the cause,
then start again **from the top of part A, on a new form**. A fix can break
something that passed before.

A row that wasn't done is not a pass. Write "not done" and why.

## About this session

| | |
|---|---|
| Date | |
| Room | |
| Floor surface (for part B) | |
| People present, and each one's job (driver / holds the stop / measures) | |
| Software version: `git rev-parse --short HEAD` on the Pi | |
| Pico program version: the same commit, unless it was built from another | |
| Stop distance in use (m) | |
| Caution distance in use (m) | |
| Speed setting used for the tests | |
| Payload (should be none) | |
| Battery charge at the start (voltmeter reading, V) | |
| Recording folder name(s) | |
| Report verdict(s) | |

**Before starting:** the second person has the stop button in their hand,
the Mac's Terminal is logged into the Pi, `base_mapping.launch.py` is running
and recording, and the driver has checked the gate says clear.

"Time to stop" is from the action to the last wheel standing still. A phone
video of the wheels is the easiest way to measure it; "under 1 s by eye" is
acceptable in part A.

## Part A — wheels in the air

The base sits on a box, all four wheels off the ground, for every row.

| # | What to do | What should happen | What happened | Time to stop | Pass / fail | Initials |
|---|---|---|---|---|---|---|
| A1 | **Step 1.** Put the base on the box. Spin each wheel by hand. | All four wheels turn freely and touch nothing. Nobody's hands are near them. | | — | | |
| A2 | **Step 2.** Hold a key to drive slowly. Second person presses the stop button. | Relay clicks off. All four wheels stop. | | | | |
| A3 | Drive slowly. Pull the tether until the pull-apart plug comes apart. | Relay clicks off. All four wheels stop. | | | | |
| A4 | **Step 3.** Button still pressed, driver's hands off the keys. Twist the button to release it. | Relay clicks on. The wheels **stay still**. Only a key press moves them. | | — | | |
| A5 | Hands off the keys. Push the pull-apart plug back together. | The wheels **stay still**. | | — | | |
| A6 | Drive slowly. Press the software stop (command in getting-started.md). | All four wheels stop. The gate reports an emergency stop. | | | | |
| A7 | Send `false` on the software stop. Then press a drive key. | The wheels **stay still**: `false` does not release the stop. | | — | | |
| A8 | Hands off the keys. Send the software stop's reset. Then press a key. | Still after the reset; they move only when the key is pressed. | | — | | |
| A9 | Drive slowly, then let go of the key. | All wheels stop within about half a second. | | | | |
| A10 | **Steps 4 and 5.** Hold a key at the slowest speed. Move a cushion slowly towards the lidar from the front. Measure where things happen with a tape. | The wheels slow once the cushion is inside the caution distance, and stop inside the stop distance. Write both distances. | | | | |
| A11 | **Step 6.** Drive slowly. Unplug the lidar's USB. | All four wheels stop, and stay stopped with a key held. | | | | |
| A12 | Drive slowly. Close the Terminal window on the Mac (drops the connection). | All four wheels stop. | | | | |
| A13 | Drive slowly. Unplug the Pico's USB. | All four wheels stop, and none twitches afterwards. | | | | |
| A14 | Plug everything back, restart. Drive slowly. Press Ctrl-C on `base_mapping.launch.py`. | All four wheels stop. | | | | |

**Part A all passed?** ☐ Yes — go on to the braking measurement. ☐ No — stop.

## Braking measurement — the first floor run

**Exclusion zone marked with tape, nobody inside it. Tether attached. The
second person holding the stop button.** Put a mark on the floor. Drive at
the speed you'll map at, and the second person presses the stop button as the
front of the robot passes the mark. Measure from the mark to the front of the
stopped robot.

| Run | Speed (m/s) | Distance rolled (m) | Initials |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |

Worst (longest) distance: ________ m at ________ m/s.

**Deceleration** = speed × speed ÷ (2 × worst distance) = ________ m/s².
It goes into `base_dynamics.yaml` as `max_deceleration`, with
`measured: true`; then run the checks, as in Stage 4 of the plan.

New stop distance: ________ m. New caution distance: ________ m.
**If either changed, redo row A10 in the air before part B.**

## Part B — on the floor

**Every row:** exclusion zone marked, nobody inside it, tether attached,
second person holding the stop button, slowest speed. Two people only in the
room.

| # | What to do | What should happen | What happened | Distance rolled / time to stop | Pass / fail | Initials |
|---|---|---|---|---|---|---|
| B1 | **Step 7, with step 2.** Drive slowly. Second person presses the stop button. | The robot stops. | | | | |
| B2 | Drive slowly. Pull the tether until the plug comes apart. | The robot stops. | | | | |
| B3 | **Step 7, with step 3.** Hands off the keys. Twist the button to release it, and reconnect the plug. | The robot **stays still**. | | — | | |
| B4 | Drive slowly. Press the software stop. Then send `false`, then press a key. | The robot stops, and stays stopped through the `false` and the key. | | | | |
| B5 | Hands off the keys. Send the reset. | Still until a key is pressed. | | — | | |
| B6 | Drive slowly, then let go of the key. | The robot stops within about half a second. | | | | |
| B7 | **Step 7, with steps 4 and 5.** Drive slowly straight at a cardboard box. | It slows inside the caution distance and stops before touching the box. Write the gap left. | | | | |
| B8 | **Step 7, with step 6.** Drive slowly. Unplug the lidar's USB. | The robot stops. | | | | |
| B9 | Drive slowly. Close the Terminal window on the Mac. | The robot stops. | | | | |

**Part B all passed?** ☐ Yes — the stop is tested; Stage 6 may start.
☐ No — stop.

## Anything unexpected

Write down anything odd, even if every row passed: a wheel that twitched, a
noise, a delay, the battery cutting out, a smell of hot electronics.

&nbsp;

&nbsp;

&nbsp;

Signed (driver): ______________________ Signed (holding the stop): ______________________
