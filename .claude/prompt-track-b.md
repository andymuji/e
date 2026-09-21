# Track B prompt — paste into Codex

Copy everything below the line into Codex as the first message.

---

You are working on a ROS 2 assistive robot for elder care. Read `AGENTS.md` at
the repo root before anything else — it holds the safety rules, and they are
not negotiable.

Start by branching from the current tip:

```bash
git fetch origin
git checkout -b track-b origin/derive-safety-margins-and-navigation
```

That tip is `7a1e1c3`. Confirm you are on it before you write anything. A
previous round lost significant time to branches cut from stale commits.

## What this project is

A robot that will drive around a house where an elderly person lives. Every
motion command passes through one gate, `robot_safety`, which is the only
thing allowed to publish to `/cmd_vel`, the topic that reaches the wheels.
Everything else — navigation, voice, the operator console — can only *request*
motion on `/cmd_vel_requested`. If your change creates a second publisher on
`/cmd_vel`, you have broken the single property this entire project rests on.

No robot exists yet. Nothing has ever driven to a goal. The URDF dimensions
and the braking numbers are placeholders.

## Your scope

Someone else is working in parallel on the simulator. **Do not touch these
files — they are theirs, and editing them will cause a merge conflict:**

- `robot/ros2_ws/src/robot_bringup/launch/simulation.launch.py`
- `robot/ros2_ws/src/robot_bringup/launch/navigation.launch.py`
- `robot/ros2_ws/src/robot_bringup/launch/slam.launch.py`
- `robot/ros2_ws/src/robot_bringup/launch/teleop.launch.py`
- `robot/ros2_ws/src/robot_bringup/config/nav2.yaml`
- `robot/ros2_ws/src/robot_bringup/maps/**`
- `docs/getting-started.md`

**These are read-only for you, permanently:**

- `robot/ros2_ws/src/robot_safety/**` — the gate. Not this round's work.
- `robot/ros2_ws/src/robot_core/**`
- `robot/ros2_ws/src/robot_description/urdf/robot.urdf.xacro` — placeholder
  dimensions, replaced only by measuring real hardware.
- `robot/ros2_ws/src/robot_bringup/config/base_dynamics.yaml` — same. Never
  change `measured: false` to `true`.

**Never edit `stop_distance` or `caution_distance` in any `safety*.yaml`.**
They are computed from `base_dynamics.yaml` by `robot_safety/distances.py`.
There is a hook that blocks this, but it only catches file-editing tools and
**not** `sed`, `awk` or heredocs — so the hook will not save you. If a test
says these numbers are wrong, the fix is upstream in the inputs, never in the
yaml. The same is true of the Nav2 footprint and inflation radius, which come
from the URDF via `robot_navigation/footprint.py`.

## How to verify your work

From the repo root, this one command is the whole check:

```bash
.claude/skills/run-checks/check.sh
```

It must print `PASSED`. Also check the `robot_safety` line reads a plain `OK` —
if it says `(skipped=N)`, the safety gate's tests stopped running and something
regressed. Do not use `python3 -m unittest` directly; it fails on imports
because the script sets `PYTHONPATH` for you.

You do **not** need ROS or Gazebo for any task below. `ros2`, `colcon` and `gz`
may not exist in your environment — that is expected and fine. If they are
absent, do not try to install them.

## Do the tasks in this order, one at a time

Commit after each one with a clear message and move to the next. Do not start
all three at once.

### Task 1 — a regression test for the safety topology

This is the most valuable thing you can do and it is fully verifiable locally.

The claim that only the safety gate publishes `/cmd_vel` has been checked
against a live running system exactly twice, by hand, and is never checked
again. CI has no Gazebo, so nothing would catch it breaking.

Write tests that prove the topology by reading the launch files as syntax
trees, without running ROS. `robot/ros2_ws/src/robot_bringup/tests/test_locations_launch.py`
already does exactly this — read it first and copy its approach. It parses
launch files with `ast` rather than searching text, so a package named in a
comment is not mistaken for a running node. Follow that.

Put your tests in a **new file** (for example `test_cmd_vel_topology.py`)
alongside the existing ones, not in `test_bringup.py`, so you do not collide
with the other track.

What to assert, across every launch file in `robot_bringup/launch/`:

- No node other than `robot_safety`'s `safety_node` publishes or is remapped
  onto `/cmd_vel`.
- Every Nav2 node that can emit a velocity has `cmd_vel` remapped to
  `cmd_vel_requested`. `behavior_server` matters most: its recovery behaviours
  drive the robot, and they run when something has already gone wrong. A live
  run found it has three such publishers, not one.
- The safety gate never appears in a lifecycle manager's node list. The
  lifecycle manager deactivates its nodes on failure, and the gate has to
  still be running then.
- Nothing in `robot_telemetry` or `robot_voice` publishes `emergency_stop_reset`.

Some of these may already be tested somewhere. Check first; do not duplicate.
If you find one is tested but weakly, strengthen it in your new file and say so.

### Task 2 — the hazard analysis

`README.md` lists "write the hazard analysis and emergency-stop test procedure"
as a founding task. The procedure exists at `docs/safety-test-procedure.md`.
The hazard analysis does not. Write it as `docs/hazard-analysis.md`.

This is a document, not code. It should identify what can go wrong with a
robot moving around a frail person, and for each hazard: how it happens, what
in the system currently catches it, and what does not. Read
`docs/safety-test-procedure.md` and `docs/decisions/0001-ros-baseline.md`
first so you are consistent with them.

Be honest about gaps rather than reassuring. Things genuinely not yet handled
include: no physical emergency stop is wired; the low-battery stop condition
is implemented but switched off everywhere (`low_battery_fraction: -1.0` in
both `safety.yaml` and `safety_navigation.yaml`), because the robot has no
battery to report; the localization stop is on under navigation
(`require_localization: true` in `safety_navigation.yaml`) but has never fired
in a real run, because nothing has ever navigated; and every distance in the
system derives from a robot nobody has measured. A hazard analysis that hides
these is worse than none.

Note the asymmetry between the two gate configurations — `safety.yaml` for
plain simulation has `require_localization: false`, `safety_navigation.yaml`
has it `true`. Check that this is deliberate and say so either way.

### Task 3 — Nav2 Collision Monitor

`README.md`'s integration order calls for Nav2's Collision Monitor as an
independent second layer beside `robot_safety`. It is absent from the repo.

Create **new files only**:

- `robot/ros2_ws/src/robot_bringup/config/collision_monitor.yaml`
- `robot/ros2_ws/src/robot_bringup/launch/collision_monitor.launch.py`
- a test for both in `robot_bringup/tests/`

Do not wire it into `navigation.launch.py` — that file belongs to the other
track and will be connected at merge. `setup.py` already globs
`launch/*.launch.py`, so a new launch file needs no shared edit.

Two things that matter:

1. Its stop zone geometry must derive from `robot_navigation/footprint.py`'s
   `BaseFootprint`, the same source as the Nav2 costmap footprint — not typed
   in by hand. Write the test so it recomputes the numbers and fails if the
   yaml drifts. `test_bringup.py`'s `test_both_costmaps_use_the_urdf_footprint`
   and `test_inflation_covers_the_circumscribed_radius` show the pattern.
2. The Collision Monitor is a *motion source constraint*, not a motion
   authority. Configure it so it still cannot put a velocity on `/cmd_vel`
   ahead of the gate. Argue in a comment at the top of the yaml why the
   arrangement you chose preserves that.

Base your zone choices on the hazard analysis you wrote in Task 2.

## Working style

- Prefer proper file-editing over `sed` and heredocs. This repo has a guard
  that only sees the former, and a previous agent round was bitten by exactly
  this.
- Explain what you are doing in plain language, not jargon. It will be read by
  someone who does not read code.
- Write the least code that does the job, but never trim a safety check to
  make something simpler.
- When you finish a task: run the check script, commit, push to `track-b`.
- If something looks wrong in code you have been told not to touch, **say so
  and stop** rather than fixing it. That is how the last two rounds found
  their real bugs.
