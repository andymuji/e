# Documents

A map of this folder, in the order worth reading them.

How it is laid out:

- `safety/` — the hazard analysis, the stop-test procedure, the wiring checklist.
- `guides/` — getting started, the proof-of-concept plan, privacy, useful outside projects.
- `runs/` — recorded runs, the paper stop-test form, and the room scans.
- `decisions/` — why the big choices were made.
- `foxglove/` — the saved layout for watching the robot in Foxglove.

## Read these in order

1. **[safety/hazard-analysis.md](safety/hazard-analysis.md)** — what a robot moving around a
   frail person can do to them, and which of those things the software
   currently prevents. Written to be uncomfortable rather than reassuring. If
   you read one document here, read this one.

2. **[safety/safety-test-procedure.md](safety/safety-test-procedure.md)** — what has to be
   done, and written down, before anyone stands near a moving robot. Also
   explains how a run is recorded and what the resulting report does and does
   not prove.

3. **[guides/getting-started.md](guides/getting-started.md)** — how to build the code and
   run the simulation yourself. This is the one that assumes you are at a
   terminal.

## The plan for the first real robot

**[guides/poc-plan.md](guides/poc-plan.md)** — the proof of concept: drive the robot around
one room by remote control and map it. The parts, how they connect, and the
stages in order, each ending in something you can see working.

**[safety/wiring-checklist.md](safety/wiring-checklist.md)** — wiring the battery, the
emergency stop, the motor drivers and the Pico, one check per step, with the
wheels in the air throughout.

**[guides/useful-repos.md](guides/useful-repos.md)** — other people's code and
Claude add-ons worth using, grouped by when we will need them, with license and
safety notes.

## Evidence

**[runs/](runs/README.md)** — reports from real recorded runs, committed as
evidence rather than summarised in a commit message. Includes one report kept
specifically because it was wrong, and the fix it led to.

## Decisions

Records of the choices that would be expensive to reverse, each written when
the choice was made rather than reconstructed afterwards.

- **[decisions/0001-ros-baseline.md](decisions/0001-ros-baseline.md)** — why
  Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic, and the safety consequences
  that follow from that choice.
- **[decisions/0002-voice-provider.md](decisions/0002-voice-provider.md)** —
  why no speech-recognition engine has been chosen yet, what would settle it,
  and what a misheard "stop" is required to do.

## What is not here

- Anything describing a physical robot. There isn't one. Every dimension and
  every braking figure in this repository is a placeholder, which the hazard
  analysis records as its own hazard (H-14).
- A full privacy and data-protection analysis for the voice and camera paths.
  [guides/privacy.md](guides/privacy.md) sets the basic rules (consent first, scans never
  in git); the analysis itself is still needed.
- Anything about stair climbing beyond the note that it is a separate machine
  needing its own hazard analysis.

## The rules themselves

The safety rules that every change has to respect are not in this folder —
they are in [`AGENTS.md`](../AGENTS.md) at the top of the repository, so that
anyone (or anything) working on the code reads them before touching it.
