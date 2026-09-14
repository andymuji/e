# 0001: ROS development baseline

- Status: Accepted
- Date: 2026-09-14

## Decision

Use Ubuntu 24.04, ROS 2 Jazzy Jalisco, Gazebo Harmonic, Python 3, and the Apache-2.0 license for the first capstone implementation.

Jazzy is an LTS ROS 2 release with support through May 2029. Gazebo Harmonic is the recommended Gazebo release paired with Jazzy. The repository's initial domain logic is already Python, so the first ROS packages use ament_python while preserving the option to add C++ packages for hardware-critical components later.

## Consequences

- Development, CI, simulation, and deployment should target the same Jazzy/Harmonic baseline.
- ROS package dependencies are declared in package.xml and installed with rosdep.
- Every requested motion command must pass through robot_safety before reaching the hardware controller.
- The software emergency stop is a secondary safety layer; the physical robot still requires an independently wired hardware emergency stop.
- A baseline change requires a new decision record and an explicit migration plan.
