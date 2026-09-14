# Safety Controller Test Procedure

## Software test

Run from the repository root:

```bash
PYTHONPATH=robot/ros2_ws/src/robot_safety python3 -m unittest discover -s robot/ros2_ws/src/robot_safety/tests -v
```

The test suite must verify that:

- A clear path permits full commanded speed.
- An obstacle inside the caution distance reduces speed.
- An obstacle inside the stop distance commands zero speed.
- Emergency stop commands zero speed even with a clear sensor reading.
- Missing, infinite, or NaN sensor data commands zero speed.
- Invalid stop and caution thresholds are rejected.

## Hardware test procedure

Do not substitute this software test for a physical emergency-stop test.

1. Raise the robot so the wheels cannot contact people or furniture.
2. Confirm the normally-closed emergency-stop circuit removes motor power.
3. Release the emergency stop and verify the robot remains stopped until a deliberate enable action.
4. At the lowest available speed, place a non-damaging obstacle in front of the robot.
5. Verify the robot slows inside the caution distance and stops inside the stop distance.
6. Disconnect or block the obstacle sensor and verify the robot stops on invalid or stale data.
7. Repeat on the floor inside a marked exclusion zone with a tether and a second person operating the stop.

Record sensor type, threshold values, robot speed, payload, test result, and any unexpected behavior in the issue or pull request for the change.
