# Build Plan

Each milestone must have evidence of completion before the next one begins.

## 1. Reproducible development environment

Choose a macOS-compatible ROS 2 and Gazebo workflow; document prerequisites, startup commands, and a smoke test.

**Done when:** a clean setup can launch a known simulator example and the instructions record the exact path chosen.

## 2. Rover and world

Create a differential-drive rover described in URDF/Xacro and spawn it in a small delivery environment.

**Done when:** Gazebo loads the robot and world deterministically.

## 3. Teleoperation, transforms, odometry, RViz

Drive the rover manually and inspect its TF tree, wheel odometry, and pose visualization.

**Done when:** commands move the rover; RViz shows a coherent transform tree and odometry.

## 4. Simulated sensors

Add LiDAR, RGB camera, IMU, and encoder data with explicit frame IDs and realistic noise assumptions.

**Done when:** each stream is visible and documented in RViz or suitable ROS tools.

## 5. Simulated firmware / low-level control

Implement a separate motor-control boundary: velocity PID, encoder feedback, saturation, watchdog, emergency stop, and diagnostic state.

**Done when:** command timeout and e-stop safely halt the rover in simulation.

## 6. Mapping and localization

Build a map, record data, and localize the rover without using simulator ground truth for autonomy.

**Done when:** a repeatable mapping/localization demonstration is recorded.

## 7. Autonomous navigation

Configure Nav2 to reach delivery waypoints, avoid obstacles, and expose recovery behavior.

**Done when:** the rover completes a nominal route and safely reports/recovers from a blocked path.

## 8. Conventional perception

Detect a delivery marker or package with a non-ML baseline, integrate the result into a mission behavior.

**Done when:** detection is demonstrated under specified conditions and failure cases are documented.

## 9. Robot ML extension

Integrate or train a lightweight learned detector or policy. Measure it against the conventional baseline.

**Done when:** evaluation data, metrics, limitations, and comparison are recorded.

## 10. Fault injection and final demo

Exercise noise, message loss, wheel slip, blocked routes, and e-stop; package automated scenarios and real-hardware migration notes.

**Done when:** all scenarios have outcomes, evidence, and a final end-to-end delivery demo.
