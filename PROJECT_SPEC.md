# Robotics From Scratch: Simulated Autonomous Delivery Rover

## Goal

Build an industry-style autonomous delivery rover entirely in simulation while learning the why behind each layer of a modern robotics system.

The rover will complete delivery missions in a small simulated environment: receive a goal, perceive landmarks or packages, navigate safely around obstacles, and report success or a safe failure.

## Constraints

- No physical robot hardware.
- Development happens on macOS; the toolchain must be reproducible and documented.
- Prefer free, widely used robotics tools.
- Build a classical, explainable baseline before adding machine learning.
- Codex handles most implementation and verification. Explanations must remain concise, concrete, and technically honest.

## Target stack

- ROS 2: distributed robot software, messages, launch, parameters, TF2, rosbag.
- Gazebo: physics simulation, world, differential-drive rover, simulated sensors.
- RViz: visualization and debugging.
- Nav2: localization, planning, navigation, and recovery behaviors.
- URDF/Xacro: robot description.
- Python first; use C++ only where it materially teaches a common robotics practice.
- Git plus project documentation for repeatability and build-in-public evidence.

The exact ROS 2/Gazebo distribution and environment strategy (native Linux VM, container, or another supported option) are a Milestone 1 decision, based on practical compatibility with this Mac.

## System boundary

```text
Simulated camera / LiDAR / IMU / encoders
                  |
                  v
             ROS 2 topics
                  |
                  v
Perception -> localization/map -> planner/Nav2
                  |
                  v
         desired linear/angular velocity
                  |
                  v
 simulated firmware: PID, limits, watchdog, e-stop
                  |
                  v
             Gazebo physics
```

## Definition of done

1. The repository has a documented, repeatable setup and a reproducible simulated demo.
2. The rover can be manually driven; transforms, odometry, and sensor streams are visible in RViz.
3. It safely executes waypoint or delivery goals using a map/localization and Nav2.
4. It detects delivery-relevant visual markers with a conventional perception baseline.
5. A clearly measured robot-ML extension is compared against that baseline.
6. The low-level simulated controller implements encoder feedback, velocity control, limits, watchdog behavior, and emergency stop behavior.
7. Fault scenarios demonstrate safe, observable behavior.
8. Documentation clearly states what would change on a real robot.

## Out of scope for the first version

- Buying or controlling physical hardware.
- Claiming real-world reliability based on simulation alone.
- Training a large foundation model from scratch.
- A production-grade fleet-management service.
