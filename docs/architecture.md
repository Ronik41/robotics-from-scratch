# Architecture

The intended system architecture is defined in `PROJECT_SPEC.md`. This document will expand as interfaces, topic names, frames, and responsibility boundaries become concrete.

## Principle

High-level autonomy decides *where* the rover should go. The simulated firmware/controller decides *how* to safely turn desired motion into wheel motion. Gazebo supplies the physical consequences and synthetic sensor observations.

## Implemented through Milestone 2

Xacro generates URDF; Gazebo's EntityFactory service spawns it into the SDF room.
The launcher waits for service discovery, verifies the actual paused scene and
supervises the server. Acceptance reads Gazebo scene, pose and statistics directly
as test-only ground truth. No autonomy component consumes this privileged data.

The optional desktop service adds Gazebo GUI → Xvfb/Openbox → x11vnc → noVNC →
localhost browser. The desktop supervisor stops child processes on container
shutdown. The headless and desktop environments use separate Gazebo partitions.
There are no Milestone 2 ROS topic bridges or motion controllers. The existing
Milestone 1 smoke test still bridges clock independently.

## Implemented in Milestone 3

An opt-in drive launch adds ROS commands, wheel feedback, odometry and RViz.
`simulation/config/bridge.yaml` is a directional allowlist: `/cmd_vel` goes to
Gazebo; clock and joint states come back. It does not bridge simulator pose or
native drive odometry/TF. `src/rover_odometry/node.py` consumes only joint states
and clock, integrates the wheel angles with the geometry from URDF, and owns
`/odom` plus `odom -> base_link`. `robot_state_publisher` owns the wheel and fixed
link TF edges. Runtime nodes are described by `simulation/launch/teleop.launch.py`;
the existing supervisor owns the simulator and process cleanup.

RViz consumes the robot description, odometry and TF. `teleop_twist_keyboard` in
xterm publishes the same Twist interface a future autonomy stack will use. This
is still ideal simulator actuation; firmware PID/safety remain future work.
See [the full frame/interface contract](concepts/teleop-tf-odometry.md) and
[ADR 0003](decisions/0003-wheel-feedback-and-tf.md).

## Implemented in Milestone 4

```text
Gazebo rendered scene -> GPU LiDAR -> directional bridge -> /scan
                     -> RGB camera -> directional bridge -> /camera/image_raw
                                                         -> /camera/camera_info
Gazebo IMU (gyro, specific force only) -> sim_sensor_driver -> /imu/data_raw
Gazebo joint angles -> /joint_states -> sim_sensor_driver -> /wheel/encoders
                                   -> wheel_odometry -> /odom + odom -> base_link
                                   -> robot_state_publisher -> joint TF + sensor extrinsics
```

The IMU path drops simulator orientation before ROS publication. The sensor driver
adds white noise, fixed bias and delayed delivery to IMU measurements, and
quantization/delayed delivery to encoder counts. It publishes no TF, pose, control
or odometry. The M3 estimator stays independent and unchanged. Future localization
will be a separate component; there is no `map` frame now.

The bridge allowlist for M4 is `simulation/config/bridge-sensors.yaml`. The standard
measurement messages plus `rover_interfaces/WheelEncoders` define the boundary
future software can consume. Sensor configuration lives in `sensors.xacro` and
`sensors.yaml`; frame/rate/noise contracts and hardware gaps are documented in
[the sensor concept note](concepts/sensors.md). The message package is compiled
in the image; launch and runtime Python remain mounted source assets.

A bag records the measurement graph and coordinate/time context. Automated replay
runs without Gazebo or sensor drivers in ROS domain 43. Test processes may use
room geometry and simulator pose as an oracle; autonomy must never subscribe to
those privileged outputs. [ADR 0004](decisions/0004-sensor-contracts-and-simulation-boundary.md)
records the design boundary and alternatives.
