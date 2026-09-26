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

## Implemented in Milestone 5

M5 is selected with `--firmware --sensors --drive` (or its dedicated launcher).
Earlier launch modes retain their original interfaces for regression testing.

```text
/cmd_vel (TwistStamped, base_link, acquisition stamp)
    -> sim_firmware [validation, wheel ramp, PID, watchdog, e-stop]
       ^ /wheel/encoders (M4 quantized/delayed counts)
       |                                    |
       |                 native /motor/effort [timestamp, L Nm, R Nm, mode]
       |                                    v
       |                    rover::MotorDriver [independent lease + latch]
       |                                    |
       |                              torque + deadband / brake
       |                                    v
       +----- sim_sensor_driver <------ Gazebo joint physics

/safety/estop + /safety/reset -> sim_firmware -> motor latch/reset protocol
/firmware/state <- sim_firmware       /motor/driver_state <- MotorDriver
```

`bridge-firmware.yaml` contains no body-velocity command bridge. The M5 model
contains no DiffDrive. The motor plugin is the sole wheel-effort writer and reads
joint speed only to implement its physical damping-brake model. Firmware has no
Gazebo state subscription and derives velocity exclusively from encoder counts.
Native driver telemetry is bridged for inspection, not controller feedback.
Firmware is a separate Linux process modeling the MCU role; driver watchdog checks
run inside Gazebo even when that process is suspended. All runtime components
still use simulation time for data acquisition and physics progression, with
additional monotonic receive deadlines for stalls.

Wheel odometry, TF and sensor ownership remain as in M3/M4. No map, localization,
planner or perception node is added. Simulator pose remains available only to
acceptance processes. See [controller contracts and hardware gaps](concepts/firmware-safety.md)
and [ADR 0005](decisions/0005-encoder-pid-and-independent-motor-watchdog.md).

## Milestone 6: map and localization boundary

```text
M4 /scan + wheel-only odom -> base_link + sensor extrinsics
       |                                |
       +-------- SLAM Toolbox ----------+  mapping mode
                    |          |
                   /map       map -> odom
                    |
              Nav2 map saver -> versioned PGM + YAML
                                      |
                               Nav2 map_server -> /map
                                      |            |
operator approximate /initialpose ----> AMCL <------+  localization mode
                                        |
                                   map -> odom + /amcl_pose

finite teleoperation tape OR keyboard -> /cmd_vel (TwistStamped)
      -> unchanged M5 firmware -> MotorDriver -> wheel physics
```

The two M6 estimator modes are mutually exclusive launch choices. The lifecycle
manager configures/activates only the selected mapper, or map server and AMCL.
`wheel_odometry` and `robot_state_publisher` retain their existing TF edges.
M6 adds a fixed `base_link -> base_drive` edge at the wheel axle (x=0.14 m).
AMCL uses `base_drive` for its differential motion model; its initial/reported
pose is the axle pose. SLAM Toolbox still uses `base_link`. No dynamic edge is
replaced, and the initializer consumes no TF or pose feedback.
No world/map transform or simulator pose bridge exists. `map -> odom` is dynamic
and belongs only to SLAM Toolbox or AMCL. AMCL does not publish it until initialized.

The M6 world variant raises south/west cutaway walls to the same 0.8 m height as
the other walls; its footprint, rover, sensors, crates and table are unchanged.
The initial rover pose defines the map reference. Saved-map metadata and test
oracle alignment are documented separately; world coordinates never configure
the estimator. The raw joint-state wheel estimator remains the M3 implementation,
while M5 control feedback remains quantized/delayed encoders.

`mapping_survey.py` consumes only clock and firmware diagnostics. It publishes
bounded, stamped teleoperation commands, aborts on stale status/clock or a safety
fault, and never resets a latch. `initialize_localization.py` publishes an operator
pose with nonzero covariance. Neither imports Gazebo or reads odometry/map pose.
Acceptance runs in a separate process, uses Gazebo pose only to score estimates,
and records message-level TF publisher identities and exact input subscriptions.

No planner, costmap, waypoint navigator, behavior tree, perception or ML component
is launched. A map cell is not yet a clearance or navigation decision. See
[ADR 0006](decisions/0006-slam-toolbox-and-amcl.md) and
[mapping/localization concepts](concepts/mapping-localization.md).
