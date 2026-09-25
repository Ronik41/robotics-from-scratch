# Sensor measurements, not omniscience

A sensor reports a partial, imperfect observation at a particular time and place.
A frame is a coordinate system attached to that place. ROS TF describes how
frames relate; it does not turn a poor measurement into an accurate one.

| Sensor | Observes | Cannot directly observe | ROS frame |
| --- | --- | --- | --- |
| 2D LiDAR | Distance to the first visible surface along each horizontal ray | Hidden surfaces, colour, objects above/below its plane, absolute pose | `lidar_link`, x forward, y left, z up |
| RGB camera | Colour/intensity along image rays | Metric depth from a single image, objects outside its view or behind surfaces | `camera_optical_frame`, z forward, x right, y down |
| IMU | Angular velocity and specific force in three axes | Position, velocity or absolute heading without estimation/additional references | `imu_link`, x forward, y left, z up |
| Wheel encoders | Discrete shaft rotation | Body displacement during slip, world pose, external obstacles | `base_link` header plus explicit left/right joint names and +y wheel axes |

An upright stationary accelerometer reads approximately +9.81 m/s² in z: the
support force against gravity. It is not a gravity-subtracted body acceleration.
A raw IMU does not inherently supply a trustworthy orientation estimate. This
interface marks orientation unavailable (`orientation_covariance[0] = -1`), and
never exports Gazebo's exact attitude into ROS.

## Sampling and imperfections

- **Rate** is measurements per second. Here rates and delays use simulation time;
  software rendering can make one simulation second take longer than one wall second.
- **Latency** is delivery time minus acquisition time. A late message must retain
  its original measurement stamp. Replacing it with the arrival time breaks
  time alignment between sensors and transforms.
- **Noise** varies from sample to sample. **Bias** is a persistent offset; averaging
  random noise does not remove bias. Our fixed biases do not model temperature or
  random walk. IMU covariance describes white noise only, not bias uncertainty.
- **Quantization** represents a continuous value with finite steps. Encoder counts
  and 8-bit camera channels have explicit quantization. LiDAR has a configured
  1 cm range resolution, but Gazebo's floating-point noisy returns are not claimed
  to be rounded to a 1 cm grid; angular samples are separated by one degree.
- **Field of view** is the angular region visible to a sensor. **Occlusion** means
  a nearer opaque surface hides a farther one. A 360-degree planar LiDAR still
  cannot see a low crate below its scan plane.

## Implemented contracts

| ROS topic / type | Rate | Frame / mount relative to chassis | Initial model |
| --- | --- | --- | --- |
| `/scan` / `sensor_msgs/LaserScan` | 10 Hz | `lidar_link`, (0.20, 0, 0.34) m | 361 rays, -π to π including duplicate endpoint; 0.12–8 m; independent Gaussian range σ=0.01 m |
| `/camera/image_raw` / `sensor_msgs/Image` | 10 Hz | optical frame at `camera_link` (0.33, 0, 0.16) m | 320×240 RGB8, 60° horizontal FOV, pinhole, near/far 0.05/12 m; Gaussian pixel σ=0.007 on normalized [0,1], clipping/8-bit quantization |
| `/camera/camera_info` / `sensor_msgs/CameraInfo` | 10 Hz | same optical frame and acquisition stamp as image | Generated pinhole calibration; zero lens distortion; square pixels, fx=fy≈277.128 px |
| `/imu/data_raw` / `sensor_msgs/Imu` | 100 Hz | `imu_link`, chassis origin | Gyro σ=0.002 rad/s; accel σ=0.02 m/s²; bias gyro=(.001,-.001,.003), accel=(.02,-.01,.03); 10 ms minimum modeled delivery delay |
| `/wheel/encoders` / `rover_interfaces/WheelEncoders` | 50 Hz | `base_link`; names `left_wheel_joint`, `right_wheel_joint` | 2048 decoded counts/wheel revolution; nearest-count rounding; 10 ms minimum modeled delay; no fabricated random pulse jitter |

Frames exist only when M4 is enabled. Sensor links are massless kinematic frames
merged into the chassis by URDF-to-SDF conversion. They add no collision or mass;
the M2/M3 mechanical dynamics remain unchanged. This omits physical sensor packaging.
The lidar is nominally 0.58 m above the floor, higher than either crate. The RGB
camera can see those crates within its view; the lidar cannot detect them in this
configuration. This is an intentional, tested illustration of limited observability.

Native LiDAR/camera bridges add no artificial delay. Their real transport/render
latency remains present and is measured in acceptance. Scans are simultaneous
snapshots (no spinning-beam acquisition distortion); the camera has no rolling
shutter, exposure motion blur, auto-exposure or realistic optical artifacts.
No calibrated material response, glass/multipath, IMU temperature drift, encoder
missed pulses, electrical timing or counter wrap is modeled. Gaussian examples
are starting assumptions, not a hardware datasheet or a reliability claim.
Gazebo uses seed 42; the Python IMU noise model uses a separate seed 42. Process
startup timing can change which sample receives which draw. Cross-run/cross-device
pixel identity or trajectories are not guaranteed.

## Why `/joint_states` is not an encoder protocol

`JointState` contains joint names, continuous positions, optional velocities and
efforts. Here it is ideal simulator joint feedback for robot visualization and
the existing wheel-only odometry. The message itself says nothing about pulses,
resolution, gearing, startup zero, counting direction, missed samples or wrap.

The new interface reports signed 64-bit cumulative counts relative to the first
valid acquisition, decoded counts per **wheel** revolution, a monotonic sample
index, and the actual interval since the preceding acquisition. Forward rolling
is positive for both joints. Restart resets both count origin and sequence; the
first interval is zero. In-place simulator time resets are unsupported: restart
the entire launch. Counts are derived from actual wheel angles, never commands.
Nearest-count rounding gives a half-count angular error bound of π/2048 radians.
At 0.14 m wheel radius this corresponds to about 0.215 mm at the rim, not a bound
on body-travel accuracy. Slip still breaks that relationship.

This milestone deliberately leaves M3 `wheel_odometry` consuming its established
joint-state source. The new encoder interface is ready for the future low-level
boundary; replacing estimator inputs requires its own decision and tests. Neither
wheel odometry nor an IMU alone is a localization system. There is no fusion,
`map -> odom`, map, controller PID or safety logic here.

## Inspect, record, replay

RViz uses `odom` as the fixed frame, `LaserScan` for `/scan`, `Image` for the RGB
stream and TF for sensor mounts. Choose Best Effort reliability for sensor topics.
The default RViz plugins do not directly interpret this custom encoder message
or raw six-axis IMU as pose; use `ros2 topic echo` and the acceptance statistics.
Displaying an invented IMU orientation would misrepresent the measurement.

A rosbag saves serialized observations for repeatable debugging. Include `/clock`,
`/tf`, `/tf_static`, `/robot_description`, odometry and the sensor topics so replay
has acquisition time, coordinate context and the robot model. Record with sensor
QoS compatibility and retain static transforms using transient-local durability.
Replay in a fresh ROS domain/container: concurrent live publishers would mix two
worlds. Replay the recorded `/clock`; do not also generate a competing clock.
Use `use_sim_time:=true` in visualization nodes. See the exact commands in
[setup](../setup.md).

## Simulation versus real hardware

On a real rover, replace Gazebo/native adapters with device drivers, establish
hardware clock synchronization, calibrate mounts and camera intrinsics, measure
noise/bias and latency, and implement counter rollover/reconnect semantics.
The camera/LiDAR extrinsics, wheel radius and slip behavior need measurements.
Do not infer deployment accuracy from this nominal room test.

Ground truth is a **test oracle**: a reference used only to judge measurements or
estimates. Gazebo world geometry and pose are unavailable to a real onboard stack.
Feeding them into localization or perception would hide the very errors those
systems must handle. Our tests may inspect known room geometry; runtime ROS
bridges have an explicit allowlist and never bridge world pose/native odometry.

References: [Gazebo Harmonic sensors](https://gazebosim.org/docs/harmonic/sensors/),
[ROS frame conventions REP-103](https://www.ros.org/reps/rep-0103.html).
The installed Jazzy `sensor_msgs` definitions are the message-contract authority.
