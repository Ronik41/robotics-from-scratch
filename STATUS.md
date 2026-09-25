# Project Status

## Current milestone

**Milestone 4 — Complete (2026-09-25). No acceptance checks blocked.**

Milestones 1–3 passed before implementation and again on the rebuilt image.
Only M4 was implemented. [Evidence index](evidence/milestone-4/README.md) links
measurements, logs, rosbag, replay results, RViz screenshot and retained failures.

## Current state

The opt-in M4 stack publishes native Gazebo 2D LiDAR and RGB camera observations,
raw IMU measurements with explicit noise/bias, and a separate stamped encoder
count interface. Every stream has a documented frame and simulation-time rate.

| Interface | Type | Rate | Frame |
| --- | --- | --- | --- |
| `/scan` | sensor_msgs/LaserScan | 10 Hz | lidar_link |
| `/camera/image_raw` | sensor_msgs/Image (320×240 RGB8) | 10 Hz | camera_optical_frame |
| `/camera/camera_info` | sensor_msgs/CameraInfo | 10 Hz | camera_optical_frame |
| `/imu/data_raw` | sensor_msgs/Imu | 100 Hz | imu_link |
| `/wheel/encoders` | rover_interfaces/WheelEncoders | 50 Hz | base_link; explicit wheel joint names |

Encoders use 2048 decoded counts/wheel revolution and signed cumulative counts
since driver startup. IMU and encoder delivery have a minimum modeled 10 ms delay;
headers preserve acquisition stamps. Native IMU orientation is dropped before ROS
publication and explicitly marked unavailable. The new driver publishes no pose,
TF, odometry or commands. Sensor mounts add no physical mass or collision.

M3 `wheel_odometry` source is unchanged and still consumes only `/joint_states`
and clock. It owns `/odom` and `odom -> base_link`; robot_state_publisher owns all
mechanical and sensor transforms. There is no `map -> odom` or localization/fusion.
Simulator pose/native odometry is not bridged to ROS. Ground truth stays a test oracle.

## Acceptance results

- **Baseline M1–M3:** clock smoke, three deterministic loads/duplicate guard,
  five odometry analytic tests, forward/reverse/turn/arc/zero route and TF passed.
- **Rebuilt-image regression:** M1–M3 passed. An M2 stepping timeout during concurrent
  rendering was retained; the isolated rerun passed without changing its thresholds.
- **Fresh M4 test:** `evidence/milestone-4/20260925T194706Z-acceptance-65403/`.
  All stream frame/timestamp/rate checks passed, with camera calibration pairing,
  LiDAR surface ranges/occlusion, stationary IMU statistics, physical turn response,
  signed encoder motion and 514 matched joint/count acquisitions.
- **Measured noise:** LiDAR σ=9.36 mm versus 10 mm configured; IMU gyro σ=.002005
  rad/s versus .002 configured. IMU z acceleration mean=9.8387 m/s² versus 9.84
  expected with bias. These are nominal checks, not calibration.
- **Rosbag:** 5.878 simulation seconds, 14.2 MiB, 8,107 total messages. Isolated
  replay received all recorded sensor samples: 587 IMU, 293 encoders, 59 RGB,
  59 CameraInfo and 58 scans. Decoded fields/image bytes and timestamps matched;
  static/dynamic TF resolved correctly.
- **Desktop:** sensor acceptance passed; actual RViz screenshot shows Global,
  LiDAR and RGB status OK. IMU, encoder and calibration samples are retained.
- **Documentation:** setup, concept notes, architecture and accepted ADR 0004 updated.

## Repeat

```bash
docker compose --profile gui build robotics desktop
./scripts/test-milestone-4.sh
./scripts/launch-milestone-4.sh gui
```

[Open the local desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Final desktop evidence: `evidence/milestone-4/20260925T194850Z-launch-66034/`.
The rover is stopped after verification. RViz displays scans, the RGB view, TF
and wheel odometry. Keyboard teleop remains available: `i` forward, `,` reverse,
`j/l` turn, `k` explicit stop. Stop the desktop with `docker compose stop desktop`.
Headless live launch: `./scripts/launch-milestone-4.sh headless`.

See [setup and bag commands](docs/setup.md), [sensor concepts and contracts](docs/concepts/sensors.md),
[architecture](docs/architecture.md), and [ADR 0004](docs/decisions/0004-sensor-contracts-and-simulation-boundary.md).
M2 and M3 launch/test commands remain available as earlier baselines.

## Findings and boundaries

- The LiDAR plane is about 0.58 m above the floor, so it misses low crates while
  the camera can see them. The table blocks rays to the wall behind it. Both
  properties are checked; no privileged obstacle list fills sensor blind spots.
- Whole-run rates include discovery/startup losses: final headless IMU 94.38 Hz,
  encoder 49.52 Hz, rendered streams about 10.03 Hz. The desktop stationary window
  measured 100/50 Hz. Rendering is CPU based; wall-time throughput differs.
- Native rendering/transport age reached ~206 ms. The 10 ms modeled driver delay
  is not an end-to-end guarantee. Replay uses deeper verifier queues and hashes
  image bytes efficiently; CDR padding is excluded from semantic comparison.
- Noise/bias/covariance values are initial assumptions. No rolling shutter,
  spinning-scan distortion, material multipath, thermal drift, missed pulses or
  hardware counter rollover is modeled. Full launch restart is the reset contract.
- Wheel odometry remains an imperfect planar rolling estimate with the M3 drift
  limitations. Encoder quantization does not measure or correct wheel slip.
- No mapping, localization, navigation, perception, motor PID, watchdog or e-stop
  was implemented. Gazebo DiffDrive remains ideal actuation. Key release/browser
  disconnection does not stop a persistent command; use explicit `k`/zero Twist.
- No host software was installed, no Git commit/push or external publication made.
  Changes remain local. The repository initially contained untracked project files.

Prior records: [M1](evidence/milestone-1/README.md), [M2](evidence/milestone-2/README.md),
[M3](evidence/milestone-3/README.md), ADRs 0001–0003.
