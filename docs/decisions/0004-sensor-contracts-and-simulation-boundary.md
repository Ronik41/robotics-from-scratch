# ADR 0004: Explicit sensor contracts and imperfect simulated measurements

- Date: 2026-09-25
- Status: Implementation under acceptance
- Scope: Milestone 4 only

## Decision

Keep M2/M3 launch modes intact. M4 opts into fixed sensor frames, native Gazebo
GPU LiDAR/camera/IMU systems and a dedicated sensor bridge allowlist. Generate an
M4 world from the unchanged room plus sensor systems; retain it in run evidence.
Use Ogre 2 software rendering and EGL headless rendering for automated checks.

Use standard `LaserScan`, `Image` plus synchronized `CameraInfo`, and raw `Imu`
contracts. A small ROS message package defines stamped signed wheel counts with
resolution, joint names, sample sequence and acquisition interval. Compile it
inside the Docker image using colcon; source its overlay in the common entrypoint.
Python runtime code and launch assets remain directly mounted from the repository.
Changes to the message definition require rebuilding both images.

The simulation driver consumes Gazebo IMU angular velocity/specific force and ROS
joint angles. It deliberately discards native IMU orientation before ROS
publication, applies seeded noise/fixed biases, and models 10 ms delivery delay
for IMU and encoders without altering acquisition stamps. Encoder quantization
is deterministic nearest-count rounding; do not add implausible random digital
counts just to make the signal look noisy. Native camera and LiDAR use their
Gazebo noise models and expose only their measurement interfaces.

Keep the established `wheel_odometry` implementation and inputs unchanged. It
remains sole owner of `/odom` and `odom -> base_link`; robot_state_publisher owns
sensor and mechanical extrinsics. No localization/fusion or map frame is added.
The raw simulator joint states remain an explicitly ideal M3 feedback boundary,
not a claim that they are a real encoder protocol.

## Alternatives and consequences

Using JointState alone would omit discrete count and startup semantics. Exposing
native Gazebo IMU attitude would silently give future estimation perfect attitude.
A physical IMU orientation filter belongs to later estimation work. A bespoke
ray caster or procedural camera would skip the existing renderer/occlusion model.
Native sensors are the more representative starting point, with explicit limits.

A custom message adds a small image build step and must be present for bag replay.
Massless sensor frames preserve prior dynamics but do not model packaging.
The raised LiDAR misses low crates; this is documented and checked, not hidden by
using a privileged obstacle list. Initial imperfection parameters are teaching
assumptions and will require hardware calibration later.

## Verification

Rerun M1–M3 before implementation and after rebuilding. M4 automated acceptance
checks message contracts, timestamps/rates, TF, camera calibration, known surface
ranges and occlusion, stationary IMU statistics, encoder motion/quantization and
odometry input ownership. Record a short MCAP rosbag, replay in a separate ROS
domain, verify message payload identity and TF. Capture and inspect actual RViz
rendering. Evidence index records results and retained failures.

No mapping, localization, navigation, perception, firmware PID, watchdog or e-stop
implementation is authorized by this decision.
