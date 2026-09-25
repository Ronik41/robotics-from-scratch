# Project Status

## Current milestone

**Milestone 4 — Implementation and acceptance in progress (2026-09-25).**

M1–M3 baseline passed before M4 changes: M1 `20260925T193238Z-62607`,
M2 `20260925T193244Z-acceptance-62657`, M3 `20260925T193319Z-acceptance-62606`.
M4 adds opt-in native sensors, an explicit count message, and a separate imperfect
sensor driver. Headless sensors publish; acceptance exposed an overstrict camera
floating-point comparison, now under retest. No later milestone is implemented.
The sections below retain the prior M3 completion record pending M4 final results.

Milestones 1 and 2 were rerun successfully before implementation and again after
rebuilding the images. Only Milestone 3 was implemented in this task.

## Current state

The differential-drive rover accepts ROS 2 `geometry_msgs/Twist` commands on
`/cmd_vel`. Gazebo's drive plugin moves the wheel joints; actual joint angles are
bridged to `/joint_states`. A small wheel-only estimator integrates those angles
into `/odom` and `odom -> base_link`, accounting for the axle 0.14 m ahead of the
chassis centre. `robot_state_publisher` supplies wheel and fixed-link transforms.
No Gazebo pose is bridged into ROS or consumed by the estimator.

The live tree is `odom -> base_link -> left_wheel/right_wheel/rear_support/parcel_tray`.
`map -> odom` remains absent until localization. RViz shows the robot model,
coordinate frames/tree and wheel-odometry arrows in the existing browser desktop.
Keyboard teleoperation is available in its xterm window. All nodes use simulation
time; communication stays inside one container.

The desktop is running with the rover stopped after verification. Open
[the local desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
The final scene includes the keyboard/acceptance route. Repeat the launcher below
for a fresh initial scene. Stop with `docker compose stop desktop`.

## Acceptance results

| Check | Result | Evidence |
| --- | --- | --- |
| M1 before implementation | PASS: 101 clock messages | `evidence/milestone-1/20260925T185759Z-59476/result.json` |
| M2 before implementation | PASS: three fresh starts, identical initial/settled poses, duplicate guard | `evidence/milestone-2/20260925T185803Z-acceptance-59475/` |
| Rebuilt container images | PASS; dependencies stay inside Linux | `evidence/milestone-3/build.log` |
| M1/M2 regression after rebuild | PASS | `evidence/milestone-1/20260925T190822Z-60584/`, `evidence/milestone-2/20260925T190828Z-acceptance-60583/` |
| Wheel-integration maths | PASS: five independent analytic cases | Final headless `unit.log` |
| Headless motion, wheel feedback, odometry and TF | PASS: forward/reverse/left/right/arc/zero, 601 odom messages at 50 Hz, complete tree, matched timestamps and wheel rotations, single authorities | `evidence/milestone-3/20260925T191023Z-acceptance-60960/` |
| Browser keyboard to actual ROS motion | PASS: `i` -> 0.2 m/s, `k` -> zero; 0.413 m travel, final zero speed | Final desktop `keyboard.json` |
| Motion acceptance with GUI and RViz running | PASS | Final desktop `desktop-check/result.json` |
| RViz visual evidence | PASS: robot, labels/axes, transform tree and odom trail; global/TF OK | Final desktop `rviz-browser.png` |
| Documentation, frame contract, real-hardware boundary | Complete | Setup guide, concept note, ADR 0003 |

Final desktop evidence directory:
`evidence/milestone-3/20260925T190959Z-launch-60871/`.
[Full evidence index](evidence/milestone-3/README.md) includes logs, hashes, image
identity, measurements and retained failures.

## Findings and limitations

- Wheel odometry is not ground truth. The final headless route differed from
  Gazebo pose by up to **2.62 cm and 0.1754 rad (10.1 degrees)**; desktop results
  were similar. The nominal test budgets are 10 cm / 0.20 rad. An initial 0.08 rad
  heading budget failed; the rolling-model/physics mismatch remains reported,
  not corrected with privileged pose. Finite-width contact/slip is a likely cause.
- A test startup race queried the tray before static TF arrived. Acceptance now
  waits for every required transform; subsequent headless and desktop runs passed.
- Covariance values are nonzero placeholders, not calibrated sensor uncertainty.
  The estimate is planar and ignores chassis bounce, pitch and roll. The odom
  origin is at nominal chassis height; RViz's floor grid is 0.24 m below it.
- Gazebo DiffDrive is ideal actuation. Explicit zero Twist stops motion, but there
  is **no command watchdog, e-stop, motor PID or low-level safety implementation**.
  Key release or browser disconnection is not a stop. Those features belong to M5.
- Reset by restarting the complete launcher. In-place Gazebo time resets are
  unsupported because the wheel estimator rejects backwards timestamps.
- KDL root-inertia and software-rendering warnings are documented. They did not
  prevent correct TF, physics checks or the inspected rendering.
- The supported environment remains Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Gazebo
  Harmonic on the existing Docker Desktop. Image IDs and package manifests are
  recorded; apt versions are not fully pinned. Browser port stays on localhost.

## Repeat Milestone 3

```bash
docker compose --profile gui build robotics desktop
./scripts/smoke-test.sh
./scripts/test-milestone-2.sh
./scripts/test-milestone-3.sh
./scripts/launch-milestone-3.sh gui
```

In the browser, click inside the teleop terminal: `i` forward, `,` reverse,
`j/l` turn, `k` stop. See [setup and inspection commands](docs/setup.md),
[ROS/TF/odometry concepts](docs/concepts/teleop-tf-odometry.md), and
[ADR 0003](docs/decisions/0003-wheel-feedback-and-tf.md).

## Scope boundary

Milestone 4 sensors and Milestone 5 firmware safety have not begun. There is no
LiDAR, camera, IMU, encoder noise/quantization model, mapping, localization,
navigation or perception. Gazebo pose is a test oracle only. No new macOS software,
external publication, Git commit or push was performed. Existing unrelated
containers and the Mininet VM were not modified.

Prior milestone records remain in [M1 evidence](evidence/milestone-1/README.md),
[M2 evidence](evidence/milestone-2/README.md), and ADRs 0001–0002.
