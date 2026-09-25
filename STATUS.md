# Project Status

## Current milestone

**Milestone 5 — Complete (2026-09-25). No acceptance checks blocked.**

M1–M4 passed before implementation and again on the rebuilt images, including
M4 sensor recording and isolated replay. Only M5 was implemented. The
[evidence index](evidence/milestone-5/README.md) links deterministic tests, actual
Gazebo motion/fault results, desktop captures, source manifests and resolved issues.

## Current state

M5 introduces a separate simulated firmware process between stamped `/cmd_vel`
requests and wheel actuation. It derives wheel speed from M4's delayed, quantized
encoder counts; applies PID, curvature-preserving speed saturation, acceleration
ramps and torque limits; and sends effort to a compiled Gazebo motor-driver plugin.
The generated M5 model contains no DiffDrive and the bridge has no command bypass.

| Interface | Contract |
| --- | --- |
| `/cmd_vel` | TwistStamped, simulation acquisition time, `base_link`, forward speed/yaw rate |
| `/wheel/encoders` | Existing M4 signed cumulative counts, 2048 counts/revolution, 50 Hz |
| `/safety/estop` | SetBool: true latches; false only releases the input |
| `/safety/reset` | Trigger: requires released input, fresh zero command and stationary fresh feedback |
| `/firmware/state` | Inspectable reason, PID state, requested/measured speed, limits, ages and rejection counts |
| `/motor/driver_state` | Independent drive/brake/watchdog/e-stop status, applied torque and wheel speeds |

Wheel targets are limited to ±4 rad/s and ramped at 4 rad/s². Requested effort is
limited to ±2 N m per wheel. The simulated motor has a 0.04 N m deadband and a
bounded damping brake. Firmware stops on command age >0.5 simulation seconds,
encoder age >0.2 simulation seconds, or receive age >1 monotonic second. The driver
independently brakes if effort is >0.15 simulation seconds old or no valid frame
arrives within 750 ms wall time. Both layers latch e-stop. Reset never replays a
previous motion request.

M3 odometry and TF ownership are unchanged. M4 LiDAR, RGB camera, IMU and encoder
interfaces remain available. Controller inputs are exactly clock, desired velocity
and encoders; no Gazebo pose, odometry, LiDAR or IMU feeds its PID. Gazebo pose is
used only by acceptance code. No mapping, localization, Nav2 or perception is added.

## Acceptance results

- **M1–M4 baseline and rebuilt-image regression:** all passed; repeated M4 bags
  replayed successfully in an isolated ROS domain.
- **Nine deterministic tests:** quantized closed-loop tracking, feedback response,
  explicit zero braking, saturation/acceleration/anti-windup, command and encoder
  watchdogs, latched e-stop/reset and invalid/stale inputs passed.
- **19 fresh headless checks:** actual forward/reverse/turn motion and wheel tracking,
  saturation, timeout/e-stop stops, reset, malformed requests, process suspension,
  motor deadband and driver protocol rejection passed.
- **Tracking:** final-window forward mean 1.4364 rad/s versus 1.4286 requested;
  turn -0.8437/+0.8309 versus -0.8571/+0.8571. All under 0.15 rad/s error tolerance.
- **Timeout:** observed 0.538 simulation seconds after command streaming ended;
  0.39045 m subsequent travel at the maximum requested wheel speed, then only
  0.032 mm movement in the settled half-second window.
- **E-stop:** 0.02813 m travel after assertion from the nominal 0.2 m/s case.
  Continued motion commands did not clear either latch. Reset conditions and
  fresh-command recovery passed.
- **Independent driver:** frozen firmware caused WATCHDOG in 0.194 simulation
  seconds; wheels physically stopped. Suspended encoders also caused a safe stop
  while high-level commands continued, and the recovered stream became usable.
- **Desktop:** motion, fault and e-stop/reset acceptance passed. Actual browser
  captures show timeout/braking and both e-stop latches. CLI stamped commands and
  stop service were exercised. The read-only monitor identifies stale telemetry.

## Repeat

```bash
docker compose --profile gui build robotics desktop
./scripts/test-milestone-5.sh
./scripts/launch-milestone-5.sh gui
```

[Open the local desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
It is currently running with **e-stop asserted and the rover stopped**. Sensor and
diagnostic streams remain available. Follow the [documented reset sequence](docs/setup.md#milestone-5-simulated-firmware-and-motor-safety)
to release/reset it, or relaunch for a fresh scenario. Stop the desktop with
`docker compose stop desktop`. Headless: `./scripts/launch-milestone-5.sh headless`.

M5 keyboard commands are stamped and expire without new key events. `k` brakes.
Earlier M2–M4 launch/test modes remain available as historical baselines; M3/M4
still use unstamped Twist and ideal actuation without M5 safety behavior.

## Boundaries and resolved findings

- Python on Linux models firmware behavior, not MCU real-time execution. The
  driver watchdog runs independently inside physics; real electronics, electrical
  limits and physical e-stop/brake circuitry remain hardware work.
- Torque and braking are finite. Measured stopping distances apply only to these
  nominal simulation cases. Acceleration bounds apply to wheel requests, not all
  possible chassis dynamics. No real-world reliability or certification is claimed.
- Both simulation and monotonic deadlines matter under CPU rendering. GUI timeout
  can occur sooner in simulation time; a CLI check's clock-progress stall correctly
  inhibited motion. Paused physics resumes with expired leases checked.
- Tire scrub needed a larger integral allowance. Final gains track turns within
  the tighter tolerance. The test publisher stopped repeating timestamps, and
  feedback recovery now supports skipped deliveries using cumulative counts.
  [Curated observations](evidence/milestone-5/resolved-observations.json) retain the evidence.
- Full-stack restart is the reset contract for backwards time or encoder driver
  count-origin reset. The driver e-stop survives firmware restart, not a full
  simulator restart. The public ROS/Gazebo graph is not a security boundary.
- No system-wide software was installed and no external publishing beyond the
  authorized repository is part of this milestone.

See [control concepts](docs/concepts/firmware-safety.md), [architecture](docs/architecture.md),
[ADR 0005](docs/decisions/0005-encoder-pid-and-independent-motor-watchdog.md).
Previous evidence: [M1](evidence/milestone-1/README.md), [M2](evidence/milestone-2/README.md),
[M3](evidence/milestone-3/README.md), [M4](evidence/milestone-4/README.md).
