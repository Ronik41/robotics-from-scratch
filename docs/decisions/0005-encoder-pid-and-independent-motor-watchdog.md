# ADR 0005: Encoder PID and an independent simulated motor watchdog

- Date: 2026-09-25
- Status: Accepted; deterministic, headless, regression and desktop checks passed
- Scope: Milestone 5 only

## Decision

Preserve the M1–M4 launches and add an explicit M5 firmware mode. Remove DiffDrive
from the generated M5 model and remove the direct `/cmd_vel` command bridge.
Use stamped body-velocity commands, a standalone Python firmware process, the
existing M4 encoder message, and a small compiled Gazebo torque-driver plugin.
The plugin is compiled inside both Docker images; no host dependency is installed.

Keep the PID/safety state machine independent of ROS for deterministic tests.
Compute speed from count differences and acquisition timestamps; use no privileged
pose or ideal joint-velocity feedback in the firmware. Use bounded wheel targets,
acceleration ramps, filtered derivative on measurement, conditional integration,
bounded torque and explicit motor deadband. Full contracts and numeric settings
are in [firmware safety](../concepts/firmware-safety.md).

Use both simulation acquisition-age and steady-clock receive deadlines. An
independent motor-driver lease handles stopped/killed/frozen firmware; braking
occurs in physics using finite, bounded damping torque. Latch e-stop in firmware
and driver; resetting requires release, fresh zero intent and stationary feedback,
and does not enable motion. State is independently observable from both layers.

M5 uses TwistStamped on `/cmd_vel`; the earlier baseline modes keep Twist. No
adapter restamps unstamped packets, since that would conceal their unknown age.
The keyboard tool supports stamped output and does not repeat commands in the
background. A single movement key expires; sustained user key events are needed
for sustained motion. Future high-level software must publish fresh stamped
requests within the lease.

## Alternatives and consequences

A Twist watchdog wrapped around DiffDrive would teach command expiry but retain
an ideal velocity actuator, without a wheel-effort feedback loop. A lone ROS
controller would leave a last torque active after that controller froze. An
instantaneous joint-velocity reset would obscure physical braking distance.

ros2_control is a valid future hardware integration path. For this milestone, a
small pure control core exposes the equations and safety state transitions with
less infrastructure, while a narrow Gazebo plugin supplies the independently
scheduled actuator boundary. C++ is confined to that required Gazebo component.
There are now two places enforcing the 2 N m torque bound; tests and documentation
must remain consistent when changing the motor protocol or limits.

The brake model assumes available normally-on braking electronics. This does not
establish MCU scheduling, physical e-stop circuitry, current/thermal protection,
security, power-loss behavior or real-world safety. Driver state survives a
firmware process restart but not a full simulator restart. M3 odometry and sensor
contracts remain unchanged; world pose remains exclusively a test oracle.

## Verification

Rerun documented M1–M4 checks before implementation and on the rebuilt image.
Exercise a deterministic quantized plant, locked wheels, bounds, timeout, latch,
reset and invalid/stale inputs in unit tests. Live acceptance measures wheel-speed
tracking and physical motion, rejects malformed requests, measures stops, suspends
sensor and firmware processes, and checks driver deadband/rejection independently.
The generated model must contain one MotorDriver and no DiffDrive; firmware ROS
inputs must be exactly clock, stamped command and encoders. Record browser desktop
state and curated logs/results in [M5 evidence](../../evidence/milestone-5/README.md).
No mapping, localization, Nav2 or perception is included.
