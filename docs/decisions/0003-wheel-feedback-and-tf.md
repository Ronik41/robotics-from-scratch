# ADR 0003: Wheel-only odometry, explicit TF ownership and ROS desktop

- Date: 2026-09-25
- Status: Accepted; headless and browser-desktop acceptance passed (see Milestone 3 evidence)
- Scope: Milestone 3 only

## Decision

Keep the M2 mechanical model and paused launch as the passive baseline. An opt-in
Xacro argument adds Gazebo DiffDrive and JointStatePublisher systems. Use a native
ROS launch description for the bridge, robot_state_publisher, wheel odometry,
RViz and a keyboard teleop terminal. Retain the existing Docker/noVNC supervisor;
no new host software or exposed service is needed. Assets are interpreted directly
from the mounted workspace; a colcon package/install step is not needed for this
small Python-only milestone.

The bridge allowlist admits only commands (ROS to Gazebo), wheel joint states and
clock (Gazebo to ROS). Wheel odometry consumes `/joint_states`, never `/cmd_vel`
or Gazebo model pose. It owns `/odom` and `odom -> base_link`. Robot state publisher
owns all remaining robot-link transforms. ROS sim time is enabled throughout.
Do not bridge native Gazebo drive odometry/TF or world pose.

Use a short, independently tested Python differential-drive integrator because
the wheel axle lies 0.14 m ahead of the existing chassis-centred `base_link`.
The implementation explicitly transforms the integrated axle pose and velocity
to the chassis centre, with geometry read from the expanded URDF. Relabeling an
axle-centred native odometry output as `base_link` would be incorrect during turns.
Moving the mechanical frame or introducing an inverted extra root solely for a
plugin shortcut would complicate the established model.

`odom` begins at the initial chassis centre, including its nominal height. RViz's
ground grid is offset -0.24 m. No `map` or world transform is fabricated; M6
localization will own `map -> odom`. Covariances are clearly documented nonzero
placeholders, not calibrated uncertainty.

## Verification and consequences

Analytic tests cover straight/reverse displacement, exact arcs, in-place rotation
with axle offset, stale/nonfinite timestamps and stationary wheels. Real ROS /
Gazebo acceptance commands forward, reverse, left, right, arc and zero, checks
actual joint motion, compares wheel-only displacement with test-only Gazebo pose,
looks up all live TF paths, compares odom/TF at identical timestamps and audits
publishers/subscriptions. A desktop run verifies RViz rendering through noVNC.
Evidence includes failed attempts where relevant rather than hiding them.

DiffDrive is an interim ideal actuator, not the eventual low-level controller.
No watchdog, e-stop, firmware PID, sensor noise, mapping or autonomy is claimed.
A full launcher restart resets both simulation and integration state. In-place
world/time resets are unsupported. See the concept note for the complete frame
and hardware-migration contract.
