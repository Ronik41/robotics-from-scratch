# ADR 0006: SLAM Toolbox mapping, saved occupancy grid, separate AMCL localization

- Date: 2026-09-25 (America/Toronto; captures may have September 26 UTC timestamps)
- Status: Accepted; regressions, mapping, clean-restart localization and desktop evidence passed
- Scope: Milestone 6 only

## Decision

Use the Jazzy binaries of SLAM Toolbox in asynchronous mapping mode, with Ceres
scan matching/pose-graph optimization and loop closure enabled. Use standalone
Nav2 `map_server`, `amcl` and `lifecycle_manager` for a separate localization
launch. Never launch both estimators together. No Nav2 planner, controller,
costmap, behavior tree, navigator or waypoint follower runs in M6.

Both modes consume the existing 361-ray, 10 Hz `/scan` and the existing wheel-only
`odom -> base_link` TF, with robot-state-publisher sensor extrinsics. Wheel
odometry still integrates measured joint angles as established in M3; M5's
firmware independently consumes quantized/delayed encoder counts. We do not
claim encoder quantization has been added to the M3 estimator. IMU and camera
are not estimator inputs in M6. All measurement/TF time uses the simulator clock.

The map uses 5 cm cells. Its origin is the initial mapping base pose, with axes
aligned with that pose, rather than Gazebo world coordinates. Save an ordinary
PGM/YAML map through Nav2's map saver; commit a versioned artifact with hashes,
provenance and a PNG preview. New mapping runs save new candidates and cannot
overwrite an existing version. Localization never changes the saved map. A pose
graph is not required by AMCL; the sensor/TF bag preserves the mapping inputs.

SLAM Toolbox owns `map -> odom` during mapping; AMCL owns it during localization.
The existing wheel estimator alone owns `odom -> base_link`. No static substitute
or simulator transform is published. AMCL requires an explicit operator initial
pose with nonzero uncertainty after every restart. The demonstration starts near
the known pickup area, deliberately supplies an approximate offset pose, then
uses a short bounded teleoperation tape to collect informative measurements.

AMCL's differential motion model uses `base_drive`, a massless fixed frame at the
wheel axle, 0.14 m ahead of `base_link`. Only M6 includes it. The mapper continues
to use `base_link`, preserving the initial map reference. `/initialpose` and
`/amcl_pose` represent **base_drive in map**, while reported trajectory accuracy
uses **base_link in map** through TF. M5 command semantics are unchanged.
An initial localization run at the offset chassis frame produced large covariance
spikes during turns: its small lateral arc was interpreted as rotate/translate/
rotate motion by the differential model. Correcting the reference point matches
the existing M3 axle geometry; reducing noise or loosening covariance thresholds
would conceal that model mismatch. See the retained resolved evidence.

All motion is fresh `TwistStamped` on `/cmd_vel` through the unchanged M5 firmware
and MotorDriver. Data collection is a finite time-based teleoperation script,
not a planner or waypoint navigator. It consumes only clock and firmware status;
the oracle lives in a separate test process and cannot steer its commands.

## Room geometry and honest sensor limits

The M2 scene has two 0.30 m cutaway walls for viewing, while the north/east walls
are 0.80 m. The unchanged LiDAR scans at about 0.58 m above the floor. In M6 only,
raise south/west walls to 0.80 m, preserving their x/y position, thickness and
length. The launcher records the generated SDF. M1–M5 retain their original scene.
This makes the closed room observable without fabricating LiDAR returns or a map
from world geometry. Crates remain below the scan plane and are absent from the
map. That absence is a limitation of this sensor slice, not evidence of clearance.

## Alternatives and consequences

SLAM Toolbox's serialized-graph localization is also valid. AMCL makes the
mapping/localization distinction explicit, uses an inspectable portable image
map, and is the conventional Nav2 localization interface for later work.
AMCL estimates a distribution of candidate poses (particles) and weights them
against laser/map agreement. A tight distribution can still be wrong in a
symmetric room; acceptance therefore checks both uncertainty and independent
pose error. No claim of global/kidnapped-robot recovery is made.

Asynchronous SLAM bounds processing backlog under software rendering; scans can
be skipped. The map is repeatable within measured quality budgets, not bitwise
deterministic. Slow travel, overlapping views and accurate sensor timestamps are
more useful than hiding a scan-matching failure by inserting perfect world pose.

The Dockerfile now selects `https://repo.ros2.org/ubuntu/main` using the ROS signing
key already in the pinned base image. The original archive hostname returned
403 and an HTTPS certificate mismatch on this network. TLS/signature verification
remain enabled; package versions and image identities accompany acceptance.

## Acceptance

Before implementation, rerun M1–M5. Rebuild and repeat regressions. Start mapping
in a fresh container, inspect `/map`, run the finite survey, save a candidate,
stop all processes, and localize against it in a second fresh container.
Acceptance binds TF message publisher GIDs to ROS node identities, checks every
observed edge's single authority/parent, audits estimator/controller subscriptions
and the bridge allowlist, and verifies sole M5 command consumption/actuation.
A small read-only C++ acceptance probe obtains GIDs from middleware message
metadata because Jazzy's rclpy callback dictionary omits them. It emits a TSV
snapshot and publishes no transforms, poses or commands. Rosbag is allowed as a passive command-topic
observer; it is never counted as an actuator.
The mapper's reserved interactive-marker feedback endpoint remains subscribed
in this Jazzy release even with interactive editing disabled; acceptance requires
zero publishers on that endpoint and audits the internal TF listener nodes too.

Map quality uses known wall/table geometry only inside tests: fixed map/world
alignment (no fitted registration), >=90% known interior (with wall/table buffers), >=90% coverage of each
wall within 15 cm, and <=15 cm occupied-surface 95th-percentile error. Localization
requires <15 cm and <0.12 rad error over the final ten timestamp-matched samples,
with finite positive x/y variances <0.04 m² and yaw variance <0.0225 rad².
Require no map TF before initialization, repeated AMCL updates, immutable saved
files, and physical stopping after the survey's M5 command lease expires.

See [M6 evidence](../../evidence/milestone-6/README.md) for measured results and
[mapping concepts](../concepts/mapping-localization.md) for hardware differences.

## Primary references

- [SLAM Toolbox Jazzy source and guide](https://github.com/SteveMacenski/slam_toolbox/tree/jazzy)
- [Nav2 Jazzy AMCL](https://api.nav2.org/nav2-jazzy/html/md_nav2_amcl_README.html)
- [Nav2 map server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/map_server/configuring_map_server/)
- [REP 105 frame conventions](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
- [Jazzy differential motion model source](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/motion_model/differential_motion_model.cpp)
