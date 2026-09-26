# Project Status

## Current milestone

**Milestone 6 — Complete, verified 2026-09-26.** Milestone 7 has not started.

SLAM Toolbox builds a 5 cm occupancy grid from the existing LiDAR and wheel-only
odometry. Standalone AMCL and map server localize against the versioned
[delivery room map](maps/delivery_room_v1/README.md) after a clean restart and an
approximate operator pose. Mapping and localization are separate launch modes;
only the active estimator owns `map -> odom`. Gazebo pose remains a test oracle.
All motion still passes through the unchanged M5 firmware and motor safety layer.

## Verification

- M1–M5 passed before implementation and again after the final model/frame change;
  M4 isolated rosbag replay passed in both regression sets.
- Five offline M6 contracts passed. Fresh mapping, fresh headless localization,
  and fresh desktop localization passed map/TF/input-boundary/safety checks.
- Saved grid: 160 × 121 cells, 5 cm resolution; 99.95% known scored interior,
  100% coverage of each room wall within 15 cm, 9.80 cm occupied-surface p95 error.
- Headless localization final-window maximum error: 0.00096 m and 0.03128 rad.
  Independent desktop restart: 0.01015 m and 0.02103 rad. Both satisfy the unchanged
  0.15 m / 0.12 rad limits and x/y/yaw variance limits.
- MCAP bags retain map, scans, wheel odometry, TF, initial pose/localization output
  and M5 command/safety state. Actual RViz screenshot and trajectory plots are
  included in [curated M6 evidence](evidence/milestone-6/README.md).

## Decisions and limitations

[ADR 0006](docs/decisions/0006-slam-toolbox-and-amcl.md) records the Jazzy-compatible
stack, frame ownership, approximate initialization and map versioning. M6 adds a
massless axle-centred `base_drive` frame for AMCL's differential motion model,
resolving covariance spikes caused by using the offset chassis centre. M5 commands
and wheel odometry retain `base_link`; no safety thresholds were weakened.

Only the M6 scene raises the two low cutaway walls so they intersect the unchanged
laser plane. Low crates remain invisible; the east table face is partially mapped.
This is a laser-height slice, not collision clearance. Wheel odometry still uses
ideal-resolution joint angles; M5 control uses quantized/delayed encoder counts.
Nominal simulation accuracy does not establish hardware reliability, global
relocalization or loop-closure benefit. No planning, waypoints, behavior trees,
package perception or ML are included.

The existing Compose desktop is left running for inspection, with the rover
stationary and e-stop asserted in both safety layers. Use the
[setup guide](docs/setup.md#milestone-6-mapping-and-localization) for a fresh mapping
or localization run and the M5 section for reset commands. The dependency image
must be rebuilt for M6; starting an old M5 container alone does not install the
new packages. All installs remain inside Docker.
