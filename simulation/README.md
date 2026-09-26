# Simulation

- `rover/rover.urdf.xacro`: chassis, two independent wheel joints, rear support and parcel tray.
- `worlds/delivery_room.sdf`: local 8 by 6 metre room, pads and obstacles.
- `config/scenario.json`: canonical name, seed, initial pose and expected scene membership.
- `config/gui.config`: reproducible camera and GUI layout.

Launch and verification commands are in [the setup guide](../docs/setup.md).
Generated URDF/SDF and observations are stored with each run's evidence, not edited here.

M4 opts into `rover/sensors.xacro`, `config/bridge-sensors.yaml`,
`config/sensors.yaml` and `config/sensors.rviz`. The supervisor generates a copy
of the room with Gazebo sensor systems in each evidence directory; M2/M3 still
use the original world. Native LiDAR/camera require rendering, including headless
Ogre 2/EGL. See the sensor concept note for mounts and measurement limitations.


M5 opts into the firmware argument, `config/bridge-firmware.yaml`, and the compiled
`rover::MotorDriver` plugin. Its model omits DiffDrive and its bridge omits body
commands. `scripts/launch-milestone-5.sh` launches this mode with sensors, RViz,
stamped keyboard teleop and a read-only safety monitor. Earlier modes remain
regression baselines. See the firmware safety concept note for protocol and limits.

M6 adds `--m6 mapping|localization` on top of M5, with mutually exclusive
`config/mapping.yaml` (SLAM Toolbox) and `config/localization.yaml` (AMCL).
Each has an RViz preset with fixed frame `map`. The generated M6 world raises
only south/west cutaway walls to 0.8 m; their footprint and the original world
file are unchanged. Saved maps live under `maps/`, outside simulator inputs.
See ADR 0006 for TF ownership and the separation between mapping and localization.
