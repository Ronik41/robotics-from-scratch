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
