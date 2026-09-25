# Robotics From Scratch

An industry-style autonomous delivery rover, built entirely in simulation with Codex as an implementation partner and robotics tutor.

The project begins with environment setup and grows through a simulated rover, sensors, low-level safety control, mapping, navigation, perception, robot ML, and fault injection.

Start with [the project specification](PROJECT_SPEC.md), [milestone plan](PLAN.md), and [current status](STATUS.md).

Milestone 1 uses **Ubuntu 24.04 ARM64 + ROS 2 Jazzy + Gazebo Harmonic** inside the
Mac's existing Docker Desktop. Start with the [setup guide](docs/setup.md),
[environment decision](docs/decisions/0001-ros-gazebo-environment.md), and
[concept explanation](docs/concepts/environment-and-simulation.md).

From this repository, with Docker Desktop running:

```bash
docker compose build
./scripts/smoke-test.sh
```

The smoke test launches Gazebo's supplied shapes world without a window and verifies
its advancing simulation clock reaches ROS. Timestamped evidence is saved under
`evidence/milestone-1/`.

Milestone 2 adds a Xacro differential-drive chassis and a small delivery room,
with a browser-visible Gazebo desktop inside Docker:

```bash
docker compose --profile gui build desktop
./scripts/test-milestone-2.sh
./scripts/launch-rover.sh gui
```

Open [Gazebo locally](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Repeat the launch command to reset; stop with `docker compose stop desktop`.
See [rover concepts](docs/concepts/rover-and-world.md) and
[evidence](evidence/milestone-2/README.md). The passive Milestone 2 launch remains available.

Milestone 3 is complete: keyboard `Twist` teleoperation, wheel-feedback odometry,
a live TF2 tree and RViz in the same local browser desktop:

```bash
docker compose --profile gui build robotics desktop
./scripts/test-milestone-3.sh
./scripts/launch-milestone-3.sh gui
```

Click the teleop terminal: `i` forward, `,` reverse, `j/l` turn, `k` stop.
[Setup and checks](docs/setup.md) · [Frame/odometry concepts](docs/concepts/teleop-tf-odometry.md)
· [Verified evidence](evidence/milestone-3/README.md). Wheel odometry drifts;
this historical M3 mode has no watchdog/e-stop.


Milestone 4 adds native simulated LiDAR, RGB camera, raw IMU and explicit wheel
encoder counts, with frame/rate/noise contracts and rosbag recording/replay:

```bash
docker compose --profile gui build robotics desktop
./scripts/test-milestone-4.sh
./scripts/launch-milestone-4.sh gui
```

[Sensor concepts](docs/concepts/sensors.md) · [M4 evidence](evidence/milestone-4/README.md).
This historical M4 mode retains ideal actuation; use M5 for low-level safety.


Milestone 5 adds encoder-feedback wheel PID, bounded torque and acceleration
requests, a command/feedback watchdog, a latched e-stop with explicit reset, and
an independent simulated motor-driver watchdog:

```bash
docker compose --profile gui build robotics desktop
./scripts/test-milestone-5.sh
./scripts/launch-milestone-5.sh gui
```

M5 requires stamped commands; a keyboard request expires after 0.5 simulation
seconds unless refreshed. The desktop includes a live safety-state monitor.
[Control and safety concepts](docs/concepts/firmware-safety.md) ·
[Setup and reset commands](docs/setup.md#milestone-5-simulated-firmware-and-motor-safety) ·
[M5 evidence](evidence/milestone-5/README.md).
No mapping, localization, Nav2 or perception has been implemented.
