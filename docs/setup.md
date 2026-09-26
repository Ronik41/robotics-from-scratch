# Development environment

Milestones 1–6 use Docker Desktop on Apple Silicon, Ubuntu 24.04 ARM64,
ROS 2 Jazzy, and Gazebo Harmonic. See [the decision](decisions/0001-ros-gazebo-environment.md).

## Prerequisites

- Apple Silicon macOS with Docker Desktop and its Compose plugin already installed.
- Start Docker Desktop and wait for `docker info` to succeed.
- Internet access to Docker Hub, Ubuntu package archives, and repo.ros2.org during build.
- This host has 16 GiB RAM and 156 GiB free; Docker currently has about 8 GiB RAM.
  Allow several GiB of disk for the image and build cache. No host ROS installation is needed.
- On a different Mac where Docker is missing, installing it is a separate host setup step.

## First build and verification

Run from this repository (the path contains a space, so quote it):

```bash
cd "/Users/ronikatch/Documents/ChatGPT/hardwareless robotics"
open -a Docker
docker info
docker compose config --quiet
docker compose build
./scripts/smoke-test.sh
```

On another checkout, substitute its path. The build installs dependencies inside a
Linux image, not macOS. A fresh checkout needs no locally built ROS workspace or external
world downloads. The smoke command creates a fresh disposable container and writes
timestamped evidence under `evidence/milestone-1/`. Exit zero and `"status": "PASS"`
mean that the packaged shapes world ran and its clock reached a ROS subscriber.

The smoke test has a 45-second message deadline and terminates its child processes
on success or failure. `gazebo.log`, `bridge.log`, `result.log`, `world.json`,
`image.txt`, and `installed-packages.tsv` identify what actually ran. On failure,
read both process logs and the exception in `result.log` before retrying.

## Daily commands

```bash
# Launch the known example's physics server; Ctrl-C stops it.
docker compose run --rm robotics gz sim -s -r -v 3 shapes.sdf

# Open a Linux shell with ROS sourced by the image entrypoint.
docker compose run --rm robotics bash

# Repeat the automated end-to-end check.
./scripts/smoke-test.sh
```

`-s` starts only the simulation server; `-r` starts time running rather than paused.
`shapes.sdf` is an example shipped with Gazebo. SDF is an XML format describing
worlds, models, and simulator settings. It is not our future rover model.

All communicating processes run inside one container. `ROS_DOMAIN_ID=42` selects a
ROS discovery group, localhost discovery limits its scope, and `GZ_PARTITION`
separates Gazebo transport discovery. These are discovery settings, not security
boundaries. Separate `compose run` containers do not share localhost; launch all
interacting processes in the same container, as the smoke script does.

The repository is mounted at `/workspace` for editing from macOS. ROS packages
are under `/opt/ros/jazzy`. No privileged container or host device access is needed.
Containers run as the base image's default user (root); on native Linux, bind-mounted
outputs may need a user mapping. Docker Desktop handles the Mac bind mount.

## Rebuilding and reproducibility

The ROS base image is pinned by digest and Compose requests native `linux/arm64`.
The Dockerfile intentionally uses the matching Gazebo packages from the ROS archive.
The installed package manifest and image ID accompany each smoke run. Apt repositories
can change: a future rebuild is a repeatable recipe, **not a bit-for-bit package lock**.
An existing local image can be rerun without rebuilding. For exact binary archival,
save that image with `docker image save` and record its checksum; no image was published.

To check a completely fresh dependency install deliberately:

```bash
docker compose build --no-cache
./scripts/smoke-test.sh
```

Do not prune Docker globally: it may contain unrelated projects. The test containers
remove themselves; `docker compose down` removes this project's leftover network.
It does not remove unrelated containers or this project's built image.

## Milestone 2: visible delivery rover

The optional `desktop` Docker target adds Xvfb (a Linux display held in memory),
Openbox, x11vnc, noVNC and Mesa software rendering. All packages stay in the image;
macOS needs only its existing Docker Desktop and browser. The original `robotics`
target remains the headless baseline. The GUI was verified using Ogre 2 and Mesa
llvmpipe, OpenGL 4.5. This establishes usable rendering for this small scene, not
GPU acceleration or a camera-performance benchmark.

```bash
# Once, and after Dockerfile changes:
docker compose --profile gui build desktop

# Confirm the baseline, then verify three fresh rover/world loads:
./scripts/smoke-test.sh
./scripts/test-milestone-2.sh

# Launch a fresh, paused scene and leave it available in the browser:
./scripts/launch-rover.sh gui
```

Open [the local Gazebo desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
The launcher waits for the world and rover to load, records the evidence path,
and returns. The GUI can need a few additional seconds to render. Teal is the
pickup pad; amber is the delivery pad. The front of the rover has a white stripe.
Drag the 3D view to inspect it; scroll to zoom. The scene starts paused so the
initial condition is reviewable. Gazebo's play button runs passive physics only.
No driving interface is provided in Milestone 2.

Run the same launch command again for a clean reset. It recreates **only this
project's desktop container**, resetting the scene and camera. A browser reload
may be needed to reconnect. It writes a new timestamped evidence directory.

```bash
# Inspect the desktop and rover supervisor:
docker compose logs --tail 60 desktop
docker compose ps

# Stop the scene and its browser desktop:
docker compose stop desktop

# Optional headless scene; Ctrl-C stops it:
./scripts/launch-rover.sh headless
```

Port 6080 is published to **127.0.0.1 only**. VNC port 5900 is internal loopback
and not published. This local development desktop has no password; anyone with
access to the local endpoint can control it. Do not publish or tunnel it. No host
display mount, host network, privileged mode, or hardware device access is used.
The desktop and headless tests have different Gazebo discovery partitions.

### Separate launch and spawn (optional)

The normal launcher already performs both steps. To inspect them separately,
use two shells inside the **same** disposable container:

```bash
# Terminal A: starts the paused world in a named container.
docker compose run --rm --name delivery-spawn-demo robotics \
  gz sim -s -v 4 --seed 42 /workspace/simulation/worlds/delivery_room.sdf

# Terminal B: converts Xacro, validates it, spawns once, checks the actual scene.
docker exec delivery-spawn-demo /ros_entrypoint.sh python3 scripts/rover_sim.py \
  --spawn-only --evidence /workspace/evidence/milestone-2/manual-spawn
```

Stop Terminal A with Ctrl-C. Use a fresh evidence path on subsequent experiments.
The spawn operation requires this paused delivery world; duplicates fail without
renaming or moving the existing rover. Do not launch a second server in that
container. No colcon build is needed yet: Xacro and SDF are interpreted assets.

### What acceptance checks mean

`test-milestone-2.sh` creates three fresh containers, expands Xacro, validates
converted SDF, checks wheel joints/inertias, starts the paused server with seed 42,
and calls Gazebo's create service. It reads the **actual scene** and verifies all
11 expected model names and pose `(-2.5, -1.5, 0.25)` metres, zero rotation.
It then advances exactly 2000 steps at 1 ms and reads timestamped Gazebo pose and
statistics messages. The rover must remain upright, drift less than 5 mm, and
settle to a body-centre height of 0.235–0.245 m. Across restarts, initial pose
components must agree within `1e-9`, and settled components within `1e-6` (metres
for translation, dimensionless quaternion components for orientation).

Each run records generated URDF/SDF, source hashes, scenario settings, initial
scene protobuf text, spawn request/response, server log, package versions, and
results. The parent directory records image identity and comparison results.
An additional fresh container exercises the documented separate-spawn command
and confirms a second spawn fails with exactly one rover remaining in the scene.
Determinism here means repeatable initialization and this short passive settling
check on the recorded ARM64 image. It does not promise identical trajectories
across engines, platforms, or future dependency versions. RViz, odometry,
teleoperation, sensors and autonomy are not part of this acceptance test.

## Troubleshooting

- **Cannot connect to Docker socket:** the CLI exists but the engine is stopped.
  Start the existing Docker app; wait until `docker info` works.
- **Build cannot download packages:** inspect network access and the build log.
  Do not switch ROS/Gazebo release pairs to work around a transient download failure.
- **No advancing `/clock`:** check `gazebo.log` for a world/plugin error and
  `bridge.log` for bridge creation. The test uses the world-specific Gazebo clock
  topic and remaps it to ROS `/clock`; a paused server cannot pass.
- **GUI cannot connect to display:** use the `desktop` service, not `robotics`.
  `docker compose exec` does not source ROS automatically; use `/ros_entrypoint.sh`
  as in the separate spawn command. Normal startup already sources ROS.
- **Browser disconnected after restart:** reload the local noVNC page. Check
  `docker compose logs desktop`, then the run directory's `gui.log` and `gazebo.log`.
- **Port 6080 already occupied:** identify the owner; do not stop unrelated services.
  Choose another localhost host port in Compose and the browser URL if necessary.
- **World-control timeout:** service discovery is asynchronous. The launcher now
  waits up to 15 seconds for the named service and gives stepping a 10-second
  request deadline. It does not retry ambiguous stepping commands, which could
  advance physics twice. Rerun the fresh-container test and retain failed evidence.
- **Software-rendering warnings:** unsupported antialiasing and missing DPMS are
  recorded in the evidence. They did not prevent scene rendering. Missing world
  models, a blank scene, or exited Gazebo processes are failures, not cosmetic warnings.

## Milestone 3: teleoperation, wheel odometry and RViz

Build both images after the dependency additions, then run acceptance:

```bash
docker compose --profile gui build robotics desktop
./scripts/smoke-test.sh
./scripts/test-milestone-2.sh
./scripts/test-milestone-3.sh
./scripts/launch-milestone-3.sh gui
```

Open [the local ROS/Gazebo desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
It contains Gazebo, RViz and an xterm keyboard-teleop window. RViz loads the rover
model, TF axes/names and odometry arrows automatically. Its fixed frame is `odom`.
The launcher verifies advancing ROS odometry and a complete TF tree before returning.
Use the Linux desktop's Alt-Tab to switch windows. Click **inside the teleop
terminal** before using its keys:

| Key | Request |
| --- | --- |
| `i` | Forward, initially 0.2 m/s |
| `,` | Reverse |
| `j` / `l` | Left / right turn, initially 0.5 rad/s |
| `u` / `o` | Forward and turn left / right |
| `k` | Explicit zero velocity |
| Ctrl-C | Exit teleop; publishes a best-effort zero |

The command persists until changed. No command watchdog or e-stop exists yet.
Do not treat key release, a closed browser or disconnected terminal as a stop.
Press `k` before switching command sources. Stop all simulation processes with
`docker compose stop desktop`. Reset to the original pose by re-running the M3
launcher; do not use Gazebo's in-place world/time reset because odometry also
needs a fresh reference.

A Mac terminal can drive the **same** running desktop container:

```bash
# Interactive keyboard teleop (stop/exit the desktop teleop first):
docker compose exec desktop /ros_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5

# Send one forward Twist; this remains active until another command arrives:
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'

# Explicitly stop:
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'

# Inspect feedback, frame transforms and the current graph:
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /odom
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /joint_states
docker compose exec -T desktop /ros_entrypoint.sh ros2 run tf2_ros tf2_echo odom left_wheel --ros-args -p use_sim_time:=true
docker compose exec -T desktop /ros_entrypoint.sh ros2 node list
```

The long-running `tf2_echo` exits with Ctrl-C. Headless live launch is available
with `./scripts/launch-milestone-3.sh headless`. All communicating programs must
remain in the same container, as in prior milestones. The supervisor internally
uses `ros2 launch /workspace/simulation/launch/teleop.launch.py urdf:=<generated-urdf>
rviz:=true`; this launch starts the ROS side and expects the existing Gazebo server.
No ROS workspace compilation is required.

### Milestone 3 acceptance and evidence

`test-milestone-3.sh` creates an isolated disposable container and records the image,
package manifest, generated model, logs and results under a timestamped
`evidence/milestone-3/` directory. It runs five analytic odometry tests followed
by actual forward, reverse, left, right, curved and zero-velocity commands.
It checks wheel-angle direction, minimum commanded motion, a stationary final
pose, finite normalized odometry, increasing timestamps, 35–55 Hz observed
odometry, single TF authorities, every robot-link transform, and equality of TF
and odom at a shared timestamp. Tests use simulation-time durations with wall-time
deadlines, so slower rendering does not silently shorten a drive phase.

The test uses Gazebo pose only in its own process to compare relative displacement:
each settled phase must stay within 10 cm and 0.20 rad of wheel odometry. This is a
short nominal-floor sanity check, not a drift bound for long routes or hardware.
The zero test allows less than 5 mm further motion and less than 0.01 m/s or rad/s
reported velocity after settling. It tests an explicit stop command, not timeout
safety. TF is required to have precisely the documented five edges and no `map`
frame, with one odometry publisher and the two expected dynamic TF broadcasters.

To repeat the motion test in the visible desktop, stop keyboard command entry,
start from a fresh M3 GUI launch, and use a new evidence path:

```bash
docker compose exec -T desktop /ros_entrypoint.sh python3 tests/check_milestone_3.py --existing --evidence /workspace/evidence/milestone-3/manual-desktop-check
```

This moves the rover. Do not drive from another source while it runs. The test
sends zero in cleanup. See [M3 evidence](../evidence/milestone-3/README.md),
[the frame and odometry explanation](concepts/teleop-tf-odometry.md), and
[ADR 0003](decisions/0003-wheel-feedback-and-tf.md).

### Milestone 3 debugging

- **Robot moves in Gazebo but RViz is red:** inspect the run's `ros.log`; verify
  `/clock`, `/joint_states`, `/odom` and `use_sim_time`. Check RViz fixed frame
  `odom` and the `robot_description` topic's transient-local durability.
- **A turn produces inconsistent chassis position:** account for the axle 0.14 m
  ahead of `base_link`; do not label an axle-centred estimate as chassis odometry.
- **TF has competing parents or jumps:** do not add a second odometry broadcaster
  or bridge Gazebo's native TF. Only `wheel_odometry` owns `odom -> base_link`.
- **No wheel rotation:** check `/cmd_vel` and the one-way command bridge. M2 launch
  deliberately does not load the drive plugin; use `launch-milestone-3.sh`.
- **Odometry stops after resetting time:** restart the complete M3 launch. The
  estimator rejects backwards timestamps; an in-place reset is unsupported.

## Milestone 4: LiDAR, RGB camera, IMU and encoders

Build both images: the new `rover_interfaces/WheelEncoders` message is compiled
inside Docker, and the common `/ros_entrypoint.sh` sources its overlay. Rebuild
when changing `.msg`, `package.xml` or its CMake file. Sensor/launch Python, Xacro,
YAML and RViz assets are mounted source; restarting loads their changes.

```bash
docker compose --profile gui build robotics desktop
./scripts/smoke-test.sh
./scripts/test-milestone-2.sh
./scripts/test-milestone-3.sh
./scripts/test-milestone-4.sh
./scripts/launch-milestone-4.sh gui
```

Open [the local desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
The GUI launch runs a stationary sensor acceptance probe before returning. RViz
loads measured LiDAR points, the RGB image, TF frames and the established wheel
odometry. Use Best Effort reliability for sensor subscriptions. IMU and encoder
measurements are inspected numerically; no perfect attitude visualization is
fabricated. The same keyboard controls from M3 apply; press `k` to stop.

```bash
# Run the complete sensor stack without a desktop (EGL software rendering):
./scripts/launch-milestone-4.sh headless

# Inspect messages in the existing desktop container:
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /imu/data_raw
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /wheel/encoders
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /camera/camera_info
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic info --verbose /scan
docker compose exec -T desktop /ros_entrypoint.sh ros2 interface show rover_interfaces/msg/WheelEncoders
```

Expected simulation rates: scan/image/calibration 10 Hz, IMU 100 Hz, encoder 50 Hz.
`ros2 topic hz` measures arrival rate against wall time, which can be slower when
Mesa renders on the CPU. Acceptance computes rates from acquisition timestamps.
The sensor contract lists mounts, noise, covariance, limits and startup semantics.
Restart the whole launcher to reset; in-place backwards simulation time is unsupported.

### Automated sensor acceptance

`test-milestone-4.sh` runs analytic encoder checks then a fresh headless sensor
stack. It checks:

- More than two simulation seconds per stream, positive strictly increasing
  measurement stamps, exact frame IDs and TF paths at measurement time; 80–110%
  of requested rates and a single publisher per interface.
- LiDAR samples/angles/range bounds, expected east/north wall ranges, table
  occlusion, and nonzero plausible noise; RGB dimensions/encoding/nonblank data,
  matching image/calibration stamps and calibrated focal length.
- Stationary IMU gravity, configured gyro bias and white noise, finite values,
  covariance and unavailable orientation; encoder metadata, sequence and sample
  intervals, quantization versus matching joint-angle samples.
- Forward/reverse/left commands yield correct signed count changes, followed by
  an explicit zero command. This is not a safety/watchdog test.
- M3 odometry retains its original input boundary, no new odometry authority,
  no native pose topic and no map frame.
- A short MCAP bag includes all sensors, clock, static/dynamic TF, joint states,
  odometry and robot description. A fresh container in ROS domain 43 actually
  replays it, checks matching decoded message fields and image bytes, at least 90% delivery per sensor
  and usable sensor TF paths. Recording is stopped cleanly with SIGINT.

Observed ROS-clock age allows -0.15 to +0.5 s because independent subscription
callbacks are asynchronous; this is a stale/future sanity bound, not a precision
latency measurement. The simulated driver schedules IMU/encoder delivery no earlier
than acquisition +10 ms. Renderer/transport/queue time can add delay. Evidence
retains measured minima/maxima and the limitations are not calibrated away.

Tests write timestamped directories under `evidence/milestone-4/`, including the
expanded model and generated sensor world, image/package identity, source hashes,
logs, numerical results, MCAP recording and replay result. An initial scene is
required for the known-geometry stationary probe; do not run it after manually
driving without resetting first.

### Manual rosbag workflow

Use a fresh output path. Record inside the running desktop, then Ctrl-C after a
few seconds to close metadata cleanly:

```bash
docker compose exec desktop /ros_entrypoint.sh ros2 bag record --use-sim-time -s mcap -o /workspace/evidence/milestone-4/manual-bag --topics /scan /camera/image_raw /camera/camera_info /imu/data_raw /wheel/encoders /clock /tf /tf_static /joint_states /odom /robot_description

# Read without starting the simulator:
docker compose run --rm -T robotics ros2 bag info /workspace/evidence/milestone-4/manual-bag

# Replay in an isolated container, using the recorded /clock:
docker compose run --rm --name rover-m4-replay -e ROS_DOMAIN_ID=43 robotics ros2 bag play /workspace/evidence/milestone-4/manual-bag --delay 2

# While replay runs, inspect from another terminal IN THAT SAME container:
docker exec -it rover-m4-replay /ros_entrypoint.sh ros2 topic echo /wheel/encoders
```

Do not pass `--clock` when replaying the recorded clock, or run a second live
simulator/driver in the replay graph. For RViz replay, use the desktop image with
its X server and only the bag player plus RViz (`use_sim_time:=true`), without the
rover supervisor. The automated replay verifier needs no display.

Stop the live desktop with `docker compose stop desktop`. The GUI launcher
recreates only this project's desktop. No new macOS software is needed.

### M4 troubleshooting

- **Unknown `rover_interfaces`:** rebuild images and invoke container tools through
  `/ros_entrypoint.sh`; a shell launched directly by `docker exec` is not sourced.
- **Camera/LiDAR missing but IMU present:** inspect `gazebo.log` for Ogre/EGL errors.
  Headless M4 uses `--headless-rendering` and `LIBGL_ALWAYS_SOFTWARE=1`; GUI M4 uses
  the existing Xvfb display. Sensor generation requires rendering even without a GUI.
- **RViz has no scan/image:** check the topic and Best Effort QoS, simulation clock,
  and the transform from the named sensor frame to `odom`.
- **Bag empty or missing static TF:** wait for topic discovery, use the listed
  topics and `--use-sim-time`, and stop recording gracefully. Inspect `record.log`.
- **Camera calibration almost equal values:** focal lengths use floating point;
  compare with a small numerical tolerance, not exact equality.

## Milestone 5: simulated firmware and motor safety

M5 adds a compiled simulation motor driver inside the Docker images. Rebuild both
images when its C++ source changes. Python control/launch changes load on restart.

```bash
docker compose --profile gui build robotics desktop
./scripts/smoke-test.sh
./scripts/test-milestone-2.sh
./scripts/test-milestone-3.sh
./scripts/test-milestone-4.sh
./scripts/test-milestone-5.sh
./scripts/launch-milestone-5.sh gui
```

[Open the desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Gazebo, RViz, keyboard teleop and a read-only motor safety monitor run together.
M5 teleop uses **TwistStamped** with simulation time and `base_link` frame. A key
press issues a request that expires after 0.5 simulation seconds unless new key
events arrive. There is no background command repetition; key release/browser
loss stops renewing intent. `k` sends zero and engages braking. Do not use the old
M3/M4 unstamped commands against M5. The old launchers remain available, and their
lack of safety controls is still documented above.

Headless: `./scripts/launch-milestone-5.sh headless`. Stop the desktop with
`docker compose stop desktop`. Reset the scenario by relaunching the full stack;
do not use Gazebo's in-place time reset.

### Inspect and exercise the safety boundary

These commands target the existing desktop container. Stop interactive driving
before using another command source. ROS CLI fills `stamp: now` from its clock
on each publication; `use_sim_time` is required.

```bash
# A short request; timeout brakes automatically after its 0.5 s lease.
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.2}}}' --use-sim-time

# Assert e-stop (latched in firmware and driver).
docker compose exec -T desktop /ros_entrypoint.sh ros2 service call /safety/estop std_srvs/srv/SetBool '{data: true}'

# Release the input; this deliberately leaves the latch set.
docker compose exec -T desktop /ros_entrypoint.sh ros2 service call /safety/estop std_srvs/srv/SetBool '{data: false}'

# Keep zero intent fresh while resetting (run this in terminal A, Ctrl-C afterwards).
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: now, frame_id: base_link}}' --use-sim-time

# Terminal B: reset after encoder speeds are stationary. Check success in the reply.
docker compose exec -T desktop /ros_entrypoint.sh ros2 service call /safety/reset std_srvs/srv/Trigger '{}'

# Independent diagnostic streams:
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /firmware/state
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /motor/driver_state
```

Reset needs a released stop input, fresh zero command and fresh encoder speeds
below 0.15 rad/s. The driver also checks stationary wheels and a disabled frame.
It returns to WAIT_COMMAND; a new nonzero request is required to move. A zero
publisher left running can compete with teleop, so stop it before driving again.

### Acceptance and troubleshooting

`test-milestone-5.sh` runs deterministic core tests, then a fresh Gazebo scenario.
It verifies forward/reverse/turn tracking, 4 rad/s target saturation, 4 rad/s²
requested ramps and 2 N m torque bounds; physical timeout/e-stop stops; latch and
reset refusal/recovery; stale, future, nonfinite, wrong-frame and unsupported-axis
requests; missing encoders; frozen firmware; and native driver deadband plus
malformed/stale/over-limit effort rejection. Simulator pose is used only by the
test. Torque saturation and anti-windup are tested with deterministic locked wheels.
The live nominal tracking check uses averages over the final 0.6 simulation seconds
and a 0.15 rad/s error tolerance, accounting for count quantization.

Timeout must be observed within 0.65 simulation seconds after the streaming phase
ends, with less than 0.5 m total subsequent motion from the maximum wheel
request on this floor. E-stop travel from the nominal 0.2 m/s case must be under
0.12 m. After settling, every fault-stop check requires less than 5 mm of motion
over 0.5 s and wheel speed below 0.1 rad/s. These are nominal simulation acceptance
budgets, not guaranteed stopping distances under other loads/surfaces or hardware.

Run the same motion/command/e-stop checks in a fresh visible desktop:

```bash
docker compose exec -T desktop /ros_entrypoint.sh python3 tests/check_milestone_5.py --existing --evidence /workspace/evidence/milestone-5/desktop-acceptance
```

The `--existing` variant omits process suspension and native effort injection to
leave the desktop running. Headless acceptance covers those faults in its isolated
container. Do not enter keyboard commands during the test.

- **WAIT_COMMAND / IDLE:** expected startup/reset / explicit-zero states.
- **INVALID_COMMAND:** inspect stamp, frame, finite values, unsupported axes and
  duplicate publishers. Malformed input brakes and increments the counter.
- **ENCODER_TIMEOUT / INVALID_ENCODER:** inspect `/wheel/encoders`, its acquisition
  stamps and driver process. A new command cannot substitute for missing feedback.
- **CLOCK_STALLED:** physics or ROS clock delivery stopped advancing; restart the
  full launch if needed. The independent motor lease also expires.
- **ESTOP after releasing input:** intentional. Send fresh zero, wait stationary,
  then call reset and inspect its response.
- **Stale firmware telemetry + driver WATCHDOG:** the controller stopped executing;
  the driver independently brakes. Read both telemetry timestamps/ages.
- **Plugin cannot load:** rebuild both images and check the runtime `gazebo.log`;
  a superficially running ROS process is not acceptance evidence.

See [firmware concepts and full contracts](concepts/firmware-safety.md),
[ADR 0005](decisions/0005-encoder-pid-and-independent-motor-watchdog.md), and
[M5 evidence](../evidence/milestone-5/README.md).

## Milestone 6: mapping and localization

Rebuild both images. M6 installs Jazzy SLAM Toolbox and only the Nav2 map,
localization and lifecycle packages. The Dockerfile uses the official
`https://repo.ros2.org/ubuntu/main` archive, retaining the pinned image's ROS
signing key. This resolves this host network's 403/certificate failure for the
original hostname; certificate and package-signature checks stay enabled.

```bash
docker compose --profile gui build robotics desktop
./scripts/smoke-test.sh
./scripts/test-milestone-2.sh
./scripts/test-milestone-3.sh
./scripts/test-milestone-4.sh
./scripts/test-milestone-5.sh
./scripts/test-milestone-6.sh
```

The M6 acceptance script creates a fresh mapping container, drives a bounded
survey, saves a **new candidate map in its timestamped evidence directory**, then
destroys that container. A second fresh container localizes against that exact
candidate. It never overwrites `maps/delivery_room_v1/`. Bags, source/package
manifests, graph audits, map quality and localization error accompany the run.
Allow several minutes on this Mac's software renderer. A PASS requires the
complete workflows, not just successful process startup.

### Build and inspect a map

First inspect `docker compose ps -a`. Starting an existing M5 container does not
install M6 dependencies: Docker containers retain the image they were created
from. Build the images once as above. The M6 GUI launcher then replaces only this
project's `desktop` service with that image and the selected workflow. Use
`docker compose exec` for all subsequent inspection and driving in that desktop;
do not launch an additional simulator alongside it. Automated regression scripts
use isolated disposable containers to establish independent startup conditions.

```bash
# A fresh M6 scene, SLAM Toolbox, RViz and M5 stamped keyboard teleop:
./scripts/launch-milestone-6.sh mapping gui

# In another host terminal, collect overlapping views with the finite tape:
docker compose exec -T desktop /ros_entrypoint.sh python3 scripts/mapping_survey.py --profile mapping

# Save a new version; choose an unused path on each attempt:
docker compose exec -T desktop /ros_entrypoint.sh bash scripts/save-map.sh /workspace/maps/my_delivery_map/map
```

[Open the local desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
RViz's fixed frame is `map`; white is observed free space, black is occupied,
grey is unknown. Green laser points should agree with mapped surfaces. The AMCL
pose display is relevant only in localization mode. The map's `(0,0,0)` reference
is the initial mapping rover pose, not Gazebo world zero. M6 raises only the two
low cutaway walls to 0.8 m so the unchanged horizontal laser sees them.

The tape starts only in the documented fresh scenario, and uses fixed velocity
durations. It is repeatable teleoperation, not obstacle avoidance or navigation.
Do not enter keyboard commands while it runs. It aborts on a firmware fault or
stale clock/status and never clears e-stop. Stopping the process stops renewing
the M5 command lease; `k` requests immediate braking. All M5 e-stop/reset commands
above still apply. Relaunch to reset the whole graph and simulator clock.

### Localize against the versioned map after a clean restart

```bash
# Recreates the desktop: fresh physics, wheel odometry and AMCL; no mapper.
./scripts/launch-milestone-6.sh localization gui

# Supply the operator's approximate pose near the known pickup area.
# Defaults deliberately differ from the actual starting pose.
docker compose exec -T desktop /ros_entrypoint.sh python3 scripts/initialize_localization.py --x 0.25 --y -0.20 --yaw 0.20

# Small forward/reverse and turning observations let AMCL refine its estimate:
docker compose exec -T desktop /ros_entrypoint.sh python3 scripts/mapping_survey.py --profile localization

# Inspect the estimate, global correction and safety stop:
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /amcl_pose
docker compose exec -T desktop /ros_entrypoint.sh ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true
docker compose exec -T desktop /ros_entrypoint.sh ros2 topic echo --once /firmware/state
```

Stop `tf2_echo` with Ctrl-C. Before initialization, `map -> odom` is deliberately
absent and RViz may report missing rover/laser transforms. The map itself is
visible. RViz's **2D Pose Estimate** tool is an alternative to the initializer:
click near the pickup-area map origin and drag the heading arrow along +x.
The initializer's standard deviations are 0.25 m in x/y and 0.30 rad in yaw;
it reads only clock and publishes that operator input once. No simulator pose
is copied into it. AMCL's displayed covariance should shrink as scans agree with
the saved map, but correctness is established separately by the acceptance oracle.

AMCL's pose refers to `base_drive`, the axle frame 0.14 m ahead of `base_link`.
At the original pickup pose, the axle is near `(0.14, 0)` in this map, while the
initial chassis origin is `(0, 0)`. The default `(0.25, -0.20, 0.20)` prior is
deliberately approximate. `map -> base_link` remains the chassis pose for TF
inspection and acceptance. This frame choice prevents the differential motion
model from interpreting the chassis's turning arc as unintended translation.

Pass a different saved map as the third launcher argument:

```bash
./scripts/launch-milestone-6.sh localization gui /workspace/maps/my_delivery_map/map.yaml
```

Headless modes use the same stack: `./scripts/launch-milestone-6.sh mapping headless`
or `./scripts/launch-milestone-6.sh localization headless`. They name their container
`rover-m6`; run the same helper commands with
`docker exec -it rover-m6 /ros_entrypoint.sh ...` from another terminal.
All communicating processes must be inside that same container. Ctrl-C stops
headless launch; `docker compose stop desktop` stops the GUI.

### Record and replay measurements

Acceptance records compressed MCAP bags without camera video to keep the useful
mapping/localization evidence compact. For a manual capture, choose a new path:

```bash
docker compose exec desktop /ros_entrypoint.sh ros2 bag record --use-sim-time --include-unpublished-topics --qos-profile-overrides-path simulation/config/m6-record-qos.yaml -s mcap --storage-preset-profile zstd_fast -o /workspace/evidence/milestone-6/manual-bag --topics /clock /scan /odom /tf /tf_static /map /amcl_pose /initialpose /cmd_vel /wheel/encoders /firmware/state /motor/driver_state /robot_description
```

Reliable, transient-local QoS overrides retain the static map, robot description
and fixed transforms even when recording begins long after launch.
The unpublished-topic option lets the recorder subscribe to one-shot initial-pose
input before its publisher appears. Acceptance also waits for that recorder
subscription explicitly. Ctrl-C cleanly closes the bag.
`ros2 bag info <bag-directory>` inspects it. Replay
only in an isolated container/domain without simulator, firmware or estimators;
use the recorded clock, not a second `--clock` source. A visualization-only replay
can exclude commands and initial pose:

```bash
docker compose run --rm -T --no-deps -e ROS_DOMAIN_ID=43 robotics ros2 bag play /workspace/evidence/milestone-6/acceptance/mapping/bag --topics /clock /scan /odom /tf /tf_static /map /robot_description
```

### What M6 acceptance measures

- `/map` is a valid occupancy grid from SLAM Toolbox or the saved-map server.
- Received TF message GIDs resolve to a single expected publisher per edge;
  `map -> odom` is dynamic and belongs only to the active estimator.
- Estimator subscriptions and the native bridge match narrow allowlists. No
  simulator pose is bridged. The test-only oracle does not drive or initialize
  the estimator. M5 remains the sole `/cmd_vel` actuation consumer, with no DiffDrive;
  rosbag may subscribe as a read-only observer.
- Mapping observes >=90% of room interior (excluding a 10 cm buffer around the table),
  covers >=90% of each wall within 15 cm, and has occupied-surface p95 error
  <=15 cm. Compare to actual inner wall/table coordinates with the fixed initial
  reference; no fitted alignment removes distortion.
- A clean AMCL restart has no global transform before the approximate pose.
  After the localization tape, ten final timestamp-matched samples must have
  <15 cm position and <0.12 rad yaw error; x/y variance <0.04 m² and yaw variance
  <0.0225 rad², all positive and finite. The saved files must remain unchanged.
- After either tape, command timeout and stationary driver wheels verify the
  inherited M5 safety path. Full fault coverage remains in M5 regressions.

**Troubleshooting:** missing map means inspect lifecycle activation and `/scan`
in `ros.log`; do not add a fake map transform. Doubled/bent walls suggest bad
scan matches, time/extrinsic errors or insufficient overlap. A good-looking map
is not a substitute for numerical checks. A low covariance at the wrong location
can reflect symmetry; restart and supply a reasonable operator prior. In-place
Gazebo time reset is unsupported. See [concepts and limitations](concepts/mapping-localization.md),
[ADR 0006](decisions/0006-slam-toolbox-and-amcl.md), and [evidence](../evidence/milestone-6/README.md).
