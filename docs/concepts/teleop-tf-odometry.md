# Driving and observing the rover

A ROS 2 **node** is a program with a focused responsibility. A **topic** is a named
message stream; a **message** defines the fields and types on that stream. Publishers
send messages and subscribers receive them. Discovery connects compatible peers.
A topic name alone is insufficient: message types and quality-of-service (QoS)
settings must also match. Here sensor-data QoS allows best-effort wheel feedback;
robot description and static TF are retained for late subscribers.

## Data flow and ownership

```text
keyboard teleop (or ros2 topic pub)
    -> /cmd_vel [geometry_msgs/Twist]
    -> one-way ros_gz_bridge -> Gazebo DiffDrive -> wheel joint motion
    -> Gazebo JointStatePublisher -> one-way bridge
    -> /joint_states [sensor_msgs/JointState]
        -> wheel_odometry -> /odom [nav_msgs/Odometry] + /tf (odom -> base_link)
        -> robot_state_publisher + URDF -> /tf (wheels) + /tf_static (fixed links)
Gazebo world clock -> one-way bridge -> /clock -> ROS simulation-time nodes
robot description + odom + TF -> RViz -> existing Xvfb/noVNC browser desktop
```

Gazebo scene/pose/statistics are available to acceptance tools only. There is no
ROS ground-truth pose topic. The drive plugin's native odometry and TF stay on
unused Gazebo topics; bridging them would create a second, incompatible authority.
No runtime odometry component imports Gazebo APIs or reads world pose.

`Twist.linear.x` requests forward speed in metres/second; `angular.z` requests
left-turn rate in radians/second. Other components are unused by this planar
platform. These unstamped commands are interpreted in the rover's body-aligned
axes at the drive axle. High-level autonomy can express desired body movement
without knowing wheel radius, gearbox ratios or electrical motor interfaces.
The drive layer converts `v,w` into wheel speeds `(v-w*b/2)/r` and `(v+w*b/2)/r`.
Only one command source should run at a time; command arbitration is not implemented.

## Frame contract

Coordinates use right-handed axes: x forward, y left, z up; metres and radians.
A TF transform describes the child frame's position and orientation in its parent.
Every child has exactly one parent and one owning broadcaster.

| Transform / frame | Owner and meaning |
| --- | --- |
| `odom` | Local, fixed frame initialized at the starting chassis centre, with body-aligned heading. Its z=0 plane is at nominal chassis height, 0.24 m above the floor. It is not the Gazebo world frame. |
| `odom -> base_link` | `wheel_odometry`, dynamic planar pose from wheel angle increments; z, roll and pitch remain zero. |
| `base_link -> left_wheel` | `robot_state_publisher`, origin `(0.14, +0.24, -0.10)` m; angle comes from the left joint feedback about +y. |
| `base_link -> right_wheel` | Same owner, origin `(0.14, -0.24, -0.10)` m; angle comes from right feedback about +y. |
| `base_link -> rear_support` | Same owner, static `(-0.23, 0, -0.19)` m. |
| `base_link -> parcel_tray` | Same owner, static `(-0.04, 0, 0.185)` m. |
| `map -> odom` | Absent. A future localization system owns this correction. No identity placeholder is published. |

The future tree is `map -> odom -> base_link -> robot links`. `odom` is continuous
but accumulates error; `map` is a globally corrected reference that may jump when
localization corrects its estimate. RViz uses **odom** as its fixed frame now.
There is no `world -> odom` TF: that would risk smuggling simulator truth into ROS.
The test aligns observed world and odom starting poses solely to compare motion.

## Wheel integration and the axle offset

Wheel feedback gives angular positions, not commanded values. For increments
`dL,dR` in radians, radius `r=0.14 m` and spacing `b=0.48 m`:

```text
ds = r*(dR+dL)/2
dyaw = r*(dR-dL)/b
```

We integrate the constant-curvature arc at the midpoint between the wheels.
The axle is `a=0.14 m` ahead of the chassis frame. The axle starts at `(a,0)` in
odom, and its integrated position becomes the chassis position through
`x_base=x_axle-a*cos(yaw)`, `y_base=y_axle-a*sin(yaw)`. Chassis-frame velocity is
`(v, -a*w, w)`: a turn about the axle gives the offset chassis centre a lateral
velocity. Reporting zero lateral velocity at that offset would be inconsistent.
The ROS launch reads dimensions directly from the generated URDF.

The first joint message establishes an arbitrary encoder reference. Subsequent
finite, strictly increasing timestamps drive integration; duplicates, backwards
time and malformed wheel messages are ignored. Restart the whole launch to reset
the world and odometry together. Mid-session Gazebo reset is unsupported.
Feedback uses unwrapped simulated joint positions. Real encoder wraparound needs
handling at the hardware interface.

Odometry and its TF share a timestamp and pose, published at at most 50 Hz. Wheel
TF uses the actual joint feedback timestamp. All runtime ROS nodes use simulation
time. The ROS graph cannot infer physics time from macOS wall time.

## What RViz tells an engineer

Gazebo shows the simulated physical world. RViz shows ROS's model and estimates.
Use the robot display to catch wrong geometry, TF axes to catch reversed frames,
and odometry arrows to spot jumps or implausible motion. A red missing-transform
status often means absent publishers, incompatible QoS, wrong frame IDs or clock
mismatch. A visually smooth estimate still can be wrong: compare independent
measurements, as the test does here. The grid is drawn 0.24 m below odom because
this project's odom origin is at chassis height.

## Simulation versus hardware

| This milestone | Physical robot / later work |
| --- | --- |
| Ideal Gazebo joint-velocity actuation | Motor dynamics, drivers, PID, saturation, battery/load effects; M5 |
| Exact simulated wheel angles, no injected measurement noise | Encoder ticks, quantization, missed counts, calibration and timestamp latency |
| Rolling kinematics estimates motion | Slip and incorrect radius/track accumulate position and heading drift |
| Fixed nonzero covariance placeholders, huge out-of-plane variances | Measured/calibrated covariance and uncertainty propagation before fusion; these values are not accuracy guarantees |
| Flat-floor planar estimate | Chassis pitch, roll, suspension and uneven ground require additional sensing/modeling |
| A `k` / zero Twist explicitly stops commanded motion | No watchdog, e-stop, command timeout or safety certification in M3; those are M5 |
| Restart resets all estimation state | Hardware needs controlled initialization and recovery across resets |
| CPU software-rendered desktop | Visual inspection works here; graphics/sensor throughput is not benchmarked |

A stopped keyboard publisher does not guarantee a stopped robot: Gazebo retains
the last command. Send zero explicitly, or stop the project desktop/container.
Keyboard interruption sends a best-effort zero, not a safety guarantee.
No LiDAR, camera, IMU, synthetic encoder sensor/noise model, navigation or firmware
safety is introduced. Joint-state feedback is the minimum mechanical observation
needed for M3 wheel odometry.

## References

- [ROS mobile frame convention, REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
- [Gazebo Harmonic DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Gazebo Harmonic JointStatePublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html)
