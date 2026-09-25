# Simulated firmware and motor safety

## What lives on which computer?

A robot's Linux computer usually runs ROS 2, user interfaces and planning. It
sends desired motion; it does not directly connect a browser key to motor power.
A microcontroller is a small processor close to the motor electronics. Firmware
on that processor reads encoders, controls motor current/velocity and checks
communication deadlines independently of the Linux application.

Here `sim_firmware` is a separate Python ROS process, **not a real-time MCU**.
`rover::MotorDriver` is a small C++ Gazebo system representing motor electronics.
The extra language is limited to the physics boundary: it can brake when the
Python process is frozen, without relying on ROS timers or shutdown cleanup.
M5 opts out of DiffDrive completely. M2–M4 remain historical baseline modes and
retain their old behavior; use `launch-milestone-5.sh` for safety control.

```text
operator / future high-level source
   | /cmd_vel: TwistStamped (body velocity, acquisition timestamp)
   v
sim_firmware: validation -> differential-drive kinematics -> acceleration ramp
   ^                                                      |
   | /wheel/encoders                                     PID
   |                                                      |
sim_sensor_driver <--- joint rotation       /motor/effort: bounded wheel torque
   ^                                                      v
   |                                        MotorDriver: lease, latch, deadband
   |                                          | normally-on damping brake
   +--------------------- Gazebo physics <----+
```

No pose, odometry, LiDAR or IMU enters the controller. Motor electronics use joint
velocity only for the simulated damping brake; firmware speed feedback is derived
solely from M4's delayed, quantized encoder counts. Driver telemetry is inspectable
but is not fed back into firmware. M3 odometry and TF ownership are unchanged.

## Wheel velocity PID

An encoder reports discrete increments of rotation. With 2048 decoded counts per
wheel revolution, speed is `delta_counts * 2*pi / 2048 / acquisition_dt`. The M4
measurement delay remains at least 10 ms, and sampling is nominally 50 Hz. At a
20 ms interval, one count corresponds to about 0.1534 rad/s: instantaneous speed
will visibly step even during smooth motion. Missing deliveries can use a longer
acquisition interval; a stale stream cannot keep torque enabled.

For body forward speed `v` and yaw rate `w`, left and right wheel targets are
`(v - w*track/2)/radius` and `(v + w*track/2)/radius`. Radius and track are extracted
from the expanded robot description (0.14 m and 0.48 m). The targets describe the
axle's differential-drive motion, with the same base-link offset convention as M3.

PID means proportional, integral, derivative. P reacts to speed error now. I
accumulates persistent error to overcome resistance. D reacts to changes and helps
damp the response. This controller differentiates measured velocity instead of
the target, so a changed request does not produce a derivative spike. A 50 ms
first-order filter reduces encoder quantization noise in that derivative.

For each wheel, `effort = Kp*error + integral - Kd*filtered_speed_derivative`.
The integral stores torque units and accumulates `Ki*error*dt`. Conditional
integration suppresses further windup when the requested torque is saturated and
the error would push farther into saturation. All I/D/ramp state resets on a stop.
The pure core has explicit clocks and no middleware, making its behavior testable
with a deterministic quantized plant and a locked-wheel scenario.

| Setting | M5 value / behavior |
| --- | --- |
| Wheel speed request | ±4 rad/s; both targets scaled together to preserve curvature |
| Wheel acceleration request | ±4 rad/s² per wheel, using encoder acquisition dt |
| PID gains | Kp 0.45; Ki 0.8; Kd 0.002, SI-based torque/velocity units |
| Integral torque | ±1.2 N m |
| Requested motor torque | ±2 N m per wheel, checked again by the driver |
| Motor deadband | Applied drive torque is `sign(u)*max(abs(u)-0.04, 0)` N m |
| Brake | `clamp(-wheel_speed * 1 N m s/rad, ±2 N m)` at each physics step |
| Zero body request | IDLE; brake immediately, bypassing the normal acceleration ramp |
| PID update | Each valid new encoder acquisition, nominal 50 Hz |
| Firmware safety polling | 10 ms steady-clock timer; not a hard scheduling guarantee |
| Firmware state | `/firmware/state`, JSON in `std_msgs/String`, up to 20 wall Hz |
| Driver state | `/motor/driver_state`, JSON in `std_msgs/String`, about 20 simulation Hz |

Acceleration limits bound the **requested wheel speed ramp**, not measured chassis
acceleration on all surfaces. Torque and braking have physical lag; a timeout is
not an instantaneous stop. Saturation means the requested value exceeded a limit.
Motor deadband means a small input produces no drive torque. Integral action can
work through deadband, but these gains and bounds are simulation teaching values,
not calibrated motor specifications.

## Command, feedback and stop contracts

M5 `/cmd_vel` uses `geometry_msgs/msg/TwistStamped`, with a positive simulation-time
stamp and `header.frame_id: base_link`. Only `twist.linear.x` and
`twist.angular.z` may be nonzero. M3/M4's unstamped Twist is intentionally not
accepted in M5: a receiver cannot distinguish an old unstamped packet from a new
one. There is no receive-time restamping adapter that would hide upstream delay.
A future planner must provide a genuinely fresh acquisition timestamp.

A request is rejected if stale (>0.5 simulation seconds), too far in the future
(>30 ms), nonfinite, out of order, duplicate-stamped, in the wrong frame, or using
unsupported axes. Finite values that overflow wheel kinematics are rejected too.
Rejection inhibits torque, increments a counter, clears PID state and does not
renew the last accepted command timestamp. A newer valid request can recover;
there is no automatic replay of a rejected request. Queue depth is one.

Encoder metadata must identify the two expected wheel joints, base frame, 2048
counts/revolution, an increasing index and acquisition stamp, and a plausible
positive sample period. The receiver supports skipped deliveries by differencing
cumulative counts over its longer receive interval. Nonfinite/stale/reordered
samples, wrong metadata and derived speed above 20 rad/s inhibit torque. A fresh
stream and new valid commands permit recovery. Restarting a sensor's count/index
origin alone is unsupported: restart the full stack.

| Protection | Deadline and action |
| --- | --- |
| High-level command watchdog | >0.5 s simulation age **or** >1 s monotonic receive age: brake |
| Encoder watchdog | >0.2 s simulation acquisition age **or** >1 s monotonic receive age: brake |
| Clock stall | No observed simulation-time progress for >1 wall second: inhibit; driver also expires |
| Driver watchdog | Last torque frame >0.15 simulation seconds old **or** >750 ms monotonic receipt age: brake |
| Backwards clock | Latch e-stop; full launch restart is the supported reset |
| E-stop assertion | Latches both firmware and driver; subsequent motion commands cannot clear it |

The native driver protocol is `gz.msgs.Double_V` on `/motor/effort`:
`[simulation_stamp, left_Nm, right_Nm, mode]`, where mode is 0 brake, 1 drive,
2 latch e-stop. The driver checks length, finite fields, ±2 N m bounds,
strictly increasing timestamps, 150 ms maximum age and 30 ms future tolerance.
Malformed input brakes without refreshing the accepted stamp. An unchanged
simulation stamp is never used to renew a lease. The driver enforces torque,
deadband and latching on every running physics step. If physics itself is paused,
nothing physically moves; lease expiry is evaluated when stepping resumes.

`/firmware/state` includes state/reason, e-stop input and latch, target/ramped/
measured wheel speed, requested effort, integral torque, saturation flags, ages
and rejection counters. `/motor/driver_state` independently reports DRIVE,
BRAKE, WATCHDOG or ESTOP, applied torque, actual wheel speed and rejected-frame
count. During a firmware freeze its last state message becomes stale; the driver
continues reporting its watchdog brake. Neither discovery isolation nor these
public ROS/Gazebo interfaces is an authentication/security boundary.

## E-stop reset is a deliberate sequence

1. Assert `/safety/estop` (`std_srvs/SetBool`, `data: true`). This latches the stop.
2. Release the input with the same service and `data: false`. This **does not clear** the latch.
3. Send a fresh zero stamped command and allow fresh encoder measurements to show
   both wheels below 0.15 rad/s.
4. Call `/safety/reset` (`std_srvs/Trigger`). Firmware checks the released input,
   fresh zero command and stationary fresh feedback. The driver separately checks
   a recent disabled frame and stationary physical wheels before clearing its latch.
5. Reset leaves firmware in WAIT_COMMAND and the driver disabled. Only a **newer**
   valid request can move the rover again.

A refused reset reports its reason. The driver's latch survives a firmware process
restart because it lives in Gazebo; restarting the entire simulation is analogous
to a new power session and clears state. This is not nonvolatile hardware latching.

## Why the browser and planner cannot own safety

A browser may miss key release, a terminal may disappear, and a planner may freeze
while its last request asks for motion. The firmware does not require a final
zero message to detect a silent sender. The driver does not require the firmware
to execute a graceful shutdown. A source that keeps producing fresh nonzero
requests is still commanding motion: timeout cannot identify its semantic mistake.
A separate stop assertion and bounded local motion remain necessary.

On hardware, replace the simulated driver with PWM/current-controlled electronics,
real encoder capture, an independent MCU watchdog, measured motor constants and
appropriately engineered physical emergency-stop/power/brake circuits. Linux
scheduling, ROS services and a browser button are not substitutes for those
circuits. The damping brake here assumes braking remains available after loss of
the firmware heartbeat; it does not model failed motor electronics, battery loss,
thermal/current limits, backlash, pulse loss, slopes or all friction conditions.
No safety certification or real-world stopping-distance claim follows from M5.

Implementation references: [Gazebo JointForceCmd](https://gazebosim.org/api/sim/8/jointforcecmdcomponent.html)
and [ROS control_toolbox PID](https://docs.ros.org/en/jazzy/p/control_toolbox/generated/classcontrol__toolbox_1_1Pid.html).
M5 uses a small explicit Python PID model rather than adopting a larger
ros2_control configuration at this milestone.
