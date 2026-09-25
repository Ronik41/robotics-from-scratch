# ROS 2 runtime code

`rover_odometry/kinematics.py` implements wheel-only planar integration, including
the existing axle-to-chassis offset. `node.py` adapts actual `/joint_states` to
`/odom` and TF. It has no Gazebo dependency or command-velocity subscription.

The ROS launch file is `simulation/launch/teleop.launch.py`; it uses installed
ROS packages and executes this small Python node directly from the mounted
workspace. No colcon build is required at M3. See ADR 0003 for this scope choice.

M4 adds `rover_sensors/`: a simulation device adapter that emits imperfect raw
IMU measurements and quantized wheel counts, with no estimator or control output.
`rover_interfaces/` is the small colcon-built message package baked into both
Docker images. Rebuild when its message schema changes. The driver and launch
source remain mounted. Existing `rover_odometry` is unchanged.


M5 adds `rover_control/core.py` (deterministic PID/safety model) and `node.py`
(ROS adapter consuming stamped commands and M4 encoder counts). `rover_motor_driver/`
is a C++ Gazebo plugin compiled in the Docker image; it applies bounded wheel
torque/deadband/braking and independently expires stale motor frames. Rebuild both
images for C++ changes. Runtime Python remains mounted. Neither new component
consumes Gazebo pose; firmware does not consume ideal joint speed. See ADR 0005.
