# ROS 2 runtime code

`rover_odometry/kinematics.py` implements wheel-only planar integration, including
the existing axle-to-chassis offset. `node.py` adapts actual `/joint_states` to
`/odom` and TF. It has no Gazebo dependency or command-velocity subscription.

The ROS launch file is `simulation/launch/teleop.launch.py`; it uses installed
ROS packages and executes this small Python node directly from the mounted
workspace. No colcon build is required at M3. See ADR 0003 for this scope choice.
