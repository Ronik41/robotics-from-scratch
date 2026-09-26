# Mapping and localization

An **occupancy grid** divides a horizontal plane into square cells. ROS uses
`-1` for unknown and 0–100 for occupancy probability. Our saved trinary map uses
white for free, black for occupied and grey for unknown. YAML records cell size,
the position of the lower-left image origin in `map`, and classification thresholds.
The PGM's top row is the map's highest y; ROS grid row zero is its lowest y.

A laser ray supplies evidence of free space before its return and an occupied
surface at its endpoint. A missing or occluded observation is not proof of free
space. This map represents the horizontal LiDAR slice, not a complete 3D room,
semantic labels, or a configuration-space map of where the whole rover fits.
Obstacle inflation, traversability and planning are later milestones.

**SLAM** means simultaneous localization and mapping. Wheel travel predicts the
change in pose; matching a new LiDAR scan against nearby scans improves that
estimate. A pose graph stores those scan poses and relationships. **Loop closure**
recognizes a previously visited place and adds a constraint that can correct
accumulated error throughout the graph. SLAM Toolbox then rasterizes the scans
into an occupancy grid. Returning near the start makes overlap available, but
does not by itself prove a loop-closure constraint was accepted.

**Localization** estimates pose in an existing, fixed map. AMCL maintains many
candidate poses, propagates them using wheel motion, scores their scan agreement,
and resamples promising candidates. We initialize a distribution near the known
pickup location rather than giving AMCL an exact pose. Moving a little changes
the observations and helps reduce ambiguity. The saved map stays unchanged.

## Coordinate frames and drift

```text
map -- SLAM Toolbox OR AMCL --> odom -- wheel_odometry --> base_link
                                                        |
                                             robot_state_publisher
                                                        |
                                         lidar_link / wheels / other links
```

`base_link` is attached to the rover chassis. `odom` begins at startup and gives
a continuous local motion estimate. Small wheel-radius errors, slip and integration
errors accumulate into **drift**. `map` is the reference for observations of the
room. A new match can correct the global pose abruptly, so the global correction
belongs in `map -> odom` while local odometry stays continuous:

`T_map_odom = T_map_base × inverse(T_odom_base)`.

There is exactly one owner per TF edge. Publishing an identity static map transform
alongside the estimator would hide startup problems and create competing answers.
Mapping starts its own map reference at the initial rover pose. Localization starts
fresh local odometry and computes the correction needed to place it in the saved map.
The YAML image origin is a grid indexing offset, not the rover's initial pose.
The existing planar TF model keeps base_link at z=0; the grid is an x/y
projection, not a measured floor elevation. This does not model ramps or floors.

M6 also adds `base_drive`, a fixed frame at the wheel axle, 14 cm forward of
`base_link`. AMCL's differential-drive motion model uses this rotation centre.
During an in-place turn, the axle centre stays nearly fixed but the offset chassis
centre moves along a small arc. Giving that arc to an axle-based motion model can
inflate its rotation/translation uncertainty. The initial experiment exposed this
as covariance spikes despite a reasonable mean position. We fixed the frame
choice and retained the uncertainty acceptance limits. `/amcl_pose` describes the
axle; TF still provides the chassis pose and the existing wheel odometry edge.

## Why matching fails

Repeated corridors and symmetric rectangles can make different positions look
similar. Long parallel walls constrain distance across a corridor much better
than distance along it. Open spaces may contain too few returns. Rapid turns,
incorrect timestamps/extrinsics, severe slip, moving objects or too little scan
overlap can make the right match fall outside the search range. A false loop
closure can bend or duplicate walls. Low reported covariance is not a guarantee
that the selected match is correct.

This is LiDAR SLAM: visual appearance is not an input. Textureless or repeated
visual scenes are analogous failure modes for camera SLAM; here the relevant
ambiguity is geometry. The asymmetric table helps, while crate height limits
which objects actually provide geometric information.

## Simulation and hardware boundary

Gazebo's world pose knows the answer because the simulator owns the world. A real
robot has no equivalent perfect sensor. Feeding that pose into mapping or
localization would invalidate the exercise. Only the acceptance process reads it,
compares poses at matching timestamps, and reports error; it never publishes pose
or control inputs to the estimator. Room geometry is likewise used only to score
the generated map. No best-fit alignment is used to disguise map distortion.

The M6 room raises the two cutaway walls so the existing LiDAR sees all four
boundaries. Low crates and painted pads remain invisible to this 2D slice. Camera
data could eventually help detect them, but M6 adds no perception or avoidance.
The scripted survey assumes the documented empty route and fresh startup pose;
it is not safe autonomous navigation in an arbitrary room.

Real deployment needs calibrated wheel geometry and sensor extrinsics, hardware
timestamps/time synchronization, encoder-based odometry with realistic uncertainty,
scan distortion handling where appropriate, and testing under slip and changing
objects. M3 currently uses ideal-resolution joint angles for odometry, although
M5 PID already uses quantized, delayed counts. This distinction is retained
explicitly; neither estimator consumes simulator chassis pose.

M5 watchdogs, e-stop and finite braking still guard every requested motion.
Localization confidence does not bypass them. M6 does not establish stopping
safety for low unseen obstacles, global relocalization, or real-world reliability.
