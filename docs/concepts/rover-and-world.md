# Why the rover starts with geometry and frames

Differential drive uses independently rotating left and right wheels. For wheel
radius `r`, wheel-centre spacing `b`, and wheel angular speeds `wl`, `wr`:

```text
forward speed v = r (wr + wl) / 2
turning speed w = r (wr - wl) / b
```

Equal wheel speeds move straight; opposing speeds turn in place. This rover has
`r = 0.14 m` and `b = 0.48 m`. On a real rover, these equations connect requested
body motion to motor speeds, and encoder readings to estimated motion. They assume
rolling without slipping. Actual wheel size, tyre compression and slip need
calibration. Here we implement the mechanical layout only. Driving and odometry
verification begin in Milestone 3; motor PID and safety belong to Milestone 5.

A coordinate frame is an origin plus three axes. The world frame is fixed in the
room; `base_link` sits at the chassis centre and moves with the rover. Following
ROS conventions, x is forward, y left, and z up; distances are metres and angles
are radians. Left/right wheel frames are at `(0.14, +/-0.24, -0.10)` relative to
`base_link`, with their rotation axes along +y. The wheel cylinders are rotated
so their axles, not their faces, align with y. The initial world-to-body pose is
`(-2.5, -1.5, 0.25)` with zero rotation. Wheel bottoms initially have 1 cm clearance.

Real robot software needs frame relationships to interpret a sensor measurement
or motor motion relative to the body. The model declares these relationships; it
does **not** yet publish a ROS TF tree. `map` and `odom` are future localization
frames and are deliberately absent from this milestone.

URDF describes rigid links connected by joints. Xacro generates URDF using
parameters and macros so both wheels share dimensions and inertia equations.
Three descriptions serve different purposes: visuals make the robot visible,
collisions determine contact, and inertia tells physics how mass resists motion.
A real robot also uses URDF for geometry and kinematics, but its physical mass,
contacts and motors exist independently of that file. Gazebo converts the URDF
to SDF; fixed parcel geometry is merged into the parent chassis while its mass
is retained. The passive support's fixed joint is preserved for separate friction.

SDF describes the simulation: room, fixed obstacles, lighting, gravity, physics
step size and plugins. The 8 by 6 metre room contains painted pickup/delivery
pads, a sorting block and two crates. Painted pads have no collision. Every asset
is a repository-local primitive; there are no remote Fuel dependencies.

```text
rover.urdf.xacro -> xacro -> rover.urdf -> Gazebo create service
                                              |
delivery_room.sdf -> paused Gazebo server <-----+
                                              |
                           scene / pose / statistics -> acceptance evidence
                                              |
                           Gazebo GUI -> Xvfb -> noVNC -> local browser
```

The acceptance checker uses simulator ground truth only as a test oracle. Future
autonomy must use sensors and localization, not this privileged pose information.

## Simulation versus a physical rover

| Here | Physical hardware would require |
| --- | --- |
| Perfect cylinders and known 10.3 kg total mass | Measured geometry, mass distribution and tyre deformation |
| Frictionless spherical rear support | A caster with swivel dynamics, friction, bearing resistance and possible shimmy |
| Uniform floor and constant wheel friction | Varying surfaces, slip, thresholds and uneven terrain |
| Exact initial pose, fixed seed, no injected noise | Pose initialization and uncertainty estimates |
| Fixed parcel tray/block | Changing payload mass, retention and centre of mass |
| Passive physics, no drive system yet | Motor drivers, encoders, power, limits, watchdog and e-stop |
| CPU-rendered display | Real camera optics, exposure, lighting and timing |

Passing initialization and settling checks establishes a useful starting model,
not real-world reliability or navigation capability.

## References

- [ROS coordinate conventions, REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- [URDF in Gazebo Harmonic](https://gazebosim.org/docs/harmonic/spawn_urdf/)
- [Gazebo world descriptions](https://gazebosim.org/docs/harmonic/sdf_worlds/)
- [Software-rendering troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/)
