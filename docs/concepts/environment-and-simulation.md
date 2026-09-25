# Why this environment exists

A real robot runs several programs at once: sensor drivers, localization, planners,
and motor controllers. **ROS 2** supplies communication and tools for connecting
those programs. A **node** is a participant in that system; a **topic** is a named
stream of typed messages. A subscriber receives messages published on a topic.

**Gazebo** supplies a simulated physical world. Its server computes time, motion,
and sensor effects; its GUI displays that world. Running only the server is called
**headless** operation. A **container** packages Linux software and its dependencies.
On macOS, Docker runs those containers inside a Linux virtual machine.

Gazebo and ROS speak different message formats. **ros_gz_bridge** translates between
them. For this milestone the entire data flow is:

```text
packaged shapes.sdf -> Gazebo physics server
                           |
                  Gazebo /world/shapes/clock
                           |
                    ros_gz_bridge
                           |
                      ROS /clock
                           |
                   test subscriber
```

The script reads the actual world name from SDF instead of hard-coding it. The bridge
is one-way, Gazebo to ROS, so ROS cannot feed a clock back into the simulator.

**Simulation time** measures time inside the simulated world. It can pause or run
at a different rate from wall time. Later robot nodes will set `use_sim_time=true`
so sensor timestamps, transforms, and navigation agree on time. The smoke observer
uses wall time for its deadline so it can diagnose a frozen simulated clock.

This test proves that a supplied world loads, simulation time advances, and messages
cross the bridge into ROS. It does not measure accurate physical motion or guarantee
future sensors and controllers will behave correctly.

## What changes on real hardware

Gazebo will be replaced by sensor drivers, motor interfaces, and the physical robot.
The simulator clock will usually be replaced by synchronized system/hardware clocks.
ROS message interfaces can remain similar, but real sensors require calibration and
have latency, dropped data, bias, and noise. Actual motors add saturation, friction,
power limits, and faults. Ground-truth poses available from a simulator must not become
an autonomy shortcut. A future emergency stop needs an independent hardware path;
a simulated stop alone cannot establish real-world safety.
