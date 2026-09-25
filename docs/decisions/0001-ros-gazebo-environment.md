# ADR 0001: ARM64 Ubuntu container with ROS 2 Jazzy and Gazebo Harmonic

- Date: 2026-09-25
- Status: Accepted; initial clean build and headless integration smoke test passed
- Scope: Milestone 1 only

## Context and inspection

The host is macOS 26.6.2 (25G83), ARM64, with 10 logical CPUs, 16 GiB RAM,
and approximately 156 GiB available disk. Docker Desktop and Compose are already
installed. The initial Docker CLI check failed because the engine was stopped;
starting the existing app made the daemon available. The daemon is aarch64 with
10 CPUs and 8,321,712,128 bytes RAM. Docker CLI is 29.4.3 and Compose is 5.1.3.

VirtualBox is installed and lists a course VM named Mininet. It is not a clean
robotics environment. No `podman`, `colima`, `limactl`, `orb`, or `multipass` command
was found on PATH. We leave the course VM alone. See
[`host-environment.txt`](../../evidence/milestone-1/host-environment.txt).

## Decision

Use the existing Docker Desktop Linux backend, a native ARM64 Ubuntu 24.04 image,
ROS 2 Jazzy, and Gazebo Harmonic through `ros-jazzy-ros-gz`. Keep all communicating
ROS/Gazebo processes within one container for this stage. Run Gazebo's packaged
`shapes.sdf` server headlessly and bridge its clock into ROS.

Jazzy/Harmonic is a documented compatible LTS pair. Newer Lyrical/Jetty is also
listed upstream; choosing Jazzy here deliberately favors the established Noble
package ecosystem and tutorial continuity rather than claiming Jazzy is newest.
The official ROS image manifest was inspected and contains a native ARM64 variant.

The base image is pinned to manifest digest
`sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8`.
Apt packages are resolved at build time and recorded in a manifest; this is not
an immutable archive of every dependency. No second Gazebo apt repository is added.

## Alternatives considered

| Workflow | Assessment on this Mac |
| --- | --- |
| Existing Docker Desktop + ARM64 Ubuntu | Selected: existing engine, standard Linux packages, scripted builds, easy headless checks |
| Native macOS ROS/Gazebo | More platform/build variation; upstream documents Mac Gazebo GUI instability |
| Dedicated Ubuntu desktop VM | Useful graphics fallback, but requires provisioning a new OS and desktop now |
| Reuse Mininet VM | Mixes unrelated course dependencies and lacks a verified robotics baseline |
| AMD64 container emulation | Unnecessary when a native ARM64 official base exists |

## Consequences

- Repository files stay editable on macOS; robotics dependencies remain in Linux.
- A smoke test can run without a window or GPU and checks actual ROS message delivery.
- Graphical acceleration, virtual desktop access, and RViz remain unverified.
- Docker Desktop adds a Linux VM and consumes RAM; rendering performance must be
  measured before relying on cameras or complex scenes.
- Starting Docker Desktop can resume unrelated containers with restart policies.
  An existing Home Assistant container was observed running after engine startup;
  this task did not modify its configuration or manage its lifecycle.
- No system-wide software was installed, Git changes pushed, or content published.

## Verification and acceptance

`./scripts/smoke-test.sh` launches the installed shapes example, starts a one-way
`ros_gz_bridge`, and uses `rclpy` to receive real `rosgraph_msgs/msg/Clock` messages.
It requires at least ten samples, at least 0.1 seconds of simulated-time advancement,
no backwards clock movement, and live server/bridge processes. A wall-clock deadline
prevents a stopped simulator from hanging the check. Process logs, the world's checksum
and model names, package versions, and image identity are saved with each run.

This satisfies the Milestone 1 known-example launch criterion when the test passes.
It does not establish a working rover, physical sensor fidelity, GUI rendering, or
real-world reliability. Those claims require later milestone evidence.

Observed acceptance on 2026-09-25: Gazebo Sim 8.15.0 initialized the shapes world;
the ROS subscriber received 101 clock samples from 0.002 to 0.102 simulated seconds.
The command exited zero and the disposable container was removed. See
[`result.json`](../../evidence/milestone-1/20260925T183709Z-56573/result.json) and
[`the evidence index`](../../evidence/milestone-1/README.md). The server logged mesh
fallbacks for the cone and ellipsoid collision geometry; these are recorded without
claiming that the demo validates collision fidelity.

## References checked 2026-09-25

- [Gazebo's ROS compatibility and package guidance](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Gazebo supplied shapes example and headless mode](https://gazebosim.org/docs/harmonic/getstarted/)
- [ROS Jazzy Ubuntu binary platforms](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html)
- [Official ROS container image source](https://github.com/osrf/docker_images)

Exact startup and follow-up commands are in [`docs/setup.md`](../setup.md).
