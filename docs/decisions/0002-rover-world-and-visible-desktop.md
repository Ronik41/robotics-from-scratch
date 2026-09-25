# ADR 0002: Local desktop, Xacro rover and deterministic paused spawn

- Date: 2026-09-25
- Status: Accepted; see Milestone 2 evidence index
- Scope: Milestone 2 only

## Context

ADR 0001 established working ARM64 Jazzy/Harmonic physics and ROS clock delivery
inside Docker but did not establish graphics. Milestone 2 needs a visible rover
and repeatable initialization without new host software or beginning teleoperation.

## Decision

1. Extend the existing Dockerfile with an optional `desktop` target. Keep the
   `robotics` target unchanged in purpose. Use Xvfb, Openbox, x11vnc and noVNC with
   Mesa software rendering. Publish only `127.0.0.1:6080`; keep VNC internal.
   Supervise the desktop, Gazebo server and GUI so failed children are observable.
2. Author the rover in Xacro/URDF, with analytic box/cylinder/sphere inertias and
   collision geometry. Two independent wheel joints form a differential-drive
   chassis; a low-friction rear sphere approximates passive caster support.
   Leave drive plugins, commands, TF, ROS odometry and RViz for Milestone 3.
3. Author the small world in SDF using only local primitives. Spawn the rover by
   Gazebo Transport's EntityFactory service. No ROS package or colcon build is
   necessary at this stage; ROS launch integration can follow when nodes exist.
4. Start paused, seed 42, step size 1 ms, explicit name and pose from one scenario
   file. Reject duplicate names. A GUI restart recreates only the project desktop
   container; automated acceptance uses three isolated disposable containers.
5. Read back scene membership and pose, then check 2000 passive physics steps.
   Record actual image/package identity and generated artifacts with results.

## Evidence and consequences

The existing image passed Milestone 1 again before any implementation. Gazebo's
shapes example then rendered visibly in the local browser using llvmpipe/OpenGL
4.5. The completed rover loaded at the specified pose across three fresh starts
and settled upright with identical recorded poses. See
[the evidence index](../../evidence/milestone-2/README.md).

An initial three-run test failed on its third world-control request despite a
ready scene. Discovery of one service did not prove readiness of another.
We added explicit per-service discovery waits and a longer control timeout,
without blindly retrying mutations. The failure is retained in the evidence.
A GUI anchor warning was fixed by matching the viewport title and anchor name.
Software antialiasing/DPMS warnings and an upstream WorldStats QML layout warning
remain cosmetic for the verified scene; authoritative timing comes from messages.

This avoids a new macOS display server or Linux desktop VM. It adds image size
and CPU rendering cost. The local browser endpoint is unauthenticated and must
remain local. The GUI has only been validated for this small world; later sensor
performance may require reassessment. Apt packages remain unpinned beyond the
base image digest, so this is not a bit-for-bit dependency lock.

Repeatable initialization on one recorded image is the determinism claim.
Arbitrary trajectories and cross-platform physics equivalence are not claimed.
The model is not yet a driven rover; it is the verified mechanical and world
foundation for the next milestone.

## Alternatives

- Native XQuartz/host installation: unnecessary after container rendering worked.
- Dedicated desktop VM: retained as a future performance fallback, not provisioned.
- Handwritten SDF-only rover: less useful for future ROS robot-description work.
- Remote world/model assets: avoided to keep this scene offline and repeatable.
- Automatic unpause or spawn-at-arbitrary-time: avoided so initial conditions
  can be observed before any physics integration.
