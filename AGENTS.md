# Robotics from Scratch

You are helping build and teach an industry-style, hardware-free robotics project.

## Working agreement

- Before substantial work, read `PROJECT_SPEC.md`, `PLAN.md`, and `STATUS.md`.
- Work one milestone at a time. Do not advance until its acceptance checks pass or are recorded as blocked.
- Handle implementation and testing, but first explain: what is changing, why a real robot needs it, the data flow, and how it will be verified.
- Prefer the ROS 2 ecosystem, Gazebo, RViz, Nav2, URDF/Xacro, TF2, rosbag, and reproducible development environments when they fit the milestone.
- Keep a clear boundary between simulation and real hardware. Record relevant differences in `docs/`.
- Update `STATUS.md` after meaningful implementation or investigation. Record durable choices in `docs/decisions/`.
- Preserve an honest build-in-public record: claims must be supported by a test, log, screenshot, or demo clip.
- Do not publish externally, push Git changes, install system-wide software, or make destructive changes unless the user explicitly asks.
- Keep changes scoped. Run the relevant checks and fix failures before reporting a milestone complete.

## Teaching style

- Assume the user can program but is learning robotics.
- Define new robotics terms plainly before using them in depth.
- Explain failures as an engineer would: symptom, likely cause, diagnostic evidence, and fix.
- Avoid large unexplained code dumps. Favor small, verified increments.
