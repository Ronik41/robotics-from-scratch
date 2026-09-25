# Build in Public Content Log

## Series

**Working title:** Learning Robotics from Scratch with Codex

**Promise:** Building a simulated delivery rover in public—sharing what works, what breaks, and why the robotics pieces exist.

## Publication rule

Only describe a feature as working when there is an attached test result, log, screenshot, or clip. Drafts are not publication authorization.

## Cadence

- 2–3 verified build updates each week.
- 1 concept explainer per week.
- An honest debugging/failure post when there is useful evidence.
- A recap after each major project phase.

## Backlog

| Status | Milestone | Post angle | Evidence needed |
| --- | --- | --- | --- |
| Draft idea | Launch | Why learn robotics without hardware? | Project diagram or folder screenshot |
| Evidence ready; unpublished | 1 | The environment decision: industry tools versus Mac friction | `evidence/milestone-1/20260925T183709Z-56573/result.log`; headless ROS clock smoke test only |
| Evidence ready; unpublished | 2 | A robot begins as a coordinate-frame and geometry problem | `evidence/milestone-2/README.md`: browser screenshot, identical poses over three starts, passive settling and duplicate rejection |
| Evidence ready; unpublished | 3 | What `tf` and wheel odometry mean, including measured drift | `evidence/milestone-3/README.md`: real RViz screenshot, keyboard commands, live TF/odom acceptance and error measurements |
| Waiting | 4 | Robots never see ground truth: sensor data and uncertainty | RViz sensor overlays |
| Waiting | 5 | The safety logic below autonomy | watchdog/e-stop demo |

## Published posts

None yet.

## 2026-09-25 — M4 sensor interfaces verified locally

Added native simulated planar LiDAR/RGB, imperfect raw IMU and explicit stamped
encoder counts. M1–M3 baselines/regressions passed. M4 checks physical observations,
message contracts and encoder motion, with actual RViz evidence and a 5.878 s
rosbag whose sensor fields replayed unchanged. The LiDAR misses low crates because
its scan plane is above them; this is a demonstrated limitation. No autonomy,
localization or low-level safety claim. No external publication performed.

Evidence and retained failures: `evidence/milestone-4/README.md`.
