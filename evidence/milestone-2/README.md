# Milestone 2 evidence — 2026-09-25

This is local, unpublished build evidence. No driving, ROS TF/odometry, RViz,
sensor, autonomy or hardware-reliability claim is made.

| Check | Evidence |
| --- | --- |
| Milestone 1 recheck before implementation | [101 clock messages, PASS](../milestone-1/20260925T184049Z-57465/result.json) |
| Container desktop image build | [Build log](desktop-build.log) |
| Known Gazebo example visible before rover implementation | [Browser screenshot](gui-baseline/browser.png), [renderer](gui-baseline/renderer.txt), [scene](gui-baseline/scene.pbtxt), [launch log](gui-baseline/gazebo.log) |
| Final three fresh-container comparisons | [Result](20260925T185315Z-acceptance-58783/result.json) |
| Standalone spawn and duplicate rejection | [Result](20260925T185315Z-acceptance-58783/spawn-check/result.json) |
| Final visible rover launch | [Result](20260925T184938Z-launch-58500/result.json), [server log](20260925T184938Z-launch-58500/gazebo.log), [GUI log](20260925T184938Z-launch-58500/gui.log), [supervisor log](20260925T184938Z-launch-58500/launch.log) |
| Localhost-only published port and container identity | [Compose state](20260925T184938Z-launch-58500/compose-state.json), [image](20260925T184938Z-launch-58500/image.txt) |
| Actual rendered scene | [Browser screenshot](20260925T184938Z-launch-58500/gazebo-browser.png) |

Initial pose is `[-2.5, -1.5, 0.25]` metres, identity orientation. After exactly
2000 one-millisecond physics steps, body height is approximately `0.239999993 m`.
Initial and settled pose component differences across the three restarts are
both zero at the recorded precision. See each run's `initial-scene.pbtxt`,
`settled.json`, `spawn-request.txt`, `spawn-response.txt`, generated URDF/SDF,
`inputs.json`, package manifest and server log for the underlying observations.
The snapshot source hashes tie the observed results to the tested assets.

## Debugging record

- The first acceptance attempt, `20260925T184715Z-acceptance-58131`, passed twice,
  then failed on the world-control service timeout in run 3. The scene and rover
  existed in the server log; the likely cause was service discovery/request timing.
  The launcher now waits for each named service and gives world-control requests
  ten seconds. It never retries an ambiguous stepping mutation.
- The subsequent complete comparison in `20260925T184837Z-acceptance-58333` passed.
  The final run above additionally verifies the standalone spawn command and
  duplicate guard, and records the final camera configuration.
- `20260925T184808Z-launch-58275` exposed a GUI anchor-name mismatch. The viewport
  title now matches the anchors. The final GUI retains a WorldStats QML layout
  warning and software antialiasing fallback; neither prevents rendering. The
  checker obtains timing from Gazebo messages, not GUI text.
- `duplicate-spawn-check/result.json` intentionally reports FAIL: it is an early
  negative test proving an existing rover is rejected. The final spawn-check
  result explicitly combines successful first spawn and expected duplicate failure.

The screenshot is a capture of the real Gazebo GUI viewed through noVNC, not a
rendered illustration. CPU software rendering was sufficient for inspection of
this small paused scene; frame rate and camera throughput were not benchmarked.
