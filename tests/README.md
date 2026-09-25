# Tests

Milestone 1's integration check is `../scripts/smoke-test.sh`, backed by
`../scripts/smoke_test.py`. Run it from the repository after `docker compose build`.
It uses the real Gazebo process, ROS bridge, and ROS subscriber; no simulated test
double can satisfy the clock check. It exits nonzero on a process failure or timeout.

See `../docs/setup.md` for prerequisites and `../evidence/milestone-1/` for run logs.

Milestone 2: run `./scripts/test-milestone-2.sh` from the repository. It runs
`scripts/rover_sim.py --check` in three fresh containers, exercises separate
spawning and duplicate rejection with `check_spawn.py`, then compares observed
poses using `compare_restarts.py`. These are real physics/service checks, with
deadlines and process cleanup. Detailed acceptance tolerances are in the setup
guide. Gazebo ground truth is used only for verification, not autonomy.

Milestone 3: `./scripts/test-milestone-3.sh` runs five analytic kinematics tests
and a fresh real ROS/Gazebo integration test. It checks signed physical motion,
wheel feedback, the exact TF tree, wheel-transform rotations, same-stamp TF/odom,
publication rate and permitted estimator inputs, then verifies explicit stopping.
Gazebo pose is imported only by the acceptance probe, never the runtime estimator.
See the setup guide for the declared 10 cm / 0.20 rad nominal comparison budgets
and their limits. Failed attempts are retained in the evidence index.

`check_m3_ready.py` is a non-moving launch-readiness check. The optional
`check_keyboard_teleop.py <output.json>` observes a human/browser `i` then `k`
sequence, verifying the Twist messages and resulting movement/stop; run it inside
the desktop container. It does not generate commands. The GUI acceptance uses
`check_milestone_3.py --existing --evidence <new-directory>` and does move the rover.

M4: `./scripts/test-milestone-4.sh` exercises sensor contracts, scene observations,
IMU statistics, discrete wheel counts, recording and isolated actual replay.
`test_sensor_models.py` independently bounds quantization error. See setup and
`evidence/milestone-4/README.md` for the recorded runs and limitations.
