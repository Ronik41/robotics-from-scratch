# Milestone 1 evidence — 2026-09-25

The initial build started without an existing ROS/Gazebo image or project build
cache. It succeeded using the pinned official ROS base and native ARM64 packages.
The smoke test then started a new disposable container and exited zero.

- [Host inspection](host-environment.txt): macOS/architecture/resources, Docker versions and backend, existing VM inventory.
- [Build log](build.log): full initial dependency install and image export.
- [Smoke stdout](20260925T183709Z-56573/result.log): Gazebo version, world identity, and PASS result.
- [Machine-readable result](20260925T183709Z-56573/result.json): 101 messages; simulated time advances 100,000,000 ns.
- [Gazebo server log](20260925T183709Z-56573/gazebo.log): shapes world initialization and physics profile.
- [Bridge log](20260925T183709Z-56573/bridge.log): Gazebo-to-ROS clock bridge and shutdown.
- [Image identity](20260925T183709Z-56573/image.txt): locally built image ID and ARM64 platform.
- [Package manifest](20260925T183709Z-56573/installed-packages.tsv): installed Debian package versions.
- [World identity](20260925T183709Z-56573/world.json): installed example path, SHA-256, world name, and model names.

The server emitted two warnings: DART uses generated meshes for ellipsoid and cone
collision geometry because those primitives are unsupported. Simulation continued
and the advancing clock crossed the ROS bridge. Raw logs retain these warnings.

Compose validation, shell syntax, and Python syntax were checked. After completion,
`docker compose ps --all` listed no remaining project containers. Graphics, camera
rendering, RViz operation, rover motion, and physical accuracy were not tested.

Reproduce with `docker compose build` and `./scripts/smoke-test.sh` from the repository.
Subsequent runs get their own timestamped directory; this directory is retained as
the initial acceptance evidence. Nothing was published externally.
