#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker info >/dev/null
evidence="evidence/milestone-2/$(date -u +%Y%m%dT%H%M%SZ)-acceptance-$$"
mkdir -p "$evidence"
docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
for run in 1 2 3; do
  mkdir -p "$evidence/run-$run"
  docker compose run --rm -T --no-deps -e "GZ_PARTITION=m2-check-$$-$run" robotics \
    python3 scripts/rover_sim.py --check --evidence "/workspace/$evidence/run-$run" \
    2>&1 | tee "$evidence/run-$run/launch.log"
done
docker compose run --rm -T --no-deps -e "GZ_PARTITION=m2-spawn-$$" robotics \
  python3 tests/check_spawn.py "/workspace/$evidence/spawn-check" 2>&1 | tee "$evidence/spawn-check.log"
docker compose run --rm -T --no-deps robotics \
  python3 tests/compare_restarts.py "/workspace/$evidence" 2>&1 | tee "$evidence/comparison.log"
printf 'Evidence: %s\n' "$evidence"
