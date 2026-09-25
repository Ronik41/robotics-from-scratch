#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
evidence="evidence/milestone-4/$(date -u +%Y%m%dT%H%M%SZ)-acceptance-$$"
mkdir -p "$evidence"
docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
docker compose run --rm -T --no-deps robotics python3 -m unittest discover -s tests -p 'test_sensor_models.py' -v 2>&1 | tee "$evidence/unit.log"
docker compose run --rm -T --no-deps -e "GZ_PARTITION=m4-check-$$" robotics \
  python3 tests/check_milestone_4.py --evidence "/workspace/$evidence" 2>&1 | tee "$evidence/check.log"
docker compose run --rm -T --no-deps -e ROS_DOMAIN_ID=43 robotics \
  python3 tests/check_sensor_replay.py --evidence "/workspace/$evidence" 2>&1 | tee "$evidence/replay.log"
printf 'Evidence: %s\n' "$evidence"
