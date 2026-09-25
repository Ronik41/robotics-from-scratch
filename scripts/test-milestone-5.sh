#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
evidence="evidence/milestone-5/$(date -u +%Y%m%dT%H%M%SZ)-acceptance-$$"
mkdir -p "$evidence"
docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
docker compose run --rm -T --no-deps robotics python3 -m unittest discover -s tests -p 'test_motor_control.py' -v 2>&1 | tee "$evidence/unit.log"
docker compose run --rm -T --no-deps -e "GZ_PARTITION=m5-check-$$" robotics \
  python3 tests/check_milestone_5.py --evidence "/workspace/$evidence" 2>&1 | tee "$evidence/check.log"
printf 'Evidence: %s\n' "$evidence"
