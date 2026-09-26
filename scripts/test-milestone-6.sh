#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
evidence="evidence/milestone-6/$(date -u +%Y%m%dT%H%M%SZ)-acceptance-$$"
mkdir -p "$evidence"
docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
docker compose run --rm -T --no-deps robotics python3 -m unittest discover -s tests -p 'test_m6_contracts.py' -v > "$evidence/unit.log" 2>&1 || { cat "$evidence/unit.log"; exit 1; }
# Every rerun generates its own candidate; never overwrite the versioned map.
map="/workspace/$evidence/map/map.yaml"
for mode in mapping localization; do
  mkdir -p "$evidence/$mode"
  docker compose run --rm -T --no-deps -e "GZ_PARTITION=m6-$mode-$$" robotics \
    python3 tests/check_milestone_6.py --mode "$mode" --map "$map" --evidence "/workspace/$evidence/$mode" \
    > "$evidence/$mode/check.log" 2>&1 || { cat "$evidence/$mode/check.log"; exit 1; }
  printf '%s PASS: %s\n' "$mode" "$evidence/$mode/result.json"
done
docker compose run --rm -T --no-deps robotics python3 tests/plot_milestone_6.py "/workspace/$evidence" > "$evidence/plot.log" 2>&1
printf 'Evidence: %s\n' "$evidence"
