#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker info >/dev/null
evidence="evidence/milestone-1/$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$evidence"
docker image inspect hardwareless-robotics:jazzy-harmonic \
  --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
docker compose run --rm -T --no-deps robotics \
  python3 scripts/smoke_test.py "/workspace/$evidence" 2>&1 | tee "$evidence/result.log"
printf 'Evidence: %s\n' "$evidence"
