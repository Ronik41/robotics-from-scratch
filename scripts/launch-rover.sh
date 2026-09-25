#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mode="${1:-gui}"
evidence="evidence/milestone-2/$(date -u +%Y%m%dT%H%M%SZ)-launch-$$"
mkdir -p "$evidence"
case "$mode" in
  gui)
    docker image inspect hardwareless-robotics:jazzy-harmonic-desktop --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
    export ROVER_EVIDENCE="/workspace/$evidence"
    docker compose --profile gui up -d --force-recreate desktop
    ready=false
    for attempt in {1..120}; do
      if [ -f "$evidence/result.json" ]; then
        if grep -q 'LOADED_PAUSED' "$evidence/result.json"; then ready=true; fi
        break
      fi
      sleep 0.5
    done
    docker compose logs --no-color desktop > "$evidence/launch.log"
    if [ "$ready" != true ]; then
      printf 'Launch failed or timed out. Inspect %s\n' "$evidence" >&2
      exit 1
    fi
    printf 'Open http://localhost:6080/vnc.html?autoconnect=true&resize=scale\nScene loaded paused. Evidence: %s\nStop with: docker compose stop desktop\n' "$evidence"
    ;;
  headless)
    docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
    docker compose run --rm -T --no-deps robotics python3 scripts/rover_sim.py --evidence "/workspace/$evidence" 2>&1 | tee "$evidence/launch.log"
    ;;
  *) printf 'Usage: %s [gui|headless]\n' "$0" >&2; exit 2 ;;
esac
