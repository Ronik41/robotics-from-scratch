#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mode="${1:-gui}"
evidence="evidence/milestone-5/$(date -u +%Y%m%dT%H%M%SZ)-launch-$$"
mkdir -p "$evidence"
case "$mode" in
  gui)
    docker image inspect hardwareless-robotics:jazzy-harmonic-desktop --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
    export ROVER_EVIDENCE="/workspace/$evidence" ROVER_MILESTONE=5
    docker compose --profile gui up -d --force-recreate desktop
    ready=false
    for attempt in {1..120}; do
      if [ -f "$evidence/result.json" ]; then
        if grep -q 'STARTED_M5' "$evidence/result.json"; then ready=true; fi
        break
      fi
      sleep 0.5
    done
    docker compose logs --no-color desktop > "$evidence/launch.log"
    if [ "$ready" != true ]; then
      printf 'Launch failed. Inspect %s\n' "$evidence" >&2
      exit 1
    fi
    # This readiness probe sends no motion command.
    docker compose exec -T desktop /ros_entrypoint.sh timeout 60 ros2 topic echo --once /firmware/state > "$evidence/firmware-ready.yaml"
    docker compose exec -T desktop /ros_entrypoint.sh timeout 30 ros2 topic echo --once /motor/driver_state > "$evidence/driver-ready.yaml"
    printf 'Open http://localhost:6080/vnc.html?autoconnect=true&resize=scale\nM5: stamped keyboard commands expire after 0.5 simulation seconds. k brakes.\nEvidence: %s\n' "$evidence"
    ;;
  headless)
    docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
    docker compose run --rm -T --no-deps robotics python3 scripts/rover_sim.py --drive --sensors --firmware --evidence "/workspace/$evidence" 2>&1 | tee "$evidence/launch.log"
    ;;
  *) printf 'Usage: %s [gui|headless]\n' "$0" >&2; exit 2 ;;
esac
