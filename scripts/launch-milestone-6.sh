#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
workflow="${1:-localization}"
display="${2:-gui}"
map="${3:-/workspace/maps/delivery_room_v1/map.yaml}"
case "$workflow" in mapping|localization) ;; *) echo 'Expected mapping or localization' >&2; exit 2 ;; esac
case "$display" in gui|headless) ;; *) echo 'Expected gui or headless' >&2; exit 2 ;; esac
evidence="evidence/milestone-6/$(date -u +%Y%m%dT%H%M%SZ)-$workflow-$$"
mkdir -p "$evidence"
if [ "$display" = gui ]; then
  export ROVER_EVIDENCE="/workspace/$evidence" ROVER_MILESTONE=6 ROVER_M6_MODE="$workflow" ROVER_MAP="$map"
  docker image inspect hardwareless-robotics:jazzy-harmonic-desktop --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
  docker compose --profile gui up -d --force-recreate desktop
  docker compose exec -T desktop /ros_entrypoint.sh timeout 120 ros2 topic echo --no-daemon --once --qos-reliability reliable --qos-durability transient_local /map nav_msgs/msg/OccupancyGrid > "$evidence/map-ready.yaml"
  printf 'Open http://localhost:6080/vnc.html?autoconnect=true&resize=scale\nWorkflow: %s. Evidence: %s\n' "$workflow" "$evidence"
else
  docker image inspect hardwareless-robotics:jazzy-harmonic --format '{{.Id}} {{.Os}}/{{.Architecture}}' > "$evidence/image.txt"
  docker compose run --rm -T --no-deps --name rover-m6 robotics python3 scripts/rover_sim.py \
    --drive --sensors --firmware --m6 "$workflow" --map "$map" --evidence "/workspace/$evidence" 2>&1 | tee "$evidence/launch.log"
fi
