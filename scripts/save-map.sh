#!/usr/bin/env bash
# Run INSIDE the live mapping container. Refuse overwriting a versioned map.
set -euo pipefail
prefix="${1:?Usage: scripts/save-map.sh /workspace/maps/new_version/map}"
if [ -e "$prefix.yaml" ] || [ -e "$prefix.pgm" ]; then
  echo "Map already exists: $prefix; choose a new version/directory" >&2; exit 2
fi
mkdir -p "$(dirname "$prefix")"
ros2 run nav2_map_server map_saver_cli -f "$prefix" --fmt pgm --mode trinary --free 0.196 --occ 0.65 \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
