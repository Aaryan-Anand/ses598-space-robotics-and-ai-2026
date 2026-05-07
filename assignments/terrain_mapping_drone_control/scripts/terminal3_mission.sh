#!/bin/bash
# Terminal 3 — aruco_tracker + auto_detect_land (start after Terminal 1; optional with terminal 2).
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/source_assignment3_env.sh"

echo "[terminal3] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (must match terminal 1)"

exec ros2 launch terrain_mapping_drone_control mission.launch.py
