#!/bin/bash
# Terminal 2 — RTAB-Map + odom_to_camera_tf (start after Terminal 1 world is up).
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/source_assignment3_env.sh"

echo "[terminal2] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (must match terminal 1)"

exec ros2 launch terrain_mapping_drone_control rtabmap.launch.py
