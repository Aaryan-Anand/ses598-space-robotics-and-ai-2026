#!/bin/bash
# Terminal 1 — PX4 + Gazebo + ros_gz bridges + Micro XRCE on UDP 8889 (ROS_DOMAIN_ID=0).
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/source_assignment3_env.sh"

echo "[terminal1] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  PX4_AUTOPILOT=$PX4_AUTOPILOT"
echo "[terminal1] Stop stale SITL if you see 'PX4 server already running':  killall -9 px4 2>/dev/null; pkill -f 'gz sim' 2>/dev/null || true"

exec ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py \
  "px4_autopilot_path:=${PX4_AUTOPILOT}"
