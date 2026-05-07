#!/bin/bash
# Pin px4_msgs to a tag compatible with course PX4-Autopilot @ 9ac03f03eb (v1.16.0-alpha lineage).
# Run in WSL: bash scripts/pin_px4_msgs_for_course_px4.sh [path/to/px4_msgs]
#
# Fixes uXRCE "create entities failed" / message_format errors when px4_msgs main is ahead of firmware.
set -euo pipefail

MSGS_DIR="${1:-$HOME/ros2_ws/src/px4_msgs}"
TAG="${PX4_MSGS_TAG:-v1.16.0}"

if [[ ! -d "$MSGS_DIR/.git" ]]; then
  echo "Not a git clone: $MSGS_DIR" >&2
  exit 1
fi

cd "$MSGS_DIR"
git fetch --tags --force
git checkout "$TAG"
echo "Checked out px4_msgs at $TAG"
echo "Rebuild: cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select px4_msgs --symlink-install"
