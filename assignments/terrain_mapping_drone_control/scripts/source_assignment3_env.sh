#!/bin/bash
# Source before ros2 launch/run for assignment 3 (must match stack_constants.py).
#
# Usage:
#   source /path/to/terrain_mapping_drone_control/scripts/source_assignment3_env.sh
#
# Optional overrides:
#   export ROS2_WS=~/ros2_ws
#   export PX4_AUTOPILOT=~/PX4-Autopilot

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
# Use $HOME (not ~) so paths work when passed to ros2 launch arguments.
PX4_AUTOPILOT="${PX4_AUTOPILOT:-$HOME/PX4-Autopilot}"
PX4_AUTOPILOT="${PX4_AUTOPILOT/#\~/$HOME}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$ROS2_WS/install/setup.bash"
fi

export PX4_AUTOPILOT
