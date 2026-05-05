#!/usr/bin/env bash
# Source in WSL before ros2 launch if the Gazebo window does not appear:
#   source ~/ros2_ws/src/terrain_mapping_drone_control/scripts/wsl_gazebo_gui_env.sh
# (Adjust path if your symlink differs.)

export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11
unset WAYLAND_DISPLAY
