#!/usr/bin/env bash
# Install prerequisites for terrain_mapping_drone_control on Ubuntu 22.04 (WSL2).
# Run inside WSL: bash scripts/install_wsl_prerequisites.sh
#
# Requires: sudo (you will be prompted for your password).
# Optional: INSTALL_PX4_NUTTX=0  →  pass --no-nuttx to PX4 ubuntu.sh (faster if you only need SITL).

set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

info() { echo "[prereqs] $*"; }

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot read /etc/os-release"; exit 1
fi
# shellcheck source=/dev/null
. /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "This script targets Ubuntu 22.04 (ROS 2 Humble). You have: ${PRETTY_NAME:-unknown}"
  exit 1
fi

info "Updating apt metadata..."
sudo apt-get update -y -qq

info "Ensuring universe repository (ROS 2 dependency)..."
sudo apt-get install -y -qq software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update -y -qq

info "Base packages..."
sudo apt-get install -y -qq \
  curl gnupg lsb-release git wget \
  build-essential python3-pip python3-venv

# --- ROS 2 Humble ---
if [[ -f /opt/ros/humble/setup.bash ]]; then
  info "ROS 2 Humble already installed."
else
  info "Adding ROS 2 apt repository..."
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update -y -qq
  info "Installing ros-humble-desktop and dev tools (large download)..."
  sudo apt-get install -y -qq ros-humble-desktop ros-dev-tools
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
  fi
  rosdep update || true
fi

# --- RTAB-Map (ROS 2), OpenCV, Python helpers ---
info "Installing RTAB-Map ROS 2, OpenCV, and common Python packages..."
sudo apt-get install -y -qq \
  ros-humble-rtabmap-ros \
  python3-opencv \
  libopencv-dev \
  python3-numpy

# --- Gazebo Harmonic GUI on WSLg (Qt6 XCB + Mesa; fixes blank / missing window) ---
info "Installing Gazebo GUI / Qt6 platform packages..."
sudo apt-get install -y -qq \
  qt6-qpa-plugins \
  libqt6core6 libqt6gui6 libqt6widgets6 libqt6openglwidgets6 \
  libxcb-cursor0 libxcb-xinerama0 libxkbcommon-x11-0 \
  libegl1-mesa libgl1-mesa-dri mesa-utils libvulkan1 vulkan-tools

# --- PX4-Autopilot @ course-tested commit ---
PX4_COMMIT="9ac03f03eb"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [[ ! -d "$PX4_DIR/.git" ]]; then
  info "Cloning PX4-Autopilot into $PX4_DIR ..."
  git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" --recursive
else
  info "Using existing repo at $PX4_DIR"
fi

cd "$PX4_DIR"
info "Checking out $PX4_COMMIT and syncing submodules..."
git fetch origin
git checkout "$PX4_COMMIT"
git submodule sync --recursive
git submodule update --init --recursive

# Shallow or partial clones can omit NuttX tags; PX4's version header step then fails at configure time.
if [[ -d "$PX4_DIR/platforms/nuttx/NuttX/nuttx/.git" ]]; then
  info "Fetching NuttX tags (avoids px_update_git_header / IndexError on some clones)..."
  git -C "$PX4_DIR/platforms/nuttx/NuttX/nuttx" fetch --tags origin 2>/dev/null || true
fi

UBUNTU_SH_ARGS=()
if [[ "${INSTALL_PX4_NUTTX:-1}" == "0" ]]; then
  info "Skipping NuttX toolchain (INSTALL_PX4_NUTTX=0)."
  UBUNTU_SH_ARGS+=(--no-nuttx)
fi

info "Running PX4 Tools/setup/ubuntu.sh (sim + deps; may take several minutes)..."
bash ./Tools/setup/ubuntu.sh "${UBUNTU_SH_ARGS[@]}"

# ubuntu.sh runs pip against PX4's requirements.txt, which can upgrade NumPy to 2.x.
# ROS 2 Humble's cv_bridge (Boost/pybind) is built against NumPy 1.x → import/crash errors otherwise.
info "Pinning user-site NumPy to <2 for ROS Humble cv_bridge compatibility..."
python3 -m pip install --user 'numpy>=1.20,<2'

# Newer pip opencv-python wheels require NumPy 2; ROS nodes here use cv_bridge + apt python3-opencv.
if python3 -m pip show opencv-python &>/dev/null || python3 -m pip show opencv-contrib-python &>/dev/null; then
  info "Removing user-site opencv-python (incompatible with NumPy<2); apt python3-opencv remains."
  python3 -m pip uninstall -y opencv-python opencv-contrib-python 2>/dev/null || true
fi

info "Done."
echo ""
echo "Add to ~/.bashrc (if not already):"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Build SITL (first time is slow):"
echo "  cd $PX4_DIR && make px4_sitl"
echo ""
echo "Then follow README: ros2 workspace symlink, deploy_px4_model.sh, colcon build."
echo ""
echo "If Gazebo still shows no window in WSL, open a new terminal and:"
echo "  source .../terrain_mapping_drone_control/scripts/wsl_gazebo_gui_env.sh"
echo "or re-run this script (Qt6 QPA plugins are installed above)."
