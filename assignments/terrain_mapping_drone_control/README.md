# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze

This ROS2 package implements an autonomous drone system for geological feature detection, mapping, and analysis using an RGBD camera and PX4 SITL simulation.

## Challenge Overview

<img width="1195" height="1020" alt="image" src="https://github.com/user-attachments/assets/6e3d9610-a63a-4949-88a1-a14166a9ed50" />

Students will develop a controller for a PX4-powered drone to efficiently search, map, and analyze 3D objects in an unknown environment. The drone must map the Perseverance rover, and land on it.

### Mission Objectives
Intermediate: 
1. Search and locate the cylinder
2. Map the cylinder in 3D
3. Land safely on top of the cylinder

Advanced (extra credit): 
Execute intermediate objective, and do the following additional tasks. 
1. Search and locate the rover
2. Map the rover in 3D
3. Land safely on top of the rover

In both cases, complete mission while logging time and energy performance. 

### Evaluation Criteria (100 points)

The assignment will be evaluated based on:
- Total time taken to complete the mission
- Total energy units consumed during operation
- Accuracy of rover 3D model
- Landing precision on rover
- Performance across 3 trials

### Key Requirements

- Autonomous takeoff and search strategy implementation
- Real-time rover detection 
- Energy-conscious path planning for mapping using SLAM 
- Safe and precise landing on the rover once mapping is complete
- Robust performance across trials

## Prerequisites

- ROS2 Humble
- [px4_msgs](https://github.com/PX4/px4_msgs) cloned into `~/ros2_ws/src` and built (see below)
- PX4 SITL Simulator (Tested with PX4-Autopilot main branch 9ac03f03eb)
- RTAB-Map ROS2 package
- OpenCV
- Python 3.8+

## Repository Setup

### If you already have a fork of the course repository:

```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

### If you don't have a fork yet:

1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace

```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/terrain_mapping_drone_control .
```

### PX4 messages (`px4_msgs`)

This package depends on [px4_msgs](https://github.com/PX4/px4_msgs). If `colcon` reports that `install/px4_msgs/share/px4_msgs/package.sh` is missing, clone and build `px4_msgs` in the same workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

**Match `px4_msgs` to firmware** (required for PX4 @ `9ac03f03eb` from this course): if SITL logs show `uxrce_dds_client` **create entities failed** or ROS2 cannot arm via `/fmu/in/*`, pin messages and rebuild:

```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
chmod +x scripts/pin_px4_msgs_for_course_px4.sh
./scripts/pin_px4_msgs_for_course_px4.sh ~/ros2_ws/src/px4_msgs
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select px4_msgs --symlink-install
```

**Micro XRCE DDS agent on Ubuntu/WSL:** a snap service often binds UDP **8888** (`snap.micro-xrce-dds-agent.daemon`). This stack starts its **own** agent on **8889** (and keeps `ROS_DOMAIN_ID=0` so PX4 DDS publishers are visible). The launch file prefers **`/usr/local/bin/MicroXRCEAgent`** or a non-snap binary on `PATH`.

If you built the agent from source and see **`libmicroxrcedds_agent.so.*: cannot open shared object file`** while PX4 still runs, either:

1. **Recommended (system-wide):** from your agent build directory, install libraries and refresh the loader cache:

```bash
cd ~/Micro-XRCE-DDS-Agent/build   # or wherever you cloned it
sudo cmake --install .   # or: sudo make install
sudo ldconfig
```

2. **No sudo:** `cylinder_landing.launch.py` prepends **`$HOME/Micro-XRCE-DDS-Agent/build`** (and the same path under `~/ros2_ws/src/terrain_mapping_drone_control/scripts/...`) to **`LD_LIBRARY_PATH`** before starting the agent. Override with **`export MICRO_XRCE_DDS_LIB_DIR=/path/to/dir/containing/libmicroxrcedds_agent.so`** if your build lives elsewhere.

Set **`MICRO_XRCE_DDS_AGENT`** to a full path to force a specific binary. If you must use the snap agent only, expect possible uXRCE instability; `sudo snap stop micro-xrce-dds-agent` avoids a second agent on the machine.

### Copy PX4 Model Files

Copy the custom PX4 model files to the PX4-Autopilot folder

```bash
# Navigate to the package
cd ~/ros2_ws/src/terrain_mapping_drone_control

# Make the setup script executable
chmod +x scripts/deploy_px4_model.sh

# Run the setup script to copy model files
./scripts/deploy_px4_model.sh -p /path/to/PX4-Autopilot
```

## Building and Running

One-time (or after changing Python entry points / launch files):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to terrain_mapping_drone_control --symlink-install
source install/setup.bash
# Optional: `mission.launch.py` uses these install-space executables (must exist after build):
# ros2 pkg executables terrain_mapping_drone_control | grep -E 'auto_detect_land|aruco_tracker'
```

**Three terminals** (same machine, same `ROS_DOMAIN_ID`; default **`0`** via `scripts/source_assignment3_env.sh` — must match PX4 uXRCE / Micro XRCE DDS).

### Typical workflow (three terminals)

1. **Terminal 2 — RTAB-Map + TF** — run `terminal2_rtabmap.sh` so `/drone/rtab_odom` and the map/database update while you fly the mission trajectory.
2. **Terminal 3 — mission** (if not already) — `aruco_tracker` + `auto_detect_land`: search, cylinder logic, hover/measure, then ArUco-assisted landing flow.
3. **Tune** — HSV / detection thresholds in `auto_detect_land.py`, marker layout for `aruco_tracker.py`, and mission altitudes/states per `MISSION_AND_SLAM.md`.
4. **Deliverables** — log **time** and **battery** deltas the nodes print at the end of a run; for extra credit export a mesh from RTAB-Map (see MISSION_AND_SLAM.md).

Clean stale SITL/agent if the sim misbehaves:

```bash
killall -9 px4 MicroXRCEAgent micro-xrce-dds-agent 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
fuser -k 8889/udp 2>/dev/null || true
```

**Terminal 1 — sim + Micro XRCE + Gazebo bridge**

```bash
source ~/ros2_ws/src/terrain_mapping_drone_control/scripts/source_assignment3_env.sh
bash ~/ros2_ws/src/terrain_mapping_drone_control/scripts/terminal1_sim.sh
```

**Terminal 2 — RTAB-Map + TF (after the world is up)**

```bash
source ~/ros2_ws/src/terrain_mapping_drone_control/scripts/source_assignment3_env.sh
bash ~/ros2_ws/src/terrain_mapping_drone_control/scripts/terminal2_rtabmap.sh
```

**Terminal 3 — mission (`aruco_tracker` + `auto_detect_land` via install executables)**

```bash
source ~/ros2_ws/src/terrain_mapping_drone_control/scripts/source_assignment3_env.sh
bash ~/ros2_ws/src/terrain_mapping_drone_control/scripts/terminal3_mission.sh
```

Smoke test (while Terminal 1 is running). Use **`-s` / `--use-sim-time`** so rates use `/clock`:

```bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/src/terrain_mapping_drone_control/scripts/source_assignment3_env.sh
ros2 topic hz /clock -s --window 50
ros2 topic hz /fmu/out/vehicle_local_position -s --window 50
ros2 topic hz /drone/front_rgb -s --window 30
```

Mission / perception / SLAM notes: see [MISSION_AND_SLAM.md](./MISSION_AND_SLAM.md).

**Written report (rubric mapping, trials, implementation summary):** compile `report/HW3_final_report.tex` to PDF (`pdflatex HW3_final_report.tex` from the `report/` directory). Figures under `report/figures/` come from `scripts/plot_mission_report_figures.py` run at the package root: `pip install matplotlib pyulog`, then `python3 scripts/plot_mission_report_figures.py`. The script prefers `TELEM` lines in `terminal3_mission.txt` when present; otherwise it plots `vehicle_local_position` from each `run_artifacts/trial_XX/latest_px4.ulg` (keep those `.ulg` files locally; they are gitignored).

## Extra credit -- 3D reconstruction (50 points)
Use RTAB-Map or a SLAM ecosystem of your choice to map both rocks, and export the world as a mesh file, and upload to your repo. Use git large file system (LFS) if needed. 

## License

This assignment is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0). 
For more details: https://creativecommons.org/licenses/by-nc-sa/4.0/ 
