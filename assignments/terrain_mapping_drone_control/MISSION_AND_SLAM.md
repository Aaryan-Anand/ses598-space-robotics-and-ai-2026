# Mission, perception, and SLAM (what you must deliver)

For the **submission PDF** (implementation summary, rubric mapping, trial methodology), compile **`report/HW3_final_report.tex`**.

## Course expectations (from the assignment brief)

**Mission (autonomy)**  
- Autonomous **takeoff** and a **search** strategy that finds the target(s).  
- **Intermediate:** cylinder → **3D map** → **land on top**.  
- **Advanced:** same for the **rover** as well, with **time** and **energy** logged across **three trials**.

**Perception**  
- **Real-time detection** of the target (rover for the main story; cylinder is the staged object in this repo).  
- The starter stack uses **HSV + depth** on the front RGB-D (`auto_detect_land.py`) and **ArUco** on the down camera for landing alignment (`aruco_tracker.py`). You are expected to **tune, replace, or extend** these so detection is reliable in your runs.

**SLAM / 3D mapping**  
- **Energy-aware mapping path** (how you fly while building the map) is part of the narrative; RTAB-Map is the suggested stack.  
- **Extra credit:** export a **mesh** (and use Git LFS if large).

## What was already in the repo vs what we wired up

| Piece | Role | Status |
|--------|------|--------|
| `cylinder_landing.launch.py` | PX4 + Gazebo + ROS bridges | Use as **terminal 1** |
| `mission.launch.py` | Runs `aruco_tracker.py` + `auto_detect_land.py` | **Terminal 3** after sim is up |
| `auto_detect_land.py` | Offboard state machine + cylinder cueing + ArUco landing | Implemented mission pipeline; tune HSV/thresholds per environment |
| `aruco_tracker.py` | Down-camera ArUco → `/aruco/marker_pose` | Works if markers exist in view; **check marker size / calibration** |
| `rtabmap.launch.py` | RTAB-Map + `odom_to_camera_tf` | **Rewired** to `/drone/front_*` and `/drone/rtab_odom` (see below) |
| `odom_to_camera_tf` | `nav_msgs/Odometry` + TF `odom` → `oakd_rgb` | Integrated; aligns RGB-D frame name with RTAB’s `frame_id` |

## How to run SLAM with the sim

0. **After pulling:** run `./scripts/deploy_px4_model.sh -p ~/PX4-Autopilot` so the airframe (incl. `NAV_DLL_ACT` for SITL arming) is installed. Pin `px4_msgs` if needed: `bash scripts/pin_px4_msgs_for_course_px4.sh` (use `bash`, not `./`, on WSL if line endings were CRLF).

1. **Terminal 1 (sim):** from the package `scripts/` directory run `bash terminal1_sim.sh` (or `chmod +x terminal*.sh && ./terminal1_sim.sh`). Same as `ros2 launch … cylinder_landing` with env + PX4 path wired.

2. **Terminal 2 (RTAB-Map):** after the world is up, `bash terminal2_rtabmap.sh`

3. **Terminal 3 (mission):** `bash terminal3_mission.sh` — starts `aruco_tracker` + `auto_detect_land` on **ROS_DOMAIN_ID=0** so offboard reaches the same PX4 as terminal 1.

All three scripts `source` `scripts/source_assignment3_env.sh` (ROS Humble + workspace + `ROS_DOMAIN_ID=0`). Override paths with `export ROS2_WS=…` or `export PX4_AUTOPILOT=…` before running if needed.

Rebuild after pulling changes:  
`colcon build --packages-select terrain_mapping_drone_control --symlink-install`

## Mesh export (typical RTAB-Map workflow)

After mapping, use RTAB-Map’s tools to export a mesh from the database (see [RTAB-Map export](http://introlab.github.io/rtabmap-doc/)); commit with **Git LFS** if the file is large.

## Debug OpenCV windows (optional)

`auto_detect_land.py` only calls `cv2.imshow` when **`TERRAIN_DEBUG_CV=1`** is set in the environment.
