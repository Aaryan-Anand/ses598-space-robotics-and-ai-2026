PNG figures are generated from real run logs (not hand-drawn).

INPUT (plot script reads):
  terrain_mapping_drone_control/run_artifacts/trial_XX/terminal3_mission.txt

OUTPUT (plot script writes PNGs here):
  terrain_mapping_drone_control/report/figures/
  e.g. trial_01_ned_xy.png, trial_01_altitude.png (and trial_02, trial_03).

Optional: trial_XX_cylinder_bars.png when mission logs contain [Cylinder Dimensions] HOVER lines.

Uses TELEM lines (full mission, ~2 s) when present. If there is no TELEM, the script reads
latest_px4.ulg in the same trial folder (vehicle_local_position). Install pyulog for that path:
  pip install pyulog
Otherwise it falls back to legacy ARM_TAKEOFF-only lines in the mission log.

Command: python3 scripts/plot_mission_report_figures.py (from terrain_mapping_drone_control/)
The script also prints absolute INPUT/OUTPUT paths when it runs.
