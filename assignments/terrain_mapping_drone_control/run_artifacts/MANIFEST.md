# Assignment 3 — canonical submission trials

This folder holds **three labeled simulation trials** (`trial_01` … `trial_03`) collected with `scripts/collect_clean_trials.py`. Each trial directory is a complete grading bundle:

| Artifact | Role |
|----------|------|
| `terminal1_sim.txt`, `terminal2_rtabmap.txt`, `terminal3_mission.txt` | Captured stdout from the three-terminal workflow |
| `topics_*.txt`, `nodes_*.txt`, `*_info_*.txt`, `processes_*.txt` | ROS graph snapshots during collection |
| `rtabmap.db` | RTAB-Map database snapshot after the run |
| `latest_px4.ulg` | PX4 ulog (**local only** — `.ulg` is gitignored; copy from `~/PX4-Autopilot/.../log/` after each flight or rely on the collector) |
| `latest_px4_source.txt` | Absolute path to the `.ulg` file that was copied (on the machine that ran the collector) |
| `TRIAL_SUMMARY.txt` | `collect_clean_trials.py` excerpt plus artifact sizes |

**Trajectory for this submission.** The mission transcripts here stop shortly after **circle entry** (collector timeout / redirected stdout limits), but each **`latest_px4.ulg`** records the **full** SITL flight, including the circular survey and later phases. Report figures (`report/figures/trial_*_ned_xy.png`, `trial_*_altitude.png`) are generated from **`vehicle_local_position` inside the ulog** via `scripts/plot_mission_report_figures.py` (`pip install matplotlib pyulog`). Optional periodic **`TELEM`** lines in `terminal3_mission.txt` are used when present; otherwise the plot script falls back to the ulog automatically.

## Sizes (reference)

| Trial | `rtabmap.db` | `latest_px4.ulg` (local) |
|-------|--------------|---------------------------|
| `trial_01` | 610304 B | ~99 MB |
| `trial_02` | 626688 B | ~99 MB |
| `trial_03` | 643072 B | ~108 MB |

See each `TRIAL_SUMMARY.txt` for `status=` (`completed` vs `timeout` / `artifacts_archived`) and mission-line excerpts.
