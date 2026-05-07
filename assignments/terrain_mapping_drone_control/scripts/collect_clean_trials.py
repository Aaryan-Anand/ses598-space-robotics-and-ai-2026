#!/usr/bin/env python3
"""Run and collect three clean Assignment 3 SITL trials.

This script is intended to be run from WSL:

    python3 ~/ros2_ws/src/terrain_mapping_drone_control/scripts/collect_clean_trials.py

It creates:
    run_artifacts/trial_01
    run_artifacts/trial_02
    run_artifacts/trial_03

Each trial includes terminal logs, ROS graph snapshots, RTAB database, and latest PX4 ULG.

Environment (optional):
  HW3_TRIALS        number of trials (default 3)
  HW3_MAX_TRIAL_SEC max seconds per trial before archiving artifacts (default 1200).
                    Trial folders are always saved; status is "completed" if mission footer
                    strings appear in the log, otherwise "artifacts_archived".
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTROOT = REPO_ROOT / "run_artifacts"
SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/ros2_ws/install/setup.bash && "
    "source ~/ros2_ws/src/terrain_mapping_drone_control/scripts/source_assignment3_env.sh"
)
MAX_TRIAL_SECONDS = int(os.environ.get("HW3_MAX_TRIAL_SEC", "1200"))
NUM_TRIALS = int(os.environ.get("HW3_TRIALS", "3"))


def sh(cmd: str, timeout: int = 60, out=None):
    return subprocess.run(
        ["bash", "-lc", cmd],
        text=True,
        stdout=out or subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )


def kill_all():
    sh(
        "killall -9 px4 MicroXRCEAgent micro-xrce-dds-agent 2>/dev/null || true; "
        "pkill -f '[g]z sim' 2>/dev/null || true; "
        "pkill -f '[p]arameter_bridge' 2>/dev/null || true; "
        "pkill -f '[a]uto_detect_land' 2>/dev/null || true; "
        "pkill -f '[a]ruco_tracker' 2>/dev/null || true; "
        "pkill -f '[o]dom_to_camera_tf' 2>/dev/null || true; "
        "pkill -f '[r]tabmap_slam/rtabmap' 2>/dev/null || true; "
        "pkill -f '[r]os2 launch terrain_mapping_drone_control' 2>/dev/null || true; "
        "fuser -k 8889/udp 2>/dev/null || true; "
        "ros2 daemon stop 2>/dev/null || true",
        timeout=20,
    )


def start(cmd: str, logfile: Path):
    handle = open(logfile, "w", buffering=1)
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return proc, handle


def stop_proc(proc):
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            time.sleep(3)
        except Exception:
            pass
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def topic_list() -> str:
    try:
        return sh(f"{SETUP}; ros2 topic list --no-daemon --spin-time 3", timeout=12).stdout or ""
    except Exception:
        return ""


def node_list() -> str:
    try:
        return sh(f"{SETUP}; ros2 node list --no-daemon --spin-time 3", timeout=12).stdout or ""
    except Exception:
        return ""


def wait_for(predicate, timeout_seconds: int) -> bool:
    start_time = time.time()
    while time.time() - start_time < timeout_seconds:
        if predicate():
            return True
        time.sleep(3)
    return False


def snapshot(trial_dir: Path, tag: str):
    commands = {
        f"topics_{tag}.txt": f"{SETUP}; ros2 topic list --no-daemon --spin-time 4",
        f"nodes_{tag}.txt": f"{SETUP}; ros2 node list --no-daemon --spin-time 4",
        f"vlp_info_{tag}.txt": (
            f"{SETUP}; ros2 topic info /fmu/out/vehicle_local_position "
            "-v --no-daemon --spin-time 4"
        ),
        f"mapData_info_{tag}.txt": f"{SETUP}; ros2 topic info /mapData -v --no-daemon --spin-time 4",
        f"front_rgb_info_{tag}.txt": (
            f"{SETUP}; ros2 topic info /drone/front_rgb -v --no-daemon --spin-time 4"
        ),
        f"depth_info_{tag}.txt": (
            f"{SETUP}; ros2 topic info /drone/front_depth -v --no-daemon --spin-time 4"
        ),
        f"depth_camera_info_{tag}.txt": (
            f"{SETUP}; ros2 topic info /drone/front_depth/camera_info "
            "-v --no-daemon --spin-time 4"
        ),
    }
    for name, cmd in commands.items():
        with open(trial_dir / name, "w") as out:
            try:
                sh(cmd, timeout=20, out=out)
            except Exception as exc:
                out.write(f"ERROR: {exc}\n")
    with open(trial_dir / f"processes_{tag}.txt", "w") as out:
        sh("ps -ef", timeout=10, out=out)


def copy_outputs(trial_dir: Path):
    db = Path.home() / ".ros" / "rtabmap.db"
    if db.exists():
        shutil.copy2(db, trial_dir / "rtabmap.db")

    px4_root = Path.home() / "PX4-Autopilot"
    ulg_files = sorted(px4_root.rglob("*.ulg"), key=lambda p: p.stat().st_mtime, reverse=True)
    if ulg_files:
        shutil.copy2(ulg_files[0], trial_dir / "latest_px4.ulg")
        (trial_dir / "latest_px4_source.txt").write_text(str(ulg_files[0]) + "\n")


def summarize(trial_dir: Path, status: str):
    mission_log = trial_dir / "terminal3_mission.txt"
    mission = mission_log.read_text(errors="ignore") if mission_log.exists() else ""
    keys = [
        "WAIT_INTRINSICS",
        "vehicle_local_position",
        "Camera intrinsics",
        "Moving to ARM_TAKEOFF",
        "Arm command",
        "ARM_TAKEOFF",
        "Vertical takeoff",
        "circle entry",
        "TELEM",
        "Switching to CIRCLE",
        "Detected potential",
        "HOVER",
        "Mission complete",
        "Battery Used",
        "Mission Duration",
    ]
    footer_ok = any(
        s in mission
        for s in ("Mission complete", "Mission Duration", "Battery Used")
    )
    lines = [
        f"status={status}",
        "note=All trial artifacts are saved under this folder for grading (logs, RTAB DB, PX4 ULG).",
        f"mission_footer_strings_detected={'yes' if footer_ok else 'no'}",
        "",
        "mission_key_lines:",
    ]
    for line in mission.splitlines():
        if any(key in line for key in keys):
            lines.append(line)
    lines.append("")
    for artifact in ["rtabmap.db", "latest_px4.ulg"]:
        path = trial_dir / artifact
        size = path.stat().st_size if path.exists() else "missing"
        lines.append(f"{artifact}: {size} bytes")
    (trial_dir / "TRIAL_SUMMARY.txt").write_text("\n".join(lines) + "\n")


def main():
    print(
        f"HW3 collector: NUM_TRIALS={NUM_TRIALS} MAX_TRIAL_SECONDS={MAX_TRIAL_SECONDS}",
        flush=True,
    )
    ARTROOT.mkdir(exist_ok=True)
    latest = ARTROOT / "latest_run"
    if latest.exists():
        archive = ARTROOT / ("archived_latest_run_" + time.strftime("%Y%m%d_%H%M%S"))
        shutil.move(str(latest), str(archive))

    print("clean build...", flush=True)
    kill_all()
    latest.mkdir(exist_ok=True)
    with open(latest / "build.log", "w") as out:
        sh(
            "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && "
            "colcon build --packages-select terrain_mapping_drone_control --symlink-install",
            timeout=180,
            out=out,
        )

    for idx in range(1, NUM_TRIALS + 1):
        trial_dir = ARTROOT / f"trial_{idx:02d}"
        if trial_dir.exists():
            shutil.rmtree(trial_dir)
        trial_dir.mkdir(parents=True)

        print(f"=== trial_{idx:02d}: reset/start ===", flush=True)
        kill_all()
        time.sleep(1)

        sim_proc = sim_handle = rtab_proc = rtab_handle = mission_proc = mission_handle = None
        status = "started"
        try:
            sim_proc, sim_handle = start(
                "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && "
                "source install/setup.bash && "
                "bash ~/ros2_ws/src/terrain_mapping_drone_control/scripts/terminal1_sim.sh",
                trial_dir / "terminal1_sim.txt",
            )
            sim_ok = wait_for(
                lambda: all(
                    topic in topic_list()
                    for topic in [
                        "/clock",
                        "/drone/front_rgb",
                        "/drone/front_depth",
                        "/fmu/out/vehicle_local_position",
                    ]
                ),
                120,
            )
            snapshot(trial_dir, "after_sim")
            print(f"trial_{idx:02d}: sim_ok={sim_ok}", flush=True)
            if not sim_ok:
                status = "failed_sim_topics"
                continue

            # Let bridges + uXRCE settle before spawning many ros2 CLI snapshot processes.
            time.sleep(12)

            rtab_proc, rtab_handle = start(
                "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && "
                "source install/setup.bash && "
                "bash ~/ros2_ws/src/terrain_mapping_drone_control/scripts/terminal2_rtabmap.sh",
                trial_dir / "terminal2_rtabmap.txt",
            )
            rtab_ok = wait_for(lambda: "/rtabmap" in node_list() and "/mapData" in topic_list(), 90)
            snapshot(trial_dir, "after_rtabmap")
            print(f"trial_{idx:02d}: rtab_ok={rtab_ok}", flush=True)

            mission_proc, mission_handle = start(
                "cd ~/ros2_ws && source /opt/ros/humble/setup.bash && "
                "source install/setup.bash && export PYTHONUNBUFFERED=1 && "
                "bash ~/ros2_ws/src/terrain_mapping_drone_control/scripts/terminal3_mission.sh",
                trial_dir / "terminal3_mission.txt",
            )

            status = "artifacts_archived"
            start_time = time.time()
            while time.time() - start_time < MAX_TRIAL_SECONDS:
                time.sleep(5)
                mp = trial_dir / "terminal3_mission.txt"
                mission = mp.read_text(errors="ignore") if mp.exists() else ""
                if (
                    "Mission complete" in mission
                    or "Mission Duration" in mission
                    or "Battery Used" in mission
                ):
                    status = "completed"
                    break
                if mission_proc.poll() is not None:
                    status = f"mission_exited_{mission_proc.returncode}"
                    break
            snapshot(trial_dir, "final")
        finally:
            copy_outputs(trial_dir)
            summarize(trial_dir, status)
            print(f"trial_{idx:02d}: status={status}", flush=True)

            stop_proc(mission_proc)
            stop_proc(rtab_proc)
            stop_proc(sim_proc)
            for handle in [mission_handle, rtab_handle, sim_handle]:
                if handle:
                    handle.close()
            kill_all()
            time.sleep(2)

    print("ALL_TRIALS_DONE", flush=True)


if __name__ == "__main__":
    main()
