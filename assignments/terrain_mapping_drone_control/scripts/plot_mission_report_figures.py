#!/usr/bin/env python3
"""Build publication figures from captured sim logs (no synthetic data).

Reads ``terminal3_mission.txt`` under ``run_artifacts/trial_XX/`` and extracts:
  - ``TELEM state=… pos=(x,y,z)`` — full-mission PX4 NED samples (every ~2 s in flight)
  - If there are no TELEM lines but ``latest_px4.ulg`` exists: ``vehicle_local_position``
    x,y,z from the ulog (downsampled; full circular path).
  - Legacy fallback: ``State ARM_TAKEOFF … pos=…`` (older logs; takeoff segment only)
  - Vision HOVER lines: ``[Cylinder Dimensions] Width=… Height=…``

Optional dependency for ulog plots: ``pip install pyulog``.

Writes PNGs into ``report/figures/`` for inclusion in HW3_final_report.tex.

Usage (from repo clone, any OS with matplotlib):

    python3 assignments/terrain_mapping_drone_control/scripts/plot_mission_report_figures.py

Or after symlink into ros2_ws/src:

    python3 ~/ros2_ws/src/terrain_mapping_drone_control/scripts/plot_mission_report_figures.py
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

# Repo root: …/terrain_mapping_drone_control
PKG_ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = PKG_ROOT / "run_artifacts"
FIGURES = PKG_ROOT / "report" / "figures"

# Launch/console adds optional "[process_name] " before "[INFO]".
_ROS_INFO_CORE = r"\[INFO\]\s+\[(\d+)\.(\d+)\]\s+\[cylinder_mission_node\]:"
_OPTIONAL_PROC = r"(?:\[[^\]]+\]\s+)?"

# … TELEM state=CIRCLE pos=(12.34,-1.20,-5.00)
TELEM_LINE = re.compile(
    _OPTIONAL_PROC
    + _ROS_INFO_CORE
    + r"\s+TELEM state=(\w+) pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\)"
)
# … State ARM_TAKEOFF stage=0 pos=(0.04,-0.01,-0.02) …  (legacy)
POS_LINE = re.compile(
    _OPTIONAL_PROC
    + _ROS_INFO_CORE
    + r"\s+State ARM_TAKEOFF stage=\d+ pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\)"
)
# [Cylinder Dimensions] Width=6.05 m, Height=2.96 m
CYL_LINE = re.compile(
    r"\[Cylinder Dimensions\]\s+Width=([\d.]+)\s+m,\s+Height=([\d.]+)\s+m"
)

_STATE_COLORS = {
    "ARM_TAKEOFF": "#1f77b4",
    "CIRCLE": "#2ca02c",
    "SERVO": "#ff7f0e",
    "HOVER": "#9467bd",
    "ARUCO_HOVER": "#d62728",
    "ARUCO_SELECT": "#8c564b",
    "ARUCO_MOVE": "#e377c2",
    "ARUCO_LAND": "#7f7f7f",
}


def parse_mission_log(text: str):
    """Return sorted traj: list of (t_sec, x, y, z_ned, state_label)."""
    rows = []
    for line in text.splitlines():
        m = TELEM_LINE.search(line)
        if m:
            sec, nsec = int(m.group(1)), int(m.group(2))
            t = sec + nsec * 1e-9
            st = m.group(3)
            x, y, z = float(m.group(4)), float(m.group(5)), float(m.group(6))
            rows.append((t, x, y, z, st))
            continue
        m = POS_LINE.search(line)
        if m:
            sec, nsec = int(m.group(1)), int(m.group(2))
            t = sec + nsec * 1e-9
            x, y, z = float(m.group(3)), float(m.group(4)), float(m.group(5))
            rows.append((t, x, y, z, "ARM_TAKEOFF"))
    rows.sort(key=lambda r: r[0])
    # Drop exact duplicate timestamps (shouldn't happen)
    dedup = []
    for r in rows:
        if dedup and abs(dedup[-1][0] - r[0]) < 1e-9 and dedup[-1][1:5] == r[1:5]:
            continue
        dedup.append(r)
    cyl = []
    for line in text.splitlines():
        m = CYL_LINE.search(line)
        if m:
            cyl.append((float(m.group(1)), float(m.group(2))))
    return dedup, cyl


def load_ulog_local_trajectory(ulg_path: Path, max_points: int = 5000):
    """Return ``(t_sec, x, y, z_ned, label)`` rows from PX4 ulog, or ``None``.

    Timestamps are converted to seconds relative to the first sample. Rows are
    downsampled so matplotlib stays responsive on long SITL logs.
    """
    try:
        from pyulog import ULog
    except ImportError:
        return None
    if not ulg_path.is_file():
        return None
    try:
        log = ULog(str(ulg_path))
    except Exception:
        return None
    ds = None
    for d in log.data_list:
        if d.name == "vehicle_local_position":
            ds = d
            break
    if ds is None:
        return None
    ts = ds.data["timestamp"]
    xv = ds.data["x"]
    yv = ds.data["y"]
    zv = ds.data["z"]
    n = len(xv)
    if n < 2:
        return None
    step = max(1, (n + max_points - 1) // max_points)
    t0 = float(ts[0])
    rows = []
    for i in range(0, n, step):
        t = (float(ts[i]) - t0) * 1e-6
        rows.append((t, float(xv[i]), float(yv[i]), float(zv[i]), "ULOG"))
    return rows


def plot_trial(trial_name: str, mission_path: Path, out_prefix: str) -> None:
    if not mission_path.is_file():
        print(f"skip {trial_name}: missing {mission_path}")
        return
    mission_abs = mission_path.resolve()
    text = mission_path.read_text(encoding="utf-8", errors="ignore")
    n_raw_telem = sum(
        1
        for line in text.splitlines()
        if "TELEM state=" in line and "cylinder_mission_node" in line
    )
    traj, cyl = parse_mission_log(text)
    ulg_path = mission_abs.parent / "latest_px4.ulg"
    ulg_traj = None if n_raw_telem > 0 else load_ulog_local_trajectory(ulg_path)

    if n_raw_telem > 0:
        plot_traj = traj
    elif ulg_traj:
        plot_traj = ulg_traj
    else:
        plot_traj = traj

    if len(plot_traj) < 2 and not cyl:
        print(f"skip {trial_name}: no trajectory (TELEM / ulog / ARM_TAKEOFF) and no cylinder lines")
        print(f"  read: {mission_abs}")
        return

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    FIGURES.mkdir(parents=True, exist_ok=True)
    out_dir = FIGURES.resolve()

    if n_raw_telem > 0:
        print(f"{trial_name}: parsed {n_raw_telem} TELEM lines from\n  {mission_abs}")
    elif ulg_traj:
        print(
            f"{trial_name}: no TELEM in mission log; using PX4 ulog trajectory\n"
            f"  {ulg_path.resolve()}"
        )
    elif len(traj) >= 2:
        print(
            f"WARNING {trial_name}: log has ZERO 'TELEM' lines and no readable ulog - plot is takeoff-only (legacy ARM_TAKEOFF).\n"
            f"  Fix: colcon build terrain_mapping_drone_control, rerun mission, copy fresh terminal3_mission.txt into:\n"
            f"  {mission_abs.parent}\n"
            f"  Or install pyulog and keep latest_px4.ulg next to the mission log."
        )

    # --- Horizontal trajectory (NED x vs y) ---
    if len(plot_traj) >= 2:
        t0 = plot_traj[0][0]
        ts = [p[0] - t0 for p in plot_traj]
        xs = [p[1] for p in plot_traj]
        ys = [p[2] for p in plot_traj]
        zs = [p[3] for p in plot_traj]
        states = [p[4] for p in plot_traj]
        has_telem_states = n_raw_telem > 0 and any(s != "ARM_TAKEOFF" for s in states)

        fig, ax = plt.subplots(figsize=(6.2, 5.6))
        ax.plot(xs, ys, "-", color="#aec7e8", lw=1.4, zorder=1, label="Path (time order)")
        if has_telem_states:
            for st in dict.fromkeys(states):
                ix = [i for i, s in enumerate(states) if s == st]
                if not ix:
                    continue
                col = _STATE_COLORS.get(st, "#333333")
                ax.scatter(
                    [xs[i] for i in ix],
                    [ys[i] for i in ix],
                    c=col,
                    s=36,
                    zorder=3,
                    label=st,
                    edgecolors="white",
                    linewidths=0.35,
                )
        ax.scatter([xs[0]], [ys[0]], c="green", s=120, zorder=5, marker="s", label="Start")
        ax.scatter([xs[-1]], [ys[-1]], c="red", s=120, zorder=5, marker="*", label="End")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel(r"NED $x$ (m)")
        ax.set_ylabel(r"NED $y$ (m)")
        if n_raw_telem > 0:
            src = "TELEM samples (~2 s): full mission" if has_telem_states else (
                "TELEM: ARM_TAKEOFF samples only"
            )
        elif ulg_traj:
            src = "PX4 ulog vehicle_local_position (downsampled)"
        else:
            src = "Legacy log: ARM_TAKEOFF only — add TELEM or ulog"
        ax.set_title(f"{trial_name}: PX4 local horizontal path\n({src})")
        ax.grid(True, alpha=0.35)
        ax.legend(loc="best", fontsize=7, ncol=2)
        fig.tight_layout()
        fig.savefig(FIGURES / f"{out_prefix}_ned_xy.png", dpi=160)
        plt.close(fig)
        print(f"  wrote: {out_dir / (out_prefix + '_ned_xy.png')}")

        # --- Altitude profile: "up" = -z in NED ---
        alt = [-z for z in zs]
        fig, ax = plt.subplots(figsize=(6.4, 3.8))
        ax.plot(ts, alt, color="#ff7f0e", lw=1.8)
        ax.set_xlabel("Time from first sample (s)")
        ax.set_ylabel(r"Altitude $-z_{\mathrm{NED}}$ (m)")
        ax.set_title(f"{trial_name}: vertical profile ({src})")
        ax.grid(True, alpha=0.35)
        fig.tight_layout()
        fig.savefig(FIGURES / f"{out_prefix}_altitude.png", dpi=160)
        plt.close(fig)
        print(f"  wrote: {out_dir / (out_prefix + '_altitude.png')}")

    # --- Vision cylinder estimates from HOVER logs ---
    if cyl:
        fig, ax = plt.subplots(figsize=(6.4, 3.8))
        idx = range(1, len(cyl) + 1)
        w = [c[0] for c in cyl]
        h = [c[1] for c in cyl]
        xpos = [i - 0.2 for i in idx]
        xpos2 = [i + 0.2 for i in idx]
        ax.bar(xpos, w, width=0.38, label="Width est. (m)", color="#2ca02c")
        ax.bar(xpos2, h, width=0.38, label="Height est. (m)", color="#9467bd")
        ax.set_xticks(list(idx))
        ax.set_xticklabels([f"HOVER #{i}" for i in idx])
        ax.set_ylabel("Metres")
        ax.set_title(
            f"{trial_name}: RGB-D vision cylinder dimensions\n"
            "(Cylinder Dimensions lines from mission log)"
        )
        ax.legend()
        ax.grid(True, axis="y", alpha=0.35)
        fig.tight_layout()
        fig.savefig(FIGURES / f"{out_prefix}_cylinder_bars.png", dpi=160)
        plt.close(fig)
        print(f"  wrote: {out_dir / (out_prefix + '_cylinder_bars.png')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--artifacts",
        type=Path,
        default=ARTIFACTS,
        help="Directory containing trial_XX folders (default: …/run_artifacts)",
    )
    args = ap.parse_args()
    root = args.artifacts
    if not root.is_dir():
        print(f"No artifacts dir: {root}")
        return 1

    print("=== plot_mission_report_figures ===")
    print(f"INPUT:  mission logs under {root.resolve()}")
    print(f"        (expects trial_XX/terminal3_mission.txt; optional trial_XX/latest_px4.ulg if no TELEM)")
    print(f"OUTPUT: PNG files under {FIGURES.resolve()}")
    print()

    for trial_dir in sorted(root.glob("trial_*")):
        if not trial_dir.is_dir():
            continue
        name = trial_dir.name
        plot_trial(name, trial_dir / "terminal3_mission.txt", name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
