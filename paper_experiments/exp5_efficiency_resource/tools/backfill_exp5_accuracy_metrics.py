#!/usr/bin/env python3
"""Backfill Exp.5 accuracy/completeness metrics into an existing summary CSV.

The Exp.5 runner originally focused on efficiency/resource metrics. This helper
keeps those columns and adds ATE/RPE/completeness columns from saved trajectories.
It is intended to run on the Ubuntu ROS machine where rosbag/evo are available.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Iterable


ACCURACY_COLUMNS = [
    "ate_rmse_m",
    "rpe_rmse_m",
    "completeness_percent",
    "trajectory_poses",
    "input_frames",
]


def read_pose_rows(path: Path) -> list[tuple[float, list[str]]]:
    rows: list[tuple[float, list[str]]] = []
    if not path.is_file():
        return rows
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            if abs(t) > 1e12:
                t *= 1e-9
            if math.isfinite(t):
                rows.append((t, parts))
    return rows


def write_tum_seconds(src: Path, dst: Path) -> bool:
    rows = read_pose_rows(src)
    if not rows:
        return False
    with dst.open("w") as f:
        for t, p in rows:
            f.write(f"{t:.9f} {' '.join(p[1:8])}\n")
    return True


def match_tum_by_first_timestamp(gt: Path, est: Path, gt_out: Path, est_out: Path) -> int:
    gt_rows = read_pose_rows(gt)
    est_rows = read_pose_rows(est)
    if len(gt_rows) < 3 or len(est_rows) < 3:
        return 0
    gt_rows.sort(key=lambda x: x[0])
    est_rows.sort(key=lambda x: x[0])
    gt_times = [r[0] for r in gt_rows]
    offset = gt_times[0] - est_rows[0][0]
    matches: list[tuple[float, list[str], list[str]]] = []
    for et, ep in est_rows:
        target = et + offset
        j = bisect.bisect_left(gt_times, target)
        cand = []
        if j < len(gt_rows):
            cand.append(j)
        if j > 0:
            cand.append(j - 1)
        if not cand:
            continue
        best = min(cand, key=lambda i: abs(gt_times[i] - target))
        if abs(gt_times[best] - target) <= 0.05:
            matches.append((et, gt_rows[best][1], ep))
    if len(matches) < 3:
        return 0
    with gt_out.open("w") as fg, est_out.open("w") as fe:
        for stamp, g, e in matches:
            fg.write(f"{stamp:.9f} {' '.join(g[1:8])}\n")
            fe.write(f"{stamp:.9f} {' '.join(e[1:8])}\n")
    return len(matches)


def extract_rmse(text: str) -> str:
    match = re.search(r"^\s*rmse\s+([0-9eE+\-.]+)", text, re.MULTILINE)
    return match.group(1) if match else ""


def run_cmd(cmd: list[str], timeout: int) -> tuple[str, str, int]:
    proc = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    return proc.stdout, proc.stdout, proc.returncode


def trajectory_span(path: Path) -> tuple[int, float]:
    rows = read_pose_rows(path)
    if len(rows) < 2:
        return len(rows), 0.0
    times = [r[0] for r in rows]
    return len(rows), max(times) - min(times)


def rosbag_duration_and_count(bag: Path, topic: str, start: float, duration: float) -> tuple[float, int]:
    code = r'''
import rosbag
import sys
bag_path, topic, start_s, duration_s = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
with rosbag.Bag(bag_path) as bag:
    begin = bag.get_start_time() + max(0.0, start_s)
    end = bag.get_end_time() if duration_s <= 0 else min(bag.get_end_time(), begin + duration_s)
    played = max(0.0, end - begin)
    n = 0
    for _, _, t in bag.read_messages(topics=[topic]):
        ts = t.to_sec()
        if begin <= ts <= end:
            n += 1
print(f"{played:.9f},{n}")
'''
    proc = subprocess.run(
        [sys.executable, "-c", code, str(bag), topic, str(start), str(duration)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        return 0.0, 0
    played_s, count_s = proc.stdout.strip().split(",", 1)
    return float(played_s), int(count_s)


def topic_for(dataset: str, sensor: str) -> str:
    if dataset == "harbor":
        return "/camera/image_raw"
    if dataset == "euroc":
        return "/cam0/image_raw"
    if dataset == "aquaticvision":
        return "/davis_left/image_raw"
    return ""


def infer_gt(dataset: str, sequence: str, bag: Path) -> Path | None:
    dataset = dataset.lower()
    if dataset == "euroc":
        return bag.parent.parent / "GT" / f"{sequence}.csv"
    if dataset == "harbor":
        m = re.search(r"harbor_sequence_(\d+)", sequence)
        if not m:
            return None
        return bag.parent.parent / "GT" / f"harbor_colmap_traj_sequence_{int(m.group(1)):02d}.txt"
    if dataset == "aquaticvision":
        root = bag.parents[2] if len(bag.parents) >= 3 else bag.parent
        seq_dirs = sorted((root / "data").glob(f"{sequence}_*")) if (root / "data").is_dir() else []
        if seq_dirs:
            return seq_dirs[0] / "groundtruth" / "gt.tum"
    return None


def evo_metrics(dataset: str, sensor: str, gt: Path, traj: Path, out_dir: Path, timeout: int) -> tuple[str, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    if not gt.is_file() or not traj.is_file() or shutil.which("evo_ape") is None or shutil.which("evo_rpe") is None:
        (out_dir / "evo_status.txt").write_text("EVO_SKIP missing gt, trajectory, or evo tools\n")
        return "", ""

    dataset_l = dataset.lower()
    if dataset_l == "euroc":
        est = out_dir / "trajectory_sec.txt"
        if not write_tum_seconds(traj, est):
            (out_dir / "evo_status.txt").write_text("EVO_SKIP trajectory conversion failed\n")
            return "", ""
        fmt = "euroc"
        ape_cmd = ["evo_ape", fmt, str(gt), str(est), "-a", "--t_max_diff", "0.05", "--no_warnings"]
        rpe_cmd = ["evo_rpe", fmt, str(gt), str(est), "-a", "--t_max_diff", "0.05", "--no_warnings"]
        if sensor == "mono":
            ape_cmd.insert(5, "-s")
            rpe_cmd.insert(5, "-s")
    else:
        est_sec = out_dir / "trajectory_sec.tum"
        gt_matched = out_dir / "groundtruth_matched.tum"
        est_matched = out_dir / "estimated_matched.tum"
        if not write_tum_seconds(traj, est_sec):
            (out_dir / "evo_status.txt").write_text("EVO_SKIP trajectory conversion failed\n")
            return "", ""
        matches = match_tum_by_first_timestamp(gt, est_sec, gt_matched, est_matched)
        if matches < 3:
            (out_dir / "evo_status.txt").write_text(f"EVO_SKIP timestamp matching failed matches={matches}\n")
            return "", ""
        ape_cmd = ["evo_ape", "tum", str(gt_matched), str(est_matched), "-a", "-s", "--t_max_diff", "0.05", "--no_warnings"]
        rpe_cmd = ["evo_rpe", "tum", str(gt_matched), str(est_matched), "-a", "-s", "--t_max_diff", "0.05", "--no_warnings"]

    try:
        ape_out, _, _ = run_cmd(ape_cmd, timeout)
        rpe_out, _, _ = run_cmd(rpe_cmd, timeout)
    except subprocess.TimeoutExpired:
        (out_dir / "evo_status.txt").write_text("EVO_SKIP timed out\n")
        return "", ""
    (out_dir / "evo_ape.txt").write_text(ape_out)
    (out_dir / "evo_rpe.txt").write_text(rpe_out)
    (out_dir / "evo_status.txt").write_text("EVO_DONE\n")
    return extract_rmse(ape_out), extract_rmse(rpe_out)


def insert_accuracy_columns(fieldnames: list[str]) -> list[str]:
    existing = [f for f in fieldnames if f not in ACCURACY_COLUMNS]
    if "status" not in existing:
        return existing + ACCURACY_COLUMNS
    idx = existing.index("status") + 1
    return existing[:idx] + ACCURACY_COLUMNS + existing[idx:]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("summary_csv", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--bag-start", type=float, default=0.0)
    parser.add_argument("--bag-duration", type=float, default=0.0)
    parser.add_argument("--evo-timeout", type=int, default=300)
    args = parser.parse_args()

    out = args.out
    if out is None:
        out = args.summary_csv.with_name(args.summary_csv.stem + "_with_accuracy.csv")

    with args.summary_csv.open(newline="") as f:
        rows = list(csv.DictReader(f))
        if not rows:
            raise SystemExit("empty summary csv")
        fieldnames = insert_accuracy_columns(list(rows[0].keys()))

    for row in rows:
        dataset = row.get("dataset", "").lower()
        sequence = row.get("sequence", "")
        sensor = row.get("sensor", "")
        result_dir = Path(row.get("result_dir", ""))
        traj = result_dir / "trajectory.txt"
        bag = Path(row.get("bag", ""))
        topic = topic_for(dataset, sensor)

        poses, span = trajectory_span(traj)
        played, frames = rosbag_duration_and_count(bag, topic, args.bag_start, args.bag_duration) if bag.is_file() and topic else (0.0, 0)
        completeness = f"{max(0.0, min(100.0, 100.0 * span / max(played, 1e-9))):.3f}" if played > 0 else ""

        gt = infer_gt(dataset, sequence, bag) if bag else None
        ate = ""
        rpe = ""
        if gt is not None:
            ate, rpe = evo_metrics(dataset, sensor, gt, traj, result_dir / "evo_backfill", args.evo_timeout)

        row["ate_rmse_m"] = ate
        row["rpe_rmse_m"] = rpe
        row["completeness_percent"] = completeness
        row["trajectory_poses"] = str(poses)
        row["input_frames"] = str(frames) if frames else ""

    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"output={out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
