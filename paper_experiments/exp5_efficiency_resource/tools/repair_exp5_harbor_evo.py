#!/usr/bin/env python3
"""Repair Harbor evo outputs in an existing Exp.5 result tree.

Harbor COLMAP ground truth files use a sparse-looking timestamp axis such as
0, 5, 10, ... while the earlier successful Harbor evaluation normalized that
axis to 0, 0.25, 0.50, ... with the same pose rows. This helper reproduces that
normalization, rematches saved trajectories, reruns evo, and writes a corrected
summary CSV without rerunning SLAM.
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


DEFAULT_GT_TIME_SCALE = 0.05
MAX_MATCH_DT = 0.05


def read_tum(path: Path) -> list[tuple[float, list[str]]]:
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
                rows.append((t, parts[:8]))
    rows.sort(key=lambda row: row[0])
    return rows


def write_tum(rows: list[tuple[float, list[str]]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for t, parts in rows:
            f.write(f"{t:.9f} {' '.join(parts[1:8])}\n")


def normalize_harbor_gt(src: Path, dst: Path, gt_time_scale: float) -> list[tuple[float, list[str]]]:
    rows = read_tum(src)
    if not rows:
        raise RuntimeError(f"empty or invalid Harbor GT: {src}")
    norm = [(t * gt_time_scale, parts) for t, parts in rows]
    write_tum(norm, dst)
    return norm


def normalize_estimate_to_sec(src: Path, dst: Path) -> list[tuple[float, list[str]]]:
    rows = read_tum(src)
    if not rows:
        raise RuntimeError(f"empty or invalid trajectory: {src}")
    write_tum(rows, dst)
    return rows


def match_by_first_timestamp(
    gt_rows: list[tuple[float, list[str]]],
    est_rows: list[tuple[float, list[str]]],
    gt_out: Path,
    est_out: Path,
    stdout_path: Path,
) -> int:
    if len(gt_rows) < 3 or len(est_rows) < 3:
        raise RuntimeError(f"not enough rows for matching: gt={len(gt_rows)} est={len(est_rows)}")

    gt_times = [row[0] for row in gt_rows]
    offset = gt_times[0] - est_rows[0][0]
    matches: list[tuple[float, list[str], list[str], float]] = []
    used_gt: set[int] = set()

    for et, ep in est_rows:
        target = et + offset
        j = bisect.bisect_left(gt_times, target)
        candidates: list[int] = []
        if j < len(gt_rows):
            candidates.append(j)
        if j > 0:
            candidates.append(j - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(gt_times[i] - target))
        dt = abs(gt_times[best] - target)
        if dt <= MAX_MATCH_DT:
            matches.append((et, gt_rows[best][1], ep, dt))
            used_gt.add(best)

    if len(matches) < 3:
        raise RuntimeError(f"not enough matches {len(matches)}")

    with gt_out.open("w") as fg, est_out.open("w") as fe:
        for stamp, gp, ep, _ in matches:
            fg.write(f"{stamp:.9f} {' '.join(gp[1:8])}\n")
            fe.write(f"{stamp:.9f} {' '.join(ep[1:8])}\n")

    mean_dt = sum(dt for *_, dt in matches) / len(matches)
    stdout_path.write_text(
        "\n".join(
            [
                f"matches={len(matches)}",
                f"unique_gt_matches={len(used_gt)}",
                f"offset={offset:.9f}",
                f"mean_abs_dt={mean_dt:.9f}",
                f"max_abs_dt={max(dt for *_, dt in matches):.9f}",
            ]
        )
        + "\n"
    )
    return len(matches)


def run_cmd(cmd: list[str], out: Path, timeout: int) -> int:
    proc = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    out.write_text(proc.stdout)
    return proc.returncode


def extract_rmse(path: Path) -> str:
    if not path.is_file():
        return ""
    for line in path.read_text(errors="ignore").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "rmse":
            return parts[1]
    return ""


def run_evo(gt: Path, est: Path, evo_dir: Path, timeout: int, plot_modes: list[str]) -> tuple[str, str]:
    missing = [tool for tool in ("evo_ape", "evo_rpe", "evo_traj") if shutil.which(tool) is None]
    if missing:
        (evo_dir / "evo_status.txt").write_text(f"EVO_SKIP missing tools: {' '.join(missing)}\n")
        return "", ""

    status = ["EVO_RUNNING"]
    try:
        ape_cmd = [
            "evo_ape",
            "tum",
            str(gt),
            str(est),
            "-a",
            "--align",
            "--correct_scale",
            "-s",
            "-v",
            "--save_results",
            str(evo_dir / "evo_ape.zip"),
            "--no_warnings",
        ]
        rpe_cmd = [
            "evo_rpe",
            "tum",
            str(gt),
            str(est),
            "-a",
            "--align",
            "--correct_scale",
            "-s",
            "-v",
            "-r",
            "trans_part",
            "-d",
            "1",
            "-u",
            "f",
            "--save_results",
            str(evo_dir / "evo_rpe.zip"),
            "--no_warnings",
        ]

        # Keep the conventional xy filenames for downstream scripts, and add
        # xz/yz plots so Harbor's COLMAP axis choice can be inspected.
        for idx, mode in enumerate(plot_modes):
            ape_out = evo_dir / ("evo_ape.txt" if idx == 0 else f"evo_ape_{mode}.txt")
            rpe_out = evo_dir / ("evo_rpe.txt" if idx == 0 else f"evo_rpe_{mode}.txt")
            traj_out = evo_dir / ("evo_traj.txt" if idx == 0 else f"evo_traj_{mode}.txt")
            run_cmd(
                ape_cmd + ["--save_plot", str(evo_dir / f"evo_ape_plot_{mode}.pdf"), "--plot_mode", mode],
                ape_out,
                timeout,
            )
            run_cmd(
                rpe_cmd + ["--save_plot", str(evo_dir / f"evo_rpe_plot_{mode}.pdf"), "--plot_mode", mode],
                rpe_out,
                timeout,
            )
            run_cmd(
                [
                    "evo_traj",
                    "tum",
                    str(est),
                    "--ref",
                    str(gt),
                    "--align",
                    "--correct_scale",
                    "--save_plot",
                    str(evo_dir / f"evo_traj_plot_{mode}.pdf"),
                    "--plot_mode",
                    mode,
                    "--no_warnings",
                ],
                traj_out,
                timeout,
            )
        status.append("EVO_DONE")
    except subprocess.TimeoutExpired:
        status.append("EVO_TIMEOUT")
    finally:
        (evo_dir / "evo_status.txt").write_text("\n".join(status) + "\n")

    return extract_rmse(evo_dir / "evo_ape.txt"), extract_rmse(evo_dir / "evo_rpe.txt")


def tag_from_result_root(result_root: Path) -> str:
    return result_root.name


def default_summary_for(result_root: Path) -> Path:
    exp_dir = result_root.parent.parent
    return exp_dir / "processed_results" / f"exp5_all_runtime_metrics_{tag_from_result_root(result_root)}.csv"


def result_dir_to_path(value: str, ws: Path | None) -> Path:
    p = Path(value)
    if p.is_absolute() or ws is None:
        return p
    return ws / p


def infer_harbor_gt(harbor_root: Path, sequence: str) -> Path:
    match = re.search(r"harbor_sequence_(\d+)", sequence)
    if not match:
        raise RuntimeError(f"cannot infer Harbor sequence number from {sequence}")
    return harbor_root / "GT" / f"harbor_colmap_traj_sequence_{int(match.group(1)):02d}.txt"


def process_run(
    row: dict[str, str],
    ws: Path | None,
    harbor_root: Path,
    timeout: int,
    gt_time_scale: float,
    plot_modes: list[str],
) -> tuple[str, str, int]:
    run_dir = result_dir_to_path(row["result_dir"], ws)
    traj = run_dir / "trajectory.txt"
    evo_dir = run_dir / "evo"
    evo_dir.mkdir(parents=True, exist_ok=True)

    gt_src = infer_harbor_gt(harbor_root, row["sequence"])
    gt_norm = evo_dir / "groundtruth_norm.tum"
    traj_sec = evo_dir / "trajectory_sec.tum"
    gt_matched = evo_dir / "groundtruth_matched.tum"
    est_matched = evo_dir / "estimated_matched.tum"

    gt_rows = normalize_harbor_gt(gt_src, gt_norm, gt_time_scale)
    est_rows = normalize_estimate_to_sec(traj, traj_sec)
    matches = match_by_first_timestamp(
        gt_rows,
        est_rows,
        gt_matched,
        est_matched,
        evo_dir / "match_timestamps_stdout.txt",
    )
    ate, rpe = run_evo(gt_matched, est_matched, evo_dir, timeout, plot_modes)
    return ate, rpe, matches


def update_evo_plot_list(result_root: Path) -> None:
    plots = sorted(str(path) for path in result_root.rglob("*plot_*.pdf"))
    (result_root / "evo_plots.txt").write_text("\n".join(plots) + ("\n" if plots else ""))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_root", type=Path, help="Exp.5 raw result root, e.g. raw_results/exp5_harbor_...")
    parser.add_argument("--summary", type=Path, help="Input summary CSV. Defaults from result_root name.")
    parser.add_argument("--out-summary", type=Path, help="Output summary CSV. Defaults to *_harbor_evo_fixed.csv.")
    parser.add_argument("--harbor-root", type=Path, default=Path(os.environ.get("HARBOR_ROOT", "~/catkin_ws/dataset_harbor")).expanduser())
    parser.add_argument("--workspace", type=Path, default=Path(os.environ.get("WS", "~/catkin_ws")).expanduser())
    parser.add_argument("--gt-time-scale", type=float, default=DEFAULT_GT_TIME_SCALE)
    parser.add_argument("--evo-timeout", type=int, default=300)
    parser.add_argument("--plot-modes", nargs="+", default=["xy", "xz", "yz"], choices=["xy", "xz", "yz"])
    parser.add_argument("--in-place-summary", action="store_true", help="Overwrite the input summary CSV.")
    args = parser.parse_args()

    result_root = args.result_root
    summary = args.summary or default_summary_for(result_root)
    out_summary = args.out_summary
    if args.in_place_summary:
        out_summary = summary
    elif out_summary is None:
        out_summary = summary.with_name(summary.stem + "_harbor_evo_fixed.csv")

    if not summary.is_file():
        raise SystemExit(f"summary not found: {summary}")
    if not result_root.is_dir():
        raise SystemExit(f"result root not found: {result_root}")

    with summary.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []
    if not rows:
        raise SystemExit(f"empty summary: {summary}")

    processed = 0
    for row in rows:
        if row.get("dataset", "").lower() != "harbor":
            continue
        if not row.get("result_dir"):
            continue
        try:
            ate, rpe, matches = process_run(
                row,
                args.workspace,
                args.harbor_root,
                args.evo_timeout,
                args.gt_time_scale,
                args.plot_modes,
            )
        except Exception as exc:  # Keep processing other runs.
            run_dir = result_dir_to_path(row.get("result_dir", ""), args.workspace)
            evo_dir = run_dir / "evo"
            evo_dir.mkdir(parents=True, exist_ok=True)
            (evo_dir / "evo_status.txt").write_text(f"EVO_REPAIR_FAILED {exc}\n")
            print(f"[WARN] {row.get('sequence')} {row.get('method')} run_{row.get('run_id')}: {exc}", file=sys.stderr)
            continue
        row["ate_rmse_m"] = ate
        row["rpe_rmse_m"] = rpe
        processed += 1
        print(
            f"[OK] {row.get('sequence')} {row.get('sensor')} {row.get('method')} "
            f"run={row.get('run_id')} matches={matches} ATE={ate} RPE={rpe}"
        )

    with out_summary.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    update_evo_plot_list(result_root)
    print(f"processed={processed}")
    print(f"summary={out_summary}")
    print(f"evo_plots={result_root / 'evo_plots.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
