#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev


NUMERIC_FIELDS = [
    "ate_rmse_m",
    "rpe_rmse_m",
    "completeness_percent",
    "trajectory_poses",
    "runtime_sec",
]


def parse_float(value):
    try:
        if value is None or value == "":
            return None
        x = float(value)
        if not math.isfinite(x):
            return None
        return x
    except Exception:
        return None


def fmt(value):
    if value is None:
        return ""
    return f"{value:.6f}"


def stats(values):
    clean = [v for v in values if v is not None]
    if not clean:
        return "", "", "", ""
    return (
        fmt(mean(clean)),
        fmt(stdev(clean) if len(clean) > 1 else 0.0),
        fmt(min(clean)),
        fmt(max(clean)),
    )


def write_summary(rows, group_keys, out_path):
    grouped = defaultdict(list)
    for row in rows:
        grouped[tuple(row[k] for k in group_keys)].append(row)

    header = list(group_keys) + [
        "total_runs",
        "pass_runs",
        "pass_rate_percent",
    ]
    for field in NUMERIC_FIELDS:
        header += [f"{field}_mean", f"{field}_std", f"{field}_min", f"{field}_max"]

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for key in sorted(grouped):
            group = grouped[key]
            total = len(group)
            pass_runs = sum(1 for row in group if row.get("status") == "PASS")
            out = list(key) + [
                total,
                pass_runs,
                fmt(100.0 * pass_runs / total if total else 0.0),
            ]
            for field in NUMERIC_FIELDS:
                out.extend(stats(parse_float(row.get(field)) for row in group))
            writer.writerow(out)


def main():
    parser = argparse.ArgumentParser(description="Summarize Exp.4 EuRoC repeated-run results.")
    parser.add_argument("summary_csv", help="Path to exp4_all_runs_*.csv")
    parser.add_argument("--out-dir", default="", help="Output directory. Defaults to the input CSV directory.")
    args = parser.parse_args()

    summary_csv = Path(args.summary_csv)
    out_dir = Path(args.out_dir) if args.out_dir else summary_csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(summary_csv, newline="") as f:
        rows = list(csv.DictReader(f))

    stem = summary_csv.stem.replace("exp4_all_runs_", "")
    by_sequence = out_dir / f"exp4_by_sequence_{stem}.csv"
    overall = out_dir / f"exp4_overall_{stem}.csv"

    write_summary(rows, ["dataset", "sequence", "sensor", "method"], by_sequence)
    write_summary(rows, ["dataset", "sensor", "method"], overall)

    print(f"input_rows={len(rows)}")
    print(f"by_sequence={by_sequence}")
    print(f"overall={overall}")


if __name__ == "__main__":
    main()
