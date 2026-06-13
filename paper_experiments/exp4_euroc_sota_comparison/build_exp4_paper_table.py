#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean


LITERATURE_ROWS = [
    {
        "Mode": "Mono-Inertial",
        "Sequence": "MH01",
        "ORB-SLAM3 ATE (m)": "0.038",
        "PKS ATE (m)": "0.020",
        "MSJCA-KS ATE (m)": "0.017",
        "ORB-SLAM3 Keyframes": "334",
        "PKS Keyframes": "322",
        "MSJCA-KS Keyframes": "314",
        "ORB-SLAM3 Runtime (s)": "≈198",
        "PKS Runtime (s)": "≈206",
        "MSJCA-KS Runtime (s)": "≈204",
    },
    {
        "Mode": "Mono-Inertial",
        "Sequence": "MH02",
        "ORB-SLAM3 ATE (m)": "0.068",
        "PKS ATE (m)": "0.029",
        "MSJCA-KS ATE (m)": "0.018",
        "ORB-SLAM3 Keyframes": "297",
        "PKS Keyframes": "294",
        "MSJCA-KS Keyframes": "283",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Mono-Inertial",
        "Sequence": "MH03",
        "ORB-SLAM3 ATE (m)": "0.050",
        "PKS ATE (m)": "0.033",
        "MSJCA-KS ATE (m)": "0.025",
        "ORB-SLAM3 Keyframes": "278",
        "PKS Keyframes": "253",
        "MSJCA-KS Keyframes": "242",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Mono-Inertial",
        "Sequence": "MH04",
        "ORB-SLAM3 ATE (m)": "0.056",
        "PKS ATE (m)": "0.054",
        "MSJCA-KS ATE (m)": "0.053",
        "ORB-SLAM3 Keyframes": "290",
        "PKS Keyframes": "256",
        "MSJCA-KS Keyframes": "244",
        "ORB-SLAM3 Runtime (s)": "≈114",
        "PKS Runtime (s)": "≈118",
        "MSJCA-KS Runtime (s)": "≈116",
    },
    {
        "Mode": "Mono-Inertial",
        "Sequence": "MH05",
        "ORB-SLAM3 ATE (m)": "0.083",
        "PKS ATE (m)": "0.054",
        "MSJCA-KS ATE (m)": "0.046",
        "ORB-SLAM3 Keyframes": "277",
        "PKS Keyframes": "265",
        "MSJCA-KS Keyframes": "260",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Mono-Inertial",
        "Sequence": "Avg. / Total",
        "ORB-SLAM3 ATE (m)": "0.059",
        "PKS ATE (m)": "0.038",
        "MSJCA-KS ATE (m)": "0.032",
        "ORB-SLAM3 Keyframes": "295.2",
        "PKS Keyframes": "278.0",
        "MSJCA-KS Keyframes": "268.6",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Stereo-Inertial",
        "Sequence": "MH01",
        "ORB-SLAM3 ATE (m)": "0.025",
        "PKS ATE (m)": "0.024",
        "MSJCA-KS ATE (m)": "0.019",
        "ORB-SLAM3 Keyframes": "127",
        "PKS Keyframes": "148",
        "MSJCA-KS Keyframes": "141",
        "ORB-SLAM3 Runtime (s)": "≈230",
        "PKS Runtime (s)": "≈250",
        "MSJCA-KS Runtime (s)": "≈248",
    },
    {
        "Mode": "Stereo-Inertial",
        "Sequence": "MH02",
        "ORB-SLAM3 ATE (m)": "0.031",
        "PKS ATE (m)": "0.039",
        "MSJCA-KS ATE (m)": "0.026",
        "ORB-SLAM3 Keyframes": "118",
        "PKS Keyframes": "142",
        "MSJCA-KS Keyframes": "135",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Stereo-Inertial",
        "Sequence": "MH03",
        "ORB-SLAM3 ATE (m)": "0.030",
        "PKS ATE (m)": "0.041",
        "MSJCA-KS ATE (m)": "0.031",
        "ORB-SLAM3 Keyframes": "139",
        "PKS Keyframes": "164",
        "MSJCA-KS Keyframes": "158",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Stereo-Inertial",
        "Sequence": "MH04",
        "ORB-SLAM3 ATE (m)": "0.051",
        "PKS ATE (m)": "0.044",
        "MSJCA-KS ATE (m)": "0.044",
        "ORB-SLAM3 Keyframes": "167",
        "PKS Keyframes": "221",
        "MSJCA-KS Keyframes": "208",
        "ORB-SLAM3 Runtime (s)": "≈118",
        "PKS Runtime (s)": "≈130",
        "MSJCA-KS Runtime (s)": "≈128",
    },
    {
        "Mode": "Stereo-Inertial",
        "Sequence": "MH05",
        "ORB-SLAM3 ATE (m)": "0.044",
        "PKS ATE (m)": "0.043",
        "MSJCA-KS ATE (m)": "0.040",
        "ORB-SLAM3 Keyframes": "156",
        "PKS Keyframes": "238",
        "MSJCA-KS Keyframes": "219",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
    {
        "Mode": "Stereo-Inertial",
        "Sequence": "Avg. / Total",
        "ORB-SLAM3 ATE (m)": "0.036",
        "PKS ATE (m)": "0.038",
        "MSJCA-KS ATE (m)": "0.032",
        "ORB-SLAM3 Keyframes": "141.4",
        "PKS Keyframes": "182.6",
        "MSJCA-KS Keyframes": "172.2",
        "ORB-SLAM3 Runtime (s)": "—",
        "PKS Runtime (s)": "—",
        "MSJCA-KS Runtime (s)": "—",
    },
]

HEADER = [
    "Mode",
    "Sequence",
    "ORB-SLAM3 ATE (m)",
    "PKS ATE (m)",
    "MSJCA-KS ATE (m)",
    "UW-IDVO ours ATE (m)",
    "ORB-SLAM3 Keyframes",
    "PKS Keyframes",
    "MSJCA-KS Keyframes",
    "UW-IDVO ours Keyframes",
    "ORB-SLAM3 Runtime (s)",
    "PKS Runtime (s)",
    "MSJCA-KS Runtime (s)",
    "UW-IDVO ours Runtime (s)",
]

SENSOR_LABEL = {
    "mono-inertial": "Mono-Inertial",
    "stereo-inertial": "Stereo-Inertial",
}


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


def fmt(value, digits=3):
    if value is None:
        return ""
    return f"{value:.{digits}f}"


def seq_label(seq):
    return seq.replace("_", "")


def status_is_usable(status):
    return status in {"PASS", "PASS_WITH_EXIT_0", "PASS_NO_EVO"} or str(status).startswith("PASS_WITH_EXIT_")


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def count_valid_lines(path):
    if not path or not Path(path).is_file():
        return None
    count = 0
    with open(path, errors="ignore") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                count += 1
    return count


def last_numeric_from_csv(path, fields):
    if not path or not Path(path).is_file():
        return None
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
    except Exception:
        return None
    for row in reversed(rows):
        for field in fields:
            value = parse_float(row.get(field))
            if value is not None:
                return value
    return None


def keyframes_for_run(row):
    adaptive = row.get("adaptive_csv", "")
    if not adaptive:
        candidate = Path(row.get("result_dir", "")) / "adaptive_frames.csv"
        if candidate.is_file():
            adaptive = str(candidate)
    value = last_numeric_from_csv(
        adaptive,
        [
            "number_of_keyframes",
            "num_keyframes",
            "keyframes",
            "mnKeyFrames",
        ],
    )
    if value is not None:
        return value

    result_dir = Path(row.get("result_dir", ""))
    if result_dir.is_dir():
        for candidate in sorted(result_dir.glob("kf_*.txt")):
            count = count_valid_lines(candidate)
            if count is not None:
                return float(count)
        for candidate in sorted(result_dir.glob("*keyframe*.txt")):
            count = count_valid_lines(candidate)
            if count is not None:
                return float(count)
    return None


def collect_ours(rows):
    values = defaultdict(lambda: {"ate": [], "runtime": [], "keyframes": []})
    for row in rows:
        if row.get("method") != "IDVO" or not status_is_usable(row.get("status", "")):
            continue
        mode = SENSOR_LABEL.get(row.get("sensor", ""))
        if not mode:
            continue
        sequence = seq_label(row.get("sequence", ""))
        key = (mode, sequence)
        ate = parse_float(row.get("ate_rmse_m"))
        runtime = parse_float(row.get("runtime_sec"))
        kfs = keyframes_for_run(row)
        if ate is not None:
            values[key]["ate"].append(ate)
        if runtime is not None:
            values[key]["runtime"].append(runtime)
        if kfs is not None:
            values[key]["keyframes"].append(kfs)
    return values


def mean_or_none(values):
    return mean(values) if values else None


def build_table(rows):
    ours = collect_ours(rows)
    output = []
    per_mode_sequence_values = defaultdict(lambda: {"ate": [], "runtime": [], "keyframes": []})

    for item in LITERATURE_ROWS:
        row = {key: item.get(key, "") for key in HEADER}
        mode = row["Mode"]
        sequence = row["Sequence"]
        if sequence != "Avg. / Total":
            own = ours.get((mode, sequence), {})
            own_ate = mean_or_none(own.get("ate", []))
            own_runtime = mean_or_none(own.get("runtime", []))
            own_keyframes = mean_or_none(own.get("keyframes", []))
            row["UW-IDVO ours ATE (m)"] = fmt(own_ate, 3)
            row["UW-IDVO ours Runtime (s)"] = fmt(own_runtime, 1)
            row["UW-IDVO ours Keyframes"] = fmt(own_keyframes, 1)
            if own_ate is not None:
                per_mode_sequence_values[mode]["ate"].append(own_ate)
            if own_runtime is not None:
                per_mode_sequence_values[mode]["runtime"].append(own_runtime)
            if own_keyframes is not None:
                per_mode_sequence_values[mode]["keyframes"].append(own_keyframes)
        else:
            own = per_mode_sequence_values[mode]
            row["UW-IDVO ours ATE (m)"] = fmt(mean_or_none(own["ate"]), 3)
            row["UW-IDVO ours Runtime (s)"] = fmt(mean_or_none(own["runtime"]), 1)
            row["UW-IDVO ours Keyframes"] = fmt(mean_or_none(own["keyframes"]), 1)
        output.append(row)
    return output


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=HEADER)
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(rows, path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("| " + " | ".join(HEADER) + " |\n")
        f.write("| " + " | ".join(["---"] * len(HEADER)) + " |\n")
        for row in rows:
            f.write("| " + " | ".join(str(row.get(key, "")) for key in HEADER) + " |\n")


def main():
    parser = argparse.ArgumentParser(description="Build the Exp.4 paper SOTA comparison table.")
    parser.add_argument("summary_csv", help="Path to exp4_all_runs_*.csv")
    parser.add_argument("--out-dir", default="", help="Output directory. Defaults to the input CSV directory.")
    args = parser.parse_args()

    summary_csv = Path(args.summary_csv)
    out_dir = Path(args.out_dir) if args.out_dir else summary_csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(summary_csv)
    stem = summary_csv.stem.replace("exp4_all_runs_", "")
    table = build_table(rows)

    csv_path = out_dir / f"exp4_paper_sota_table_{stem}.csv"
    md_path = out_dir / f"exp4_paper_sota_table_{stem}.md"
    write_csv(table, csv_path)
    write_markdown(table, md_path)

    print(f"input_rows={len(rows)}")
    print(f"paper_table_csv={csv_path}")
    print(f"paper_table_md={md_path}")


if __name__ == "__main__":
    main()
