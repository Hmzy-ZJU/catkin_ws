#!/usr/bin/env bash
set -euo pipefail

# Exp.3 AquaticVision stereo parameter scan.
# ROS-online source follows the main Exp.3 runner.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$SCRIPT_DIR"

SCAN_TAG="${RUN_TAG:-exp3_aquaticvision_stereo_param_scan_$(date +%Y%m%d_%H%M%S)}"
WS="${WS:-$HOME/catkin_ws}"
AQUATIC_ROOT="${AQUATIC_ROOT:-$WS/dataset_AquaticVision}"
BASE_STEREO_CONFIG="${BASE_STEREO_CONFIG:-$EXP_DIR/run_config/aquaticvision_stereo.yaml}"

SEQUENCES="${SEQUENCES:-01 02 03 05}"
RUNS_PER_CASE="${RUNS_PER_CASE:-1}"
BAG_DURATION="${BAG_DURATION:-0}"
SAVE_TRAJ_TIMEOUT="${SAVE_TRAJ_TIMEOUT:-60}"
DO_BUILD="${DO_BUILD:-0}"
UWIDVO_MODES="${UWIDVO_MODES:-ORB_SLAM3,IDVO}"
MIN_COMPLETENESS_PERCENT="${MIN_COMPLETENESS_PERCENT:-50}"

# Format: scan_id:topk:drop:force
# Keep the first row as the current reference setting.
SCAN_CONFIGS="${SCAN_CONFIGS:-A01_k115_d2p0_f120:115:2.0:120 A02_k115_d2p8_f300:115:2.8:300 A03_k115_d3p2_f300:115:3.2:300 A04_k100_d2p8_f300:100:2.8:300 A05_k150_d3p2_f300:150:3.2:300}"

SCAN_ROOT="$EXP_DIR/raw_results/$SCAN_TAG"
CONFIG_ROOT="$SCAN_ROOT/generated_configs"
COMBINED="$EXP_DIR/processed_results/${SCAN_TAG}_all_runs_with_efficiency.csv"
mkdir -p "$CONFIG_ROOT" "$EXP_DIR/processed_results"

make_config() {
  local topk="$1" drop="$2" force="$3" out="$4"
  awk -v topk="$topk" -v drop="$drop" -v force="$force" '
    /^[[:space:]]*InfoSelector\.TopK[[:space:]]*:/ { print "InfoSelector.TopK: " topk; next }
    /^[[:space:]]*InfoKF\.AllowBitsDrop[[:space:]]*:/ { print "InfoKF.AllowBitsDrop: " drop; next }
    /^[[:space:]]*InfoKF\.MaxFramesForce[[:space:]]*:/ { print "InfoKF.MaxFramesForce: " force; next }
    { print }
  ' "$BASE_STEREO_CONFIG" > "$out"
}

printf 'scan_id,topk,drop,force,experiment_id,dataset,sequence,sensor,method,run_id,status,ate_rmse_m,rpe_rmse_m,completeness_percent,trajectory_poses,input_frames,runtime_sec,selected_points_mean,selection_ratio_mean,keyframes_final,map_points_final,tracking_time_ms_mean,local_ba_time_ms_mean,result_dir,notes\n' > "$COMBINED"

echo "[INFO] Exp.3 AquaticVision stereo parameter scan"
echo "[INFO] SCAN_TAG=$SCAN_TAG"
echo "[INFO] SEQUENCES=$SEQUENCES"
echo "[INFO] SCAN_CONFIGS=$SCAN_CONFIGS"
echo "[INFO] RUNS_PER_CASE=$RUNS_PER_CASE"
echo "[INFO] BAG_DURATION=$BAG_DURATION"
echo "[INFO] UWIDVO_MODES=$UWIDVO_MODES"

for spec in $SCAN_CONFIGS; do
  IFS=':' read -r scan_id topk drop force <<< "$spec"
  if [ -z "${scan_id:-}" ] || [ -z "${topk:-}" ] || [ -z "${drop:-}" ] || [ -z "${force:-}" ]; then
    echo "[WARN] Invalid scan spec: $spec" >&2
    continue
  fi

  cfg="$CONFIG_ROOT/aquaticvision_stereo_${scan_id}.yaml"
  run_tag="${SCAN_TAG}_${scan_id}"
  make_config "$topk" "$drop" "$force" "$cfg"

  echo "[INFO] Running $scan_id: TopK=$topk Drop=$drop Force=$force"
  RUN_TAG="$run_tag" \
    AQUATIC_ROOT="$AQUATIC_ROOT" \
    AQUATIC_STEREO_CONFIG="$cfg" \
    SEQUENCES="$SEQUENCES" \
    SENSORS="stereo" \
    UWIDVO_MODES="$UWIDVO_MODES" \
    RUNS_PER_CASE="$RUNS_PER_CASE" \
    BAG_DURATION="$BAG_DURATION" \
    SAVE_TRAJ_TIMEOUT="$SAVE_TRAJ_TIMEOUT" \
    DO_BUILD="$DO_BUILD" \
    MIN_COMPLETENESS_PERCENT="$MIN_COMPLETENESS_PERCENT" \
    bash "$EXP_DIR/run_exp3_aquaticvision_generalization.sh"

  summary="$EXP_DIR/processed_results/exp3_all_runs_${run_tag}.csv"
  if [ ! -s "$summary" ]; then
    echo "[WARN] Missing summary: $summary" >&2
    continue
  fi

  python3 - "$summary" "$COMBINED" "$scan_id" "$topk" "$drop" "$force" <<'PY'
import csv
import os
import statistics
import sys

summary, combined, scan_id, topk, drop, force = sys.argv[1:7]

def read_efficiency(run_dir):
    path = os.path.join(run_dir, "adaptive_frames.csv")
    out = {
        "selected_points_mean": "",
        "selection_ratio_mean": "",
        "keyframes_final": "",
        "map_points_final": "",
        "tracking_time_ms_mean": "",
        "local_ba_time_ms_mean": "",
    }
    if not os.path.exists(path):
        return out
    with open(path, newline="", encoding="utf-8", errors="ignore") as fp:
        rows = list(csv.DictReader(fp))

    def values(col):
        vals = []
        for row in rows:
            v = row.get(col, "")
            if v in ("", "NA", "nan"):
                continue
            try:
                vals.append(float(v))
            except ValueError:
                pass
        return vals

    mean_cols = [
        ("selected_point_number", "selected_points_mean"),
        ("selection_ratio", "selection_ratio_mean"),
        ("tracking_time_ms", "tracking_time_ms_mean"),
        ("recent_local_ba_time_ms", "local_ba_time_ms_mean"),
    ]
    for src, dst in mean_cols:
        vals = values(src)
        if vals:
            out[dst] = f"{statistics.mean(vals):.6f}"

    last_cols = [
        ("number_of_keyframes", "keyframes_final"),
        ("number_of_map_points", "map_points_final"),
    ]
    for src, dst in last_cols:
        vals = values(src)
        if vals:
            out[dst] = f"{vals[-1]:.0f}"
    return out

with open(summary, newline="", encoding="utf-8") as fp:
    rows = list(csv.DictReader(fp))

fieldnames = [
    "scan_id", "topk", "drop", "force",
    "experiment_id", "dataset", "sequence", "sensor", "method", "run_id", "status",
    "ate_rmse_m", "rpe_rmse_m", "completeness_percent", "trajectory_poses", "input_frames",
    "runtime_sec", "selected_points_mean", "selection_ratio_mean", "keyframes_final",
    "map_points_final", "tracking_time_ms_mean", "local_ba_time_ms_mean", "result_dir", "notes",
]

with open(combined, "a", newline="", encoding="utf-8") as fp:
    writer = csv.DictWriter(fp, fieldnames=fieldnames)
    for row in rows:
        eff = read_efficiency(row.get("result_dir", ""))
        out = {
            "scan_id": scan_id,
            "topk": topk,
            "drop": drop,
            "force": force,
            "experiment_id": row.get("experiment_id", ""),
            "dataset": row.get("dataset", ""),
            "sequence": row.get("sequence", ""),
            "sensor": row.get("sensor", ""),
            "method": row.get("method", ""),
            "run_id": row.get("run_id", ""),
            "status": row.get("status", ""),
            "ate_rmse_m": row.get("ate_rmse_m", ""),
            "rpe_rmse_m": row.get("rpe_rmse_m", ""),
            "completeness_percent": row.get("completeness_percent", ""),
            "trajectory_poses": row.get("trajectory_poses", ""),
            "input_frames": row.get("input_frames", ""),
            "runtime_sec": row.get("runtime_sec", ""),
            "result_dir": row.get("result_dir", ""),
            "notes": row.get("notes", ""),
        }
        out.update(eff)
        writer.writerow(out)
PY
done

echo "[INFO] Combined results: $COMBINED"
column -s, -t "$COMBINED" || true
