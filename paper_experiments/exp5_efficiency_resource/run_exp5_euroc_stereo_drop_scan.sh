#!/usr/bin/env bash
set -euo pipefail

# Exp.5 EuRoC stereo Drop scan.
# Fixed: TopK=150, MaxFramesForce=300.
# Scanned: InfoKF.AllowBitsDrop.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$SCRIPT_DIR"

SCAN_TAG="${RUN_TAG:-exp5_euroc_stereo_drop_scan_$(date +%Y%m%d_%H%M%S)}"
DROPS="${DROPS:-3.2 3.6 4.0 4.4}"
TOPK="${TOPK:-150}"
FORCE="${FORCE:-300}"

WS="${WS:-$HOME/catkin_ws}"
EUROC_ROOT="${EUROC_ROOT:-$WS/dataset_EuRoc}"
BASE_STEREO_CONFIG="${BASE_STEREO_CONFIG:-$EXP_DIR/run_config/euroc_stereo.yaml}"

EUROC_SEQUENCES="${EUROC_SEQUENCES:-MH_01}"
RUNS_PER_CASE="${RUNS_PER_CASE:-1}"
BAG_DURATION="${BAG_DURATION:-60}"
SAVE_TRAJ_TIMEOUT="${SAVE_TRAJ_TIMEOUT:-60}"
DO_BUILD="${DO_BUILD:-0}"
UWIDVO_MODES="${UWIDVO_MODES:-ORB_SLAM3,IDVO}"

SCAN_ROOT="$EXP_DIR/raw_results/$SCAN_TAG"
CONFIG_ROOT="$SCAN_ROOT/generated_configs"
COMBINED="$EXP_DIR/processed_results/${SCAN_TAG}_all_runs_with_evo.csv"
mkdir -p "$CONFIG_ROOT" "$EXP_DIR/processed_results"

drop_id() {
  printf '%s' "$1" | sed 's/\./p/g'
}

make_config() {
  local drop="$1" out="$2"
  awk -v topk="$TOPK" -v drop="$drop" -v force="$FORCE" '
    /^[[:space:]]*InfoSelector\.TopK[[:space:]]*:/ { print "InfoSelector.TopK: " topk; next }
    /^[[:space:]]*InfoKF\.AllowBitsDrop[[:space:]]*:/ { print "InfoKF.AllowBitsDrop: " drop; next }
    /^[[:space:]]*InfoKF\.MaxFramesForce[[:space:]]*:/ { print "InfoKF.MaxFramesForce: " force; next }
    { print }
  ' "$BASE_STEREO_CONFIG" > "$out"
}

printf 'scan_id,drop,experiment_id,dataset,sequence,sensor,method,run_id,status,ate_rmse_m,rpe_rmse_m,tracking_time_ms_mean,local_ba_time_ms_mean,selected_points_mean,keyframes_final,map_points_final,memory_mb,runtime_sec,bag,result_dir,notes\n' > "$COMBINED"

echo "[INFO] Exp.5 EuRoC stereo Drop scan"
echo "[INFO] SCAN_TAG=$SCAN_TAG"
echo "[INFO] DROPS=$DROPS"
echo "[INFO] TOPK=$TOPK"
echo "[INFO] FORCE=$FORCE"
echo "[INFO] EUROC_SEQUENCES=$EUROC_SEQUENCES"
echo "[INFO] RUNS_PER_CASE=$RUNS_PER_CASE"
echo "[INFO] BAG_DURATION=$BAG_DURATION"
echo "[INFO] UWIDVO_MODES=$UWIDVO_MODES"

for drop in $DROPS; do
  sid="D$(drop_id "$drop")"
  cfg="$CONFIG_ROOT/euroc_stereo_topk${TOPK}_drop${sid}_force${FORCE}.yaml"
  run_tag="${SCAN_TAG}_${sid}"
  make_config "$drop" "$cfg"

  echo "[INFO] Running $sid: TopK=$TOPK Drop=$drop Force=$FORCE"
  RUN_TAG="$run_tag" \
    DATASETS="euroc" \
    EUROC_SEQUENCES="$EUROC_SEQUENCES" \
    SENSORS="stereo" \
    UWIDVO_MODES="$UWIDVO_MODES" \
    RUNS_PER_CASE="$RUNS_PER_CASE" \
    BAG_DURATION="$BAG_DURATION" \
    SAVE_TRAJ_TIMEOUT="$SAVE_TRAJ_TIMEOUT" \
    DO_BUILD="$DO_BUILD" \
    EUROC_STEREO_CONFIG="$cfg" \
    bash "$EXP_DIR/run_exp5_efficiency_resource.sh"

  summary="$EXP_DIR/processed_results/exp5_all_runtime_metrics_${run_tag}.csv"
  if [ ! -s "$summary" ]; then
    echo "[WARN] Missing summary: $summary" >&2
    continue
  fi

  python3 - "$summary" "$COMBINED" "$sid" "$drop" "$EUROC_ROOT" <<'PY'
import csv
import os
import subprocess
import sys

summary, combined, scan_id, drop, euroc_root = sys.argv[1:6]

def convert_traj(src, dst):
    with open(src, "r", encoding="utf-8", errors="ignore") as fin, open(dst, "w", encoding="utf-8") as fout:
        for line in fin:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            if abs(t) > 1e12:
                t /= 1e9
            fout.write(f"{t:.9f} {' '.join(parts[1:8])}\n")

def evo_rmse(kind, gt, traj):
    cmd = [
        "evo_ape" if kind == "ape" else "evo_rpe",
        "euroc",
        gt,
        traj,
        "-a",
        "--t_max_diff",
        "0.05",
        "--no_warnings",
    ]
    try:
        out = subprocess.run(cmd, check=False, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout
    except Exception:
        return ""
    for line in out.splitlines():
        cols = line.split()
        if cols and cols[0] == "rmse" and len(cols) >= 2:
            return cols[1]
    return ""

with open(summary, newline="", encoding="utf-8") as fp:
    rows = list(csv.DictReader(fp))

fieldnames = [
    "scan_id", "drop", "experiment_id", "dataset", "sequence", "sensor", "method", "run_id", "status",
    "ate_rmse_m", "rpe_rmse_m", "tracking_time_ms_mean", "local_ba_time_ms_mean", "selected_points_mean",
    "keyframes_final", "map_points_final", "memory_mb", "runtime_sec", "bag", "result_dir", "notes",
]

with open(combined, "a", newline="", encoding="utf-8") as fp:
    writer = csv.DictWriter(fp, fieldnames=fieldnames)
    for row in rows:
        seq = row.get("sequence", "")
        run_dir = row.get("result_dir", "")
        traj = os.path.join(run_dir, "trajectory.txt")
        traj_sec = os.path.join(run_dir, "trajectory_sec.txt")
        gt = os.path.join(euroc_root, "GT", f"{seq}.csv")
        ate = ""
        rpe = ""
        if os.path.exists(traj) and os.path.exists(gt):
            convert_traj(traj, traj_sec)
            ate = evo_rmse("ape", gt, traj_sec)
            rpe = evo_rmse("rpe", gt, traj_sec)
        out = {
            "scan_id": scan_id,
            "drop": drop,
            "experiment_id": row.get("experiment_id", ""),
            "dataset": row.get("dataset", ""),
            "sequence": seq,
            "sensor": row.get("sensor", ""),
            "method": row.get("method", ""),
            "run_id": row.get("run_id", ""),
            "status": row.get("status", ""),
            "ate_rmse_m": ate,
            "rpe_rmse_m": rpe,
            "tracking_time_ms_mean": row.get("tracking_time_ms_mean", ""),
            "local_ba_time_ms_mean": row.get("local_ba_time_ms_mean", ""),
            "selected_points_mean": row.get("selected_points_mean", ""),
            "keyframes_final": row.get("keyframes_final", ""),
            "map_points_final": row.get("map_points_final", ""),
            "memory_mb": row.get("memory_mb", ""),
            "runtime_sec": row.get("runtime_sec", ""),
            "bag": row.get("bag", ""),
            "result_dir": run_dir,
            "notes": row.get("notes", ""),
        }
        writer.writerow(out)
PY
done

echo "[INFO] Combined results: $COMBINED"
column -s, -t "$COMBINED" || true
