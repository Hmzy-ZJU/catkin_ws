#!/usr/bin/env bash
set -euo pipefail

# EuRoC UW-IDVO weak-IDPS + strong-IDKD scan.
#
# Purpose:
#   Keep more visual constraints than the standard IDPS setting, while using
#   stronger information-driven keyframe control. This is intended to test
#   whether UW-IDVO can preserve ORB-SLAM3 accuracy while still reducing runtime.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$SCRIPT_DIR"
RUNNER="$EXP_DIR/run_exp4_euroc_sota_comparison.sh"

SCAN_TAG="${SCAN_TAG:-exp4_euroc_weak_idps_strong_idkd_$(date +%Y%m%d_%H%M%S)}"
SCAN_ROOT="$EXP_DIR/param_scan_results/$SCAN_TAG"
SCAN_CONFIG_ROOT="$SCAN_ROOT/generated_configs"
SCAN_LOG="$SCAN_ROOT/${SCAN_TAG}.log"
SCAN_SUMMARY="$EXP_DIR/processed_results/${SCAN_TAG}_all_runs.csv"

BASE_MONO_CONFIG="${BASE_MONO_CONFIG:-$EXP_DIR/run_config/euroc_mono_inertial.yaml}"
BASE_STEREO_CONFIG="${BASE_STEREO_CONFIG:-$EXP_DIR/run_config/euroc_stereo_inertial.yaml}"

SEQUENCES="${SEQUENCES:-MH_01 MH_03 MH_05}"
SENSORS="${SENSORS:-stereo-inertial}"
UWIDVO_MODES="${UWIDVO_MODES:-ORB_SLAM3,IDKD,IDVO}"
RUNS_PER_CASE="${RUNS_PER_CASE:-2}"
MAX_FRAMES="${MAX_FRAMES:-0}"
DO_BUILD="${DO_BUILD:-0}"

mkdir -p "$SCAN_ROOT" "$SCAN_CONFIG_ROOT" "$EXP_DIR/processed_results"
: > "$SCAN_LOG"

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$SCAN_LOG"
}

set_yaml_value() {
  local file="$1" key="$2" value="$3"
  if grep -qE "^[[:space:]]*${key//./\\.}[[:space:]]*:" "$file"; then
    sed -i -E "s|^([[:space:]]*${key//./\\.}[[:space:]]*:[[:space:]]*).*$|\\1${value}|" "$file"
  else
    printf '%s: %s\n' "$key" "$value" >> "$file"
  fi
}

make_config_pair() {
  local scan_id="$1" topk="$2" uniform="$3" allow_drop="$4" cum_thr="$5" min_px="$6" safe_keep="$7" max_force="$8"
  local out_dir="$SCAN_CONFIG_ROOT/$scan_id"
  mkdir -p "$out_dir"
  cp "$BASE_MONO_CONFIG" "$out_dir/euroc_mono_inertial.yaml"
  cp "$BASE_STEREO_CONFIG" "$out_dir/euroc_stereo_inertial.yaml"

  for cfg in "$out_dir/euroc_mono_inertial.yaml" "$out_dir/euroc_stereo_inertial.yaml"; do
    set_yaml_value "$cfg" "InfoSelector.TopK" "$topk"
    set_yaml_value "$cfg" "InfoSelector.w_uniform" "$uniform"
    set_yaml_value "$cfg" "InfoSelector.MinPxDist" "$min_px"
    set_yaml_value "$cfg" "InfoSelector.Greedy" "0"
    set_yaml_value "$cfg" "InfoSelector.StereoSafeKeep" "$safe_keep"
    set_yaml_value "$cfg" "InfoKF.AllowBitsDrop" "$allow_drop"
    set_yaml_value "$cfg" "InfoKF.Cum.Thr" "$cum_thr"
    set_yaml_value "$cfg" "InfoKF.MaxFramesForce" "$max_force"
    set_yaml_value "$cfg" "UWIDVO.Verbose" "0"
    set_yaml_value "$cfg" "InfoSelector.Verbose" "0"
    set_yaml_value "$cfg" "InfoKF.Verbose" "0"
  done
}

append_scan_rows() {
  local summary="$1" scan_id="$2" topk="$3" uniform="$4" allow_drop="$5" cum_thr="$6" min_px="$7" safe_keep="$8" max_force="$9"
  python3 - "$summary" "$SCAN_SUMMARY" "$scan_id" "$topk" "$uniform" "$allow_drop" "$cum_thr" "$min_px" "$safe_keep" "$max_force" <<'PY'
import csv
import sys
from pathlib import Path

src, dst, scan_id, topk, uniform, allow_drop, cum_thr, min_px, safe_keep, max_force = sys.argv[1:11]
src = Path(src)
dst = Path(dst)
if not src.exists():
    raise SystemExit(f"missing summary: {src}")

with src.open(newline="") as f:
    rows = list(csv.DictReader(f))
if not rows:
    raise SystemExit(f"empty summary: {src}")

extra = {
    "scan_id": scan_id,
    "scan_topk": topk,
    "scan_w_uniform": uniform,
    "scan_allow_bits_drop": allow_drop,
    "scan_cum_thr": cum_thr,
    "scan_min_px_dist": min_px,
    "scan_stereo_safe_keep": safe_keep,
    "scan_max_frames_force": max_force,
}
fieldnames = list(extra.keys()) + list(rows[0].keys())
write_header = not dst.exists()
with dst.open("a", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    if write_header:
        writer.writeheader()
    for row in rows:
        merged = dict(extra)
        merged.update(row)
        writer.writerow(merged)
PY
}

run_one_config() {
  local scan_id="$1" topk="$2" uniform="$3" allow_drop="$4" cum_thr="$5" min_px="$6" safe_keep="$7" max_force="$8"
  local run_tag="${SCAN_TAG}_${scan_id}"
  local mono_cfg="$SCAN_CONFIG_ROOT/$scan_id/euroc_mono_inertial.yaml"
  local stereo_cfg="$SCAN_CONFIG_ROOT/$scan_id/euroc_stereo_inertial.yaml"
  local summary="$EXP_DIR/processed_results/exp4_all_runs_${run_tag}.csv"

  log "============================================================"
  log "[SCAN] id=$scan_id topk=$topk w_uniform=$uniform allow_bits_drop=$allow_drop cum_thr=$cum_thr min_px=$min_px stereo_safe_keep=$safe_keep max_force=$max_force"
  make_config_pair "$scan_id" "$topk" "$uniform" "$allow_drop" "$cum_thr" "$min_px" "$safe_keep" "$max_force"

  RUN_TAG="$run_tag" \
  MONO_INERTIAL_CONFIG="$mono_cfg" \
  STEREO_INERTIAL_CONFIG="$stereo_cfg" \
  SEQUENCES="$SEQUENCES" \
  SENSORS="$SENSORS" \
  UWIDVO_MODES="$UWIDVO_MODES" \
  RUNS_PER_CASE="$RUNS_PER_CASE" \
  MAX_FRAMES="$MAX_FRAMES" \
  DO_BUILD="$DO_BUILD" \
  bash "$RUNNER" 2>&1 | tee -a "$SCAN_LOG"

  append_scan_rows "$summary" "$scan_id" "$topk" "$uniform" "$allow_drop" "$cum_thr" "$min_px" "$safe_keep" "$max_force"
}

log "EuRoC weak-IDPS + strong-IDKD scan started"
log "SCAN_TAG=$SCAN_TAG"
log "SEQUENCES=$SEQUENCES"
log "SENSORS=$SENSORS"
log "UWIDVO_MODES=$UWIDVO_MODES"
log "RUNS_PER_CASE=$RUNS_PER_CASE"
log "MAX_FRAMES=$MAX_FRAMES"
log "SCAN_SUMMARY=$SCAN_SUMMARY"

# Five candidates. IDKD-only is included through UWIDVO_MODES=...,IDKD by
# default, so these five rows focus on weak IDPS + strong IDKD.
run_one_config "W01_k300_u008_drop1p8" "300" "0.08" "1.8" "0.4" "5" "0" "240"
run_one_config "W02_k300_u012_drop1p8" "300" "0.12" "1.8" "0.4" "5" "0" "240"
run_one_config "W03_k400_u008_drop1p8" "400" "0.08" "1.8" "0.4" "5" "0" "240"
run_one_config "W04_k400_u012_drop1p6" "400" "0.12" "1.6" "0.35" "5" "0" "220"
run_one_config "W05_k500_u008_drop1p6" "500" "0.08" "1.6" "0.35" "5" "0" "220"

log "============================================================"
log "EuRoC weak-IDPS + strong-IDKD scan finished"
log "Merged scan summary: $SCAN_SUMMARY"
log "Generated configs: $SCAN_CONFIG_ROOT"
log "Log: $SCAN_LOG"

python3 - "$SCAN_SUMMARY" <<'PY' | tee -a "$SCAN_LOG"
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

path = Path(sys.argv[1])
rows = list(csv.DictReader(path.open()))
groups = defaultdict(list)
for row in rows:
    if row.get("status", "").startswith("PASS"):
        groups[(row["scan_id"], row["method"])].append(row)

print("weak-IDPS + strong-IDKD scan means:")
print("scan_id,method,passes,ate_mean,rpe_mean,completeness_mean,runtime_mean")
for key in sorted(groups):
    rs = groups[key]
    def mean_float(name):
        vals = [float(r[name]) for r in rs if r.get(name) not in ("", "NA")]
        return statistics.mean(vals) if vals else float("nan")
    print(
        f"{key[0]},{key[1]},{len(rs)},"
        f"{mean_float('ate_rmse_m'):.6f},"
        f"{mean_float('rpe_rmse_m'):.6f},"
        f"{mean_float('completeness_percent'):.3f},"
        f"{mean_float('runtime_sec'):.3f}"
    )
PY

