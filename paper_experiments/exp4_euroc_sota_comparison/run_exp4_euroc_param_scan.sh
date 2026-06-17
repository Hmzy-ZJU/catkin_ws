#!/usr/bin/env bash
set -euo pipefail

# EuRoC UW-IDVO parameter scan.
#
# This script keeps the main Exp.4 runner unchanged. For each candidate
# parameter set, it creates temporary mono-inertial and stereo-inertial YAML
# files, invokes run_exp4_euroc_sota_comparison.sh, and merges the per-run CSVs
# into one scan summary.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$SCRIPT_DIR"
RUNNER="$EXP_DIR/run_exp4_euroc_sota_comparison.sh"

SCAN_TAG="${SCAN_TAG:-exp4_euroc_param_scan_$(date +%Y%m%d_%H%M%S)}"
SCAN_ROOT="$EXP_DIR/param_scan_results/$SCAN_TAG"
SCAN_CONFIG_ROOT="$SCAN_ROOT/generated_configs"
SCAN_LOG="$SCAN_ROOT/${SCAN_TAG}.log"
SCAN_SUMMARY="$EXP_DIR/processed_results/${SCAN_TAG}_all_runs.csv"

BASE_MONO_CONFIG="${BASE_MONO_CONFIG:-$EXP_DIR/run_config/euroc_mono_inertial.yaml}"
BASE_STEREO_CONFIG="${BASE_STEREO_CONFIG:-$EXP_DIR/run_config/euroc_stereo_inertial.yaml}"

# Default scan is intentionally small enough for iteration. Override these
# variables when doing the final confirmation run.
SEQUENCES="${SEQUENCES:-MH_01 MH_03 MH_05}"
SENSORS="${SENSORS:-stereo-inertial}"
UWIDVO_MODES="${UWIDVO_MODES:-ORB_SLAM3,IDVO}"
RUNS_PER_CASE="${RUNS_PER_CASE:-2}"
MAX_FRAMES="${MAX_FRAMES:-0}"
DO_BUILD="${DO_BUILD:-0}"

# Optional grid-scan interface. If TOPK_LIST or DROP_LIST is set, the script
# scans the Cartesian product of TOPK_LIST x DROP_LIST and writes FORCE to
# InfoKF.MaxFramesForce. If neither is set, the legacy five hand-picked
# candidates below are used.
TOPK_LIST="${TOPK_LIST:-}"
DROP_LIST="${DROP_LIST:-}"
FORCE="${FORCE:-}"
W_UNIFORM="${W_UNIFORM:-0.12}"
CUM_THR="${CUM_THR:-0.5}"
GREEDY="${GREEDY:-0}"

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
  local scan_id="$1" topk="$2" uniform="$3" allow_drop="$4" cum_thr="$5" greedy="$6" force="$7"
  local out_dir="$SCAN_CONFIG_ROOT/$scan_id"
  mkdir -p "$out_dir"
  cp "$BASE_MONO_CONFIG" "$out_dir/euroc_mono_inertial.yaml"
  cp "$BASE_STEREO_CONFIG" "$out_dir/euroc_stereo_inertial.yaml"

  for cfg in "$out_dir/euroc_mono_inertial.yaml" "$out_dir/euroc_stereo_inertial.yaml"; do
    set_yaml_value "$cfg" "InfoSelector.TopK" "$topk"
    set_yaml_value "$cfg" "InfoSelector.w_uniform" "$uniform"
    set_yaml_value "$cfg" "InfoSelector.Greedy" "$greedy"
    set_yaml_value "$cfg" "InfoSelector.StereoSafeKeep" "0"
    set_yaml_value "$cfg" "InfoKF.AllowBitsDrop" "$allow_drop"
    set_yaml_value "$cfg" "InfoKF.Cum.Thr" "$cum_thr"
    if [[ -n "$force" ]]; then
      set_yaml_value "$cfg" "InfoKF.MaxFramesForce" "$force"
    fi
    set_yaml_value "$cfg" "UWIDVO.Verbose" "0"
    set_yaml_value "$cfg" "InfoSelector.Verbose" "0"
    set_yaml_value "$cfg" "InfoKF.Verbose" "0"
  done
}

append_scan_rows() {
  local summary="$1" scan_id="$2" topk="$3" uniform="$4" allow_drop="$5" cum_thr="$6" greedy="$7"
  python3 - "$summary" "$SCAN_SUMMARY" "$scan_id" "$topk" "$uniform" "$allow_drop" "$cum_thr" "$greedy" <<'PY'
import csv
import sys
from pathlib import Path

src, dst, scan_id, topk, uniform, allow_drop, cum_thr, greedy = sys.argv[1:9]
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
    "scan_greedy": greedy,
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
  local scan_id="$1" topk="$2" uniform="$3" allow_drop="$4" cum_thr="$5" greedy="$6" force="${7:-}"
  local run_tag="${SCAN_TAG}_${scan_id}"
  local mono_cfg="$SCAN_CONFIG_ROOT/$scan_id/euroc_mono_inertial.yaml"
  local stereo_cfg="$SCAN_CONFIG_ROOT/$scan_id/euroc_stereo_inertial.yaml"
  local summary="$EXP_DIR/processed_results/exp4_all_runs_${run_tag}.csv"

  log "============================================================"
  log "[SCAN] id=$scan_id topk=$topk w_uniform=$uniform allow_bits_drop=$allow_drop cum_thr=$cum_thr greedy=$greedy force=${force:-base}"
  make_config_pair "$scan_id" "$topk" "$uniform" "$allow_drop" "$cum_thr" "$greedy" "$force"

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

  append_scan_rows "$summary" "$scan_id" "$topk" "$uniform" "$allow_drop" "$cum_thr" "$greedy"
}

log "EuRoC UW-IDVO parameter scan started"
log "SCAN_TAG=$SCAN_TAG"
log "SEQUENCES=$SEQUENCES"
log "SENSORS=$SENSORS"
log "UWIDVO_MODES=$UWIDVO_MODES"
log "RUNS_PER_CASE=$RUNS_PER_CASE"
log "MAX_FRAMES=$MAX_FRAMES"
log "TOPK_LIST=${TOPK_LIST:-<legacy>}"
log "DROP_LIST=${DROP_LIST:-<legacy>}"
log "FORCE=${FORCE:-<base config>}"
log "SCAN_SUMMARY=$SCAN_SUMMARY"

if [[ -n "$TOPK_LIST" || -n "$DROP_LIST" ]]; then
  if [[ -z "$TOPK_LIST" || -z "$DROP_LIST" ]]; then
    echo "ERROR: TOPK_LIST and DROP_LIST must be set together for grid scan." >&2
    exit 2
  fi
  for topk in $TOPK_LIST; do
    for drop in $DROP_LIST; do
      drop_id="${drop//./p}"
      force_id="${FORCE:-base}"
      run_one_config "K${topk}_D${drop_id}_F${force_id}" "$topk" "$W_UNIFORM" "$drop" "$CUM_THR" "$GREEDY" "$FORCE"
    done
  done
else
  # Five legacy candidate configurations:
  #   C01: current paper setting, fastest and most aggressive point budget.
  #   C02: slightly safer point budget.
  #   C03: accuracy-oriented point budget with original IDKD threshold.
  #   C04: same point budget as C03 with more conservative keyframe insertion.
  #   C05: most accuracy-oriented candidate while keeping fast TopK selection.
  run_one_config "C01_k095_drop2p2" "95"  "0.12" "2.2" "0.5" "0" "$FORCE"
  run_one_config "C02_k120_drop2p2" "120" "0.12" "2.2" "0.5" "0" "$FORCE"
  run_one_config "C03_k150_drop2p2" "150" "0.12" "2.2" "0.5" "0" "$FORCE"
  run_one_config "C04_k150_drop2p6" "150" "0.12" "2.6" "0.5" "0" "$FORCE"
  run_one_config "C05_k180_drop2p6" "180" "0.12" "2.6" "0.5" "0" "$FORCE"
fi

log "============================================================"
log "EuRoC UW-IDVO parameter scan finished"
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
    if row.get("method") == "IDVO" and row.get("status", "").startswith("PASS"):
        groups[row["scan_id"]].append(row)

print("IDVO scan means:")
print("scan_id,passes,ate_mean,rpe_mean,completeness_mean,runtime_mean")
for scan_id in sorted(groups):
    rs = groups[scan_id]
    def mean_float(name):
        vals = [float(r[name]) for r in rs if r.get(name) not in ("", "NA")]
        return statistics.mean(vals) if vals else float("nan")
    print(
        f"{scan_id},{len(rs)},"
        f"{mean_float('ate_rmse_m'):.6f},"
        f"{mean_float('rpe_rmse_m'):.6f},"
        f"{mean_float('completeness_percent'):.3f},"
        f"{mean_float('runtime_sec'):.3f}"
    )
PY
