#!/usr/bin/env bash
set -euo pipefail

# EuRoC Exp.4 focused parameter scan.
#
# Purpose:
#   Test four UW-IDVO settings on MH_01 and MH_04 only, using the same
#   non-TopK/non-Drop parameters as the Harbor115 setting.
#
# Default scope:
#   sequences: MH_01 MH_04
#   sensors: mono-inertial stereo-inertial
#   methods: IDVO
#   runs: 3 per case
#
# Override example:
#   RUNS_PER_CASE=10 UWIDVO_MODES=ORB_SLAM3,IDVO bash ...

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$SCRIPT_DIR"
RUNNER="$EXP_DIR/run_exp4_euroc_sota_comparison.sh"

SCAN_TAG="${SCAN_TAG:-exp4_euroc_mh01_mh04_topk_drop_scan_$(date +%Y%m%d_%H%M%S)}"
SCAN_ROOT="$EXP_DIR/param_scan_results/$SCAN_TAG"
SCAN_CONFIG_ROOT="$SCAN_ROOT/generated_configs"
SCAN_LOG="$SCAN_ROOT/${SCAN_TAG}.log"
SCAN_SUMMARY="$EXP_DIR/processed_results/${SCAN_TAG}_all_runs.csv"

BASE_MONO_CONFIG="${BASE_MONO_CONFIG:-$EXP_DIR/run_config/euroc_mono_inertial.yaml}"
BASE_STEREO_CONFIG="${BASE_STEREO_CONFIG:-$EXP_DIR/run_config/euroc_stereo_inertial.yaml}"

SEQUENCES="${SEQUENCES:-MH_01 MH_04}"
SENSORS="${SENSORS:-mono-inertial stereo-inertial}"
UWIDVO_MODES="${UWIDVO_MODES:-IDVO}"
RUNS_PER_CASE="${RUNS_PER_CASE:-3}"
MAX_FRAMES="${MAX_FRAMES:-0}"
DO_BUILD="${DO_BUILD:-0}"

mkdir -p "$SCAN_ROOT" "$SCAN_CONFIG_ROOT" "$EXP_DIR/processed_results"
: > "$SCAN_LOG"

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$SCAN_LOG"
}

set_yaml_value() {
  local file="$1" key="$2" value="$3"
  local escaped_key
  escaped_key="${key//./\\.}"
  if grep -qE "^[[:space:]]*${escaped_key}[[:space:]]*:" "$file"; then
    sed -i -E "s|^([[:space:]]*${escaped_key}[[:space:]]*:[[:space:]]*).*$|\\1${value}|" "$file"
  else
    printf '%s: %s\n' "$key" "$value" >> "$file"
  fi
}

apply_harbor115_common_params() {
  local cfg="$1" topk="$2" allow_drop="$3"

  set_yaml_value "$cfg" "InfoSelector.Enable" "1"
  set_yaml_value "$cfg" "InfoSelector.TopK" "$topk"
  set_yaml_value "$cfg" "InfoSelector.UseUniform" "1"
  set_yaml_value "$cfg" "InfoSelector.w_uniform" "0.12"
  set_yaml_value "$cfg" "InfoSelector.MinPxDist" "7"
  set_yaml_value "$cfg" "InfoSelector.LambdaInit" "1.0e-3"
  set_yaml_value "$cfg" "InfoSelector.StereoSafeKeep" "0"
  set_yaml_value "$cfg" "InfoSelector.Greedy" "0"

  set_yaml_value "$cfg" "InfoKF.Use" "1"
  set_yaml_value "$cfg" "InfoKF.AllowBitsDrop" "$allow_drop"
  set_yaml_value "$cfg" "InfoKF.LambdaMean" "1.0e-3"
  set_yaml_value "$cfg" "InfoKF.Dyn.Alpha" "0.25"
  set_yaml_value "$cfg" "InfoKF.Dyn.Beta" "0.0"
  set_yaml_value "$cfg" "InfoKF.Dyn.TauMin" "0.1"
  set_yaml_value "$cfg" "InfoKF.Dyn.TauMax" "5.0"
  set_yaml_value "$cfg" "InfoKF.Cum.Decay" "0.95"
  set_yaml_value "$cfg" "InfoKF.Cum.Thr" "0.5"
  set_yaml_value "$cfg" "InfoKF.MaxFramesForce" "15"

  set_yaml_value "$cfg" "EnableAdaptiveIDVO" "0"
  set_yaml_value "$cfg" "AdaptivePolicyType" '"Fixed"'
  set_yaml_value "$cfg" "UWIDVO.Verbose" "0"
  set_yaml_value "$cfg" "InfoSelector.Verbose" "0"
  set_yaml_value "$cfg" "InfoKF.Verbose" "0"
}

make_config_pair() {
  local scan_id="$1" topk="$2" allow_drop="$3"
  local out_dir="$SCAN_CONFIG_ROOT/$scan_id"
  mkdir -p "$out_dir"
  cp "$BASE_MONO_CONFIG" "$out_dir/euroc_mono_inertial.yaml"
  cp "$BASE_STEREO_CONFIG" "$out_dir/euroc_stereo_inertial.yaml"

  apply_harbor115_common_params "$out_dir/euroc_mono_inertial.yaml" "$topk" "$allow_drop"
  apply_harbor115_common_params "$out_dir/euroc_stereo_inertial.yaml" "$topk" "$allow_drop"
}

append_scan_rows() {
  local summary="$1" scan_id="$2" topk="$3" allow_drop="$4"
  python3 - "$summary" "$SCAN_SUMMARY" "$scan_id" "$topk" "$allow_drop" <<'PY'
import csv
import sys
from pathlib import Path

src, dst, scan_id, topk, allow_drop = sys.argv[1:6]
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
    "scan_allow_bits_drop": allow_drop,
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
  local scan_id="$1" topk="$2" allow_drop="$3"
  local run_tag="${SCAN_TAG}_${scan_id}"
  local mono_cfg="$SCAN_CONFIG_ROOT/$scan_id/euroc_mono_inertial.yaml"
  local stereo_cfg="$SCAN_CONFIG_ROOT/$scan_id/euroc_stereo_inertial.yaml"
  local summary="$EXP_DIR/processed_results/exp4_all_runs_${run_tag}.csv"

  log "============================================================"
  log "[SCAN] id=$scan_id topk=$topk allow_bits_drop=$allow_drop"
  make_config_pair "$scan_id" "$topk" "$allow_drop"

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

  append_scan_rows "$summary" "$scan_id" "$topk" "$allow_drop"
}

log "EuRoC MH_01/MH_04 UW-IDVO TopK/Drop scan started"
log "SCAN_TAG=$SCAN_TAG"
log "SEQUENCES=$SEQUENCES"
log "SENSORS=$SENSORS"
log "UWIDVO_MODES=$UWIDVO_MODES"
log "RUNS_PER_CASE=$RUNS_PER_CASE"
log "MAX_FRAMES=$MAX_FRAMES"
log "SCAN_SUMMARY=$SCAN_SUMMARY"

run_one_config "E01_k250_drop2p4" "250" "2.4"
run_one_config "E02_k300_drop2p8" "300" "2.8"
run_one_config "E03_k300_drop2p4" "300" "2.4"
run_one_config "E04_k300_drop2p0" "300" "2.0"

log "============================================================"
log "EuRoC MH_01/MH_04 UW-IDVO TopK/Drop scan finished"
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
        groups[(row["scan_id"], row["sensor"], row["method"])].append(row)

print("scan_id,sensor,method,passes,ate_mean,rpe_mean,completeness_mean,runtime_mean")
for key in sorted(groups):
    rs = groups[key]
    def mean_float(name):
        vals = [float(r[name]) for r in rs if r.get(name) not in ("", "NA")]
        return statistics.mean(vals) if vals else float("nan")
    print(
        f"{key[0]},{key[1]},{key[2]},{len(rs)},"
        f"{mean_float('ate_rmse_m'):.6f},"
        f"{mean_float('rpe_rmse_m'):.6f},"
        f"{mean_float('completeness_percent'):.3f},"
        f"{mean_float('runtime_sec'):.3f}"
    )
PY

