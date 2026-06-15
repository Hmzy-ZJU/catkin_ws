#!/usr/bin/env bash
set -euo pipefail

# Exp.4 EuRoC TopK/Drop scan, one run per setting.
#
# This script is designed for parameter selection. It runs ORB-SLAM3 once as
# the same-machine baseline, then scans IDVO with TopK/AllowBitsDrop candidates.
# All other UW-IDVO parameters are fixed to the Harbor115 setting.
#
# Default scope:
#   sequences: MH_01 MH_04
#   sensors: mono-inertial stereo-inertial
#   runs: 1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$SCRIPT_DIR"
RUNNER="$EXP_DIR/run_exp4_euroc_sota_comparison.sh"

SCAN_TAG="${SCAN_TAG:-exp4_euroc_topk_drop_1run_scan_$(date +%Y%m%d_%H%M%S)}"
SCAN_ROOT="$EXP_DIR/param_scan_results/$SCAN_TAG"
SCAN_CONFIG_ROOT="$SCAN_ROOT/generated_configs"
SCAN_LOG="$SCAN_ROOT/${SCAN_TAG}.log"
SCAN_SUMMARY="$EXP_DIR/processed_results/${SCAN_TAG}_all_runs.csv"
SCAN_NORMALIZED="$EXP_DIR/processed_results/${SCAN_TAG}_normalized_runtime.csv"
SCAN_MEANS="$EXP_DIR/processed_results/${SCAN_TAG}_means.csv"

BASE_MONO_CONFIG="${BASE_MONO_CONFIG:-$EXP_DIR/run_config/euroc_mono_inertial.yaml}"
BASE_STEREO_CONFIG="${BASE_STEREO_CONFIG:-$EXP_DIR/run_config/euroc_stereo_inertial.yaml}"

SEQUENCES="${SEQUENCES:-MH_01 MH_04}"
SENSORS="${SENSORS:-mono-inertial stereo-inertial}"
RUNS_PER_CASE="${RUNS_PER_CASE:-1}"
MAX_FRAMES="${MAX_FRAMES:-0}"
DO_BUILD="${DO_BUILD:-0}"

mkdir -p "$SCAN_ROOT" "$SCAN_CONFIG_ROOT" "$EXP_DIR/processed_results"
: > "$SCAN_LOG"
rm -f "$SCAN_SUMMARY" "$SCAN_NORMALIZED" "$SCAN_MEANS"

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

rows = list(csv.DictReader(src.open(newline="")))
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

run_baseline_once() {
  local run_tag="${SCAN_TAG}_ORB_BASELINE"
  local summary="$EXP_DIR/processed_results/exp4_all_runs_${run_tag}.csv"

  log "============================================================"
  log "[BASELINE] ORB_SLAM3"
  RUN_TAG="$run_tag" \
  SEQUENCES="$SEQUENCES" \
  SENSORS="$SENSORS" \
  UWIDVO_MODES="ORB_SLAM3" \
  RUNS_PER_CASE="$RUNS_PER_CASE" \
  MAX_FRAMES="$MAX_FRAMES" \
  DO_BUILD="$DO_BUILD" \
  bash "$RUNNER" 2>&1 | tee -a "$SCAN_LOG"

  append_scan_rows "$summary" "ORB_BASELINE" "" ""
}

run_idvo_config() {
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
  UWIDVO_MODES="IDVO" \
  RUNS_PER_CASE="$RUNS_PER_CASE" \
  MAX_FRAMES="$MAX_FRAMES" \
  DO_BUILD="$DO_BUILD" \
  bash "$RUNNER" 2>&1 | tee -a "$SCAN_LOG"

  append_scan_rows "$summary" "$scan_id" "$topk" "$allow_drop"
}

build_normalized_tables() {
  python3 - "$SCAN_SUMMARY" "$SCAN_NORMALIZED" "$SCAN_MEANS" <<'PY'
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

summary, normalized_out, means_out = map(Path, sys.argv[1:4])
rows = list(csv.DictReader(summary.open(newline="")))

baseline = {}
for row in rows:
    if row.get("method") == "ORB_SLAM3" and row.get("status", "").startswith("PASS"):
        key = (row["sequence"], row["sensor"], row["run_id"])
        baseline[key] = float(row["runtime_sec"])

norm_rows = []
for row in rows:
    if row.get("method") != "IDVO" or not row.get("status", "").startswith("PASS"):
        continue
    key = (row["sequence"], row["sensor"], row["run_id"])
    base_runtime = baseline.get(key)
    if not base_runtime:
        continue
    runtime = float(row["runtime_sec"])
    ratio = runtime / base_runtime
    delta = (ratio - 1.0) * 100.0
    out = dict(row)
    out["orb3_runtime_sec"] = f"{base_runtime:.6f}"
    out["runtime_ratio_vs_orb3"] = f"{ratio:.6f}"
    out["runtime_delta_percent_vs_orb3"] = f"{delta:.6f}"
    norm_rows.append(out)

if norm_rows:
    fields = list(norm_rows[0].keys())
    with normalized_out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(norm_rows)

groups = defaultdict(list)
for row in norm_rows:
    groups[(row["scan_id"], row["sensor"])].append(row)

mean_fields = [
    "scan_id",
    "scan_topk",
    "scan_allow_bits_drop",
    "sensor",
    "pass_runs",
    "ate_rmse_m_mean",
    "rpe_rmse_m_mean",
    "completeness_percent_mean",
    "runtime_sec_mean",
    "orb3_runtime_sec_mean",
    "runtime_ratio_vs_orb3_mean",
    "runtime_delta_percent_vs_orb3_mean",
]
with means_out.open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=mean_fields)
    w.writeheader()
    for key in sorted(groups):
        rs = groups[key]
        def mean_float(name):
            vals = [float(r[name]) for r in rs if r.get(name) not in ("", "NA")]
            return statistics.mean(vals) if vals else float("nan")
        w.writerow({
            "scan_id": key[0],
            "scan_topk": rs[0].get("scan_topk", ""),
            "scan_allow_bits_drop": rs[0].get("scan_allow_bits_drop", ""),
            "sensor": key[1],
            "pass_runs": len(rs),
            "ate_rmse_m_mean": f"{mean_float('ate_rmse_m'):.6f}",
            "rpe_rmse_m_mean": f"{mean_float('rpe_rmse_m'):.6f}",
            "completeness_percent_mean": f"{mean_float('completeness_percent'):.3f}",
            "runtime_sec_mean": f"{mean_float('runtime_sec'):.3f}",
            "orb3_runtime_sec_mean": f"{mean_float('orb3_runtime_sec'):.3f}",
            "runtime_ratio_vs_orb3_mean": f"{mean_float('runtime_ratio_vs_orb3'):.6f}",
            "runtime_delta_percent_vs_orb3_mean": f"{mean_float('runtime_delta_percent_vs_orb3'):.3f}",
        })
PY
}

log "Exp.4 EuRoC TopK/Drop one-run scan started"
log "SCAN_TAG=$SCAN_TAG"
log "SEQUENCES=$SEQUENCES"
log "SENSORS=$SENSORS"
log "RUNS_PER_CASE=$RUNS_PER_CASE"
log "MAX_FRAMES=$MAX_FRAMES"
log "SCAN_SUMMARY=$SCAN_SUMMARY"
log "SCAN_NORMALIZED=$SCAN_NORMALIZED"
log "SCAN_MEANS=$SCAN_MEANS"

run_baseline_once

run_idvo_config "K200_D2p0" "200" "2.0"
run_idvo_config "K200_D2p4" "200" "2.4"
run_idvo_config "K250_D2p0" "250" "2.0"
run_idvo_config "K250_D2p4" "250" "2.4"
run_idvo_config "K250_D2p8" "250" "2.8"
run_idvo_config "K300_D2p0" "300" "2.0"
run_idvo_config "K300_D2p4" "300" "2.4"
run_idvo_config "K300_D2p8" "300" "2.8"
run_idvo_config "K350_D2p0" "350" "2.0"
run_idvo_config "K350_D2p4" "350" "2.4"
run_idvo_config "K350_D2p8" "350" "2.8"
run_idvo_config "K400_D2p4" "400" "2.4"

build_normalized_tables

log "============================================================"
log "Exp.4 EuRoC TopK/Drop one-run scan finished"
log "Merged scan summary: $SCAN_SUMMARY"
log "Normalized rows: $SCAN_NORMALIZED"
log "Mean table: $SCAN_MEANS"
log "Generated configs: $SCAN_CONFIG_ROOT"
log "Log: $SCAN_LOG"

column -s, -t "$SCAN_MEANS" | tee -a "$SCAN_LOG" || cat "$SCAN_MEANS" | tee -a "$SCAN_LOG"

