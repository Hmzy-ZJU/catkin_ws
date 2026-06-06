#!/usr/bin/env bash
set -o pipefail

# Full UW-AIDVO dataset and ablation runner.
# Runs EuRoC, Tank, and Harbor with AIDVO off/fixed/rule.
#
# Ubuntu usage:
#   cd ~/catkin_ws
#   bash test_aidvo_full_20260607.sh
#
# Useful overrides:
#   DO_BUILD=0 bash test_aidvo_full_20260607.sh
#   BAG_DURATION=30 bash test_aidvo_full_20260607.sh
#   AIDVO_MODES=off,fixed,rule bash test_aidvo_full_20260607.sh
#   ONLY_DATASET=tank ONLY_SENSOR=stereo bash test_aidvo_full_20260607.sh
#   RUN_ALL_BAGS=1 bash test_aidvo_full_20260607.sh
#   EUROC_GT_FILE=/path/to/groundtruth.csv bash test_aidvo_full_20260607.sh

if [ -z "$WS" ]; then WS="$HOME/catkin_ws"; fi
if [ -z "$DO_BUILD" ]; then DO_BUILD=1; fi
if [ -z "$BUILD_TOOL" ]; then BUILD_TOOL=auto; fi
if [ -z "$AIDVO_MODES" ]; then AIDVO_MODES="off fixed rule"; fi
AIDVO_MODES="$(printf '%s' "$AIDVO_MODES" | tr ',' ' ')"
if [ -z "$BAG_DURATION" ]; then BAG_DURATION=0; fi
if [ -z "$RUN_ALL_BAGS" ]; then RUN_ALL_BAGS=0; fi
if [ -z "$TANK_MONO_INERTIAL_MIN_DURATION" ]; then TANK_MONO_INERTIAL_MIN_DURATION=120; fi
if [ -z "$BAG_RATE" ]; then BAG_RATE=1.0; fi
if [ -z "$RUN_TIMEOUT" ]; then RUN_TIMEOUT=7200; fi
if [ -z "$STARTUP_WAIT" ]; then STARTUP_WAIT=8; fi
if [ -z "$ENABLE_ADAPTIVE_LOGGING" ]; then ENABLE_ADAPTIVE_LOGGING=1; fi
if [ -z "$RUN_TAG" ]; then RUN_TAG="$(date +%Y%m%d_%H%M%S)"; fi

ROS_HOME_DIR="${ROS_HOME:-$HOME/.ros}"
GLOBAL_RESULT_ROOT="$WS/results/aidvo_full_${RUN_TAG}"
GLOBAL_LOG="$GLOBAL_RESULT_ROOT/test_aidvo_full.log"
GLOBAL_SUMMARY="$GLOBAL_RESULT_ROOT/test_aidvo_full_summary.csv"

mkdir -p "$GLOBAL_RESULT_ROOT"
: > "$GLOBAL_LOG"
exec > >(tee -a "$GLOBAL_LOG") 2>&1

ts() { date "+%Y-%m-%d %H:%M:%S"; }
log() { echo "[$(ts)] $*"; }

csv_escape() {
  local value="${1:-}"
  value="${value//\"/\"\"}"
  printf '"%s"' "$value"
}

write_csv_row() {
  local file="$1"
  shift
  local first=1
  {
    for value in "$@"; do
      if [ "$first" = "1" ]; then first=0; else printf ','; fi
      csv_escape "$value"
    done
    printf '\n'
  } >> "$file"
}

dataset_root() {
  case "$1" in
    euroc) if [ -n "$EUROC_ROOT" ]; then echo "$EUROC_ROOT"; else echo "$WS/dataset_EuRoc"; fi ;;
    tank) if [ -n "$TANK_ROOT" ]; then echo "$TANK_ROOT"; else echo "$WS/dataset_tank"; fi ;;
    harbor) if [ -n "$HARBOR_ROOT" ]; then echo "$HARBOR_ROOT"; else echo "$WS/dataset_harbor"; fi ;;
  esac
}

bag_override() {
  case "$1" in
    euroc) echo "$EUROC_BAG" ;;
    tank) echo "$TANK_BAG" ;;
    harbor) echo "$HARBOR_BAG" ;;
  esac
}

first_bag() {
  [ -d "$1/data" ] || return 0
  find "$1/data" -maxdepth 1 -type f -name "*.bag" | sort | head -n 1
}

all_bags() {
  [ -d "$1/data" ] || return 0
  find "$1/data" -maxdepth 1 -type f -name "*.bag" | sort
}

duration_less_than() {
  awk -v a="$1" -v b="$2" 'BEGIN { exit !((a + 0) > 0 && (a + 0) < (b + 0)) }'
}

bag_duration_sec() {
  python3 - "$1" <<'PY'
import sys
try:
    import rosbag
    bag = rosbag.Bag(sys.argv[1])
    print(max(0.0, bag.get_end_time() - bag.get_start_time()))
    bag.close()
except Exception:
    print(0.0)
PY
}

case_play_duration() {
  local bag="$1"
  local requested="$2"
  local actual
  actual="$(bag_duration_sec "$bag")"
  python3 - "$requested" "$actual" <<'PY'
import sys
requested = float(sys.argv[1])
actual = float(sys.argv[2])
if requested <= 0:
    print(f"{actual:.6f}")
elif actual > 0:
    print(f"{min(requested, actual):.6f}")
else:
    print(f"{requested:.6f}")
PY
}

gt_file_for_case() {
  local dataset="$1"
  local root="$2"
  local bag="$3"
  local env_name value found bag_name seq seq_pad
  case "$dataset" in
    euroc) env_name="EUROC_GT_FILE" ;;
    tank) env_name="TANK_GT_FILE" ;;
    harbor) env_name="HARBOR_GT_FILE" ;;
    *) env_name="" ;;
  esac
  if [ -n "$env_name" ]; then
    eval "value=\${$env_name:-}"
    if [ -n "$value" ] && [ -f "$value" ]; then echo "$value"; return 0; fi
  fi

  bag_name="$(basename "$bag" .bag)"
  case "$dataset" in
    euroc)
      found="$root/GT/${bag_name}.csv"
      [ -f "$found" ] && { echo "$found"; return 0; }
      ;;
    tank)
      found="$root/GT/${bag_name}_gt.csv"
      [ -f "$found" ] && { echo "$found"; return 0; }
      ;;
    harbor)
      seq="$(printf '%s' "$bag_name" | sed -n 's/^harbor_sequence_\([0-9][0-9]*\)$/\1/p')"
      if [ -n "$seq" ]; then
        seq_pad="$(printf '%02d' "$seq")"
        found="$root/GT/harbor_colmap_traj_sequence_${seq_pad}.txt"
        [ -f "$found" ] && { echo "$found"; return 0; }
      fi
      ;;
  esac

  found="$(find "$root" -maxdepth 3 -type f \( -iname "*groundtruth*.tum" -o -iname "*gt*.tum" -o -iname "*truth*.txt" -o -iname "*gt*.csv" -o -iname "*.csv" \) 2>/dev/null | sort | head -n 1)"
  if [ -n "$found" ]; then echo "$found"; fi
}

prepare_aidvo_config() {
  local base_config="$1"
  local output_config="$2"
  local adaptive_log="$3"
  local mode="$4"
  local enable=1
  local policy="RuleBased"

  case "$mode" in
    off) enable=0; policy="Fixed" ;;
    fixed) enable=1; policy="Fixed" ;;
    rule) enable=1; policy="RuleBased" ;;
    *) log "[ERROR] invalid AIDVO mode: $mode"; return 1 ;;
  esac

  awk '
    !/^[[:space:]]*(EnableAdaptiveIDVO|AdaptivePolicyType|MinKappaTop|MaxKappaTop|MinTau0|MaxTau0|TrackingTimeBudget|SmoothFactor|EnableAdaptiveLogging|AdaptiveLogPath|Adaptive\.LowLogDetH|Adaptive\.PoorConditionNumber|Adaptive\.LowInlierRatio|Adaptive\.BlurThreshold)[[:space:]]*:/
  ' "$base_config" > "$output_config"

  cat >> "$output_config" <<EOF

# Generated by test_aidvo_full_20260607.sh
EnableAdaptiveIDVO: ${enable}
AdaptivePolicyType: "${policy}"
MinKappaTop: ${MIN_KAPPA_TOP:-60}
MaxKappaTop: ${MAX_KAPPA_TOP:-180}
MinTau0: ${MIN_TAU0:-0.1}
MaxTau0: ${MAX_TAU0:-5.0}
TrackingTimeBudget: ${TRACKING_TIME_BUDGET:-30.0}
SmoothFactor: ${SMOOTH_FACTOR:-0.8}
EnableAdaptiveLogging: ${ENABLE_ADAPTIVE_LOGGING:-1}
AdaptiveLogPath: "${adaptive_log}"
EOF
}

wait_for_orb_slam3_node() {
  local launch_pid="$1"
  local launch_log="$2"
  local deadline=$((SECONDS + STARTUP_WAIT))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ! kill -0 "$launch_pid" >/dev/null 2>&1; then
      log "[ERROR] roslaunch exited during startup. See $launch_log"
      tail -n 80 "$launch_log" || true
      return 1
    fi
    if rosnode list 2>/dev/null | grep -q '^/orb_slam3$'; then return 0; fi
    sleep 1
  done
  log "[ERROR] /orb_slam3 did not appear within ${STARTUP_WAIT}s"
  tail -n 80 "$launch_log" || true
  return 1
}

wait_for_launch_log_pattern() {
  local launch_pid="$1"
  local launch_log="$2"
  local pattern="$3"
  local deadline=$((SECONDS + STARTUP_WAIT))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if grep -q "$pattern" "$launch_log"; then return 0; fi
    if ! kill -0 "$launch_pid" >/dev/null 2>&1; then
      log "[ERROR] roslaunch exited before expected log appeared: $pattern"
      tail -n 80 "$launch_log" || true
      return 1
    fi
    sleep 1
  done
  log "[ERROR] missing expected startup log: $pattern"
  tail -n 80 "$launch_log" || true
  return 1
}

evaluate_case() {
  local dataset="$1"
  local sensor="$2"
  local mode="$3"
  local bag="$4"
  local played_duration="$5"
  local run_dir="$6"
  local adaptive_csv="$7"
  local traj_file="$8"
  local gt_file="$9"
  local eval_csv="${run_dir}/evaluation_metrics.csv"

  python3 - "$dataset" "$sensor" "$mode" "$bag" "$played_duration" "$adaptive_csv" "$traj_file" "$gt_file" "$eval_csv" <<'PY'
import csv
import math
import os
import re
import sys
from statistics import mean

dataset, sensor, mode, bag, played, adaptive_csv, traj_file, gt_file, out_csv = sys.argv[1:10]
played = float(played or 0.0)

def safe_float(value, default=0.0):
    try:
        if value is None or value == "":
            return default
        v = float(value)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return default

def read_adaptive(path):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def normalize_stamp(value):
    value = float(value)
    if abs(value) > 1e12:
        return value * 1e-9
    return value

def numeric_values(line):
    return [float(x) for x in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", line)]

def read_trajectory(path):
    rows = []
    if not path or not os.path.isfile(path):
        return rows
    with open(path) as f:
        index = 0
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            nums = numeric_values(line)
            if len(nums) < 3:
                continue
            try:
                if len(nums) >= 8:
                    stamp = normalize_stamp(nums[0])
                    xyz = nums[1:4]
                elif len(nums) == 7:
                    # COLMAP-style rows often contain qx/qy/qz/qw + tx/ty/tz without timestamps.
                    stamp = float(index)
                    xyz = nums[4:7]
                elif len(nums) >= 4:
                    stamp = normalize_stamp(nums[0])
                    xyz = nums[1:4]
                else:
                    stamp = float(index)
                    xyz = nums[0:3]
                rows.append((stamp, float(xyz[0]), float(xyz[1]), float(xyz[2])))
                index += 1
            except Exception:
                pass
    return rows

def hz_from_times(times):
    if len(times) < 2:
        return 0.0, 0.0
    span = max(times) - min(times)
    if span <= 0:
        return 0.0, span
    return len(times) / span, span

def associate(gt, est, max_dt=0.05):
    if not gt or not est:
        return []
    pairs = []
    j = 0
    for et, ex, ey, ez in est:
        while j + 1 < len(gt) and abs(gt[j + 1][0] - et) <= abs(gt[j][0] - et):
            j += 1
        if abs(gt[j][0] - et) <= max_dt:
            pairs.append(((ex, ey, ez), (gt[j][1], gt[j][2], gt[j][3])))
    if len(pairs) < 3:
        n = min(len(gt), len(est))
        pairs = [((est[i][1], est[i][2], est[i][3]), (gt[i][1], gt[i][2], gt[i][3])) for i in range(n)]
    return pairs

def umeyama_align(est_pts, gt_pts):
    import numpy as np
    p = np.asarray(est_pts, dtype=float)
    q = np.asarray(gt_pts, dtype=float)
    if len(p) < 3:
        return None
    mu_p = p.mean(axis=0)
    mu_q = q.mean(axis=0)
    x = p - mu_p
    y = q - mu_q
    sigma = (y.T @ x) / len(p)
    u, d, vt = np.linalg.svd(sigma)
    s_mat = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        s_mat[2, 2] = -1.0
    r = u @ s_mat @ vt
    var_p = np.mean(np.sum(x * x, axis=1))
    if var_p <= 1e-12:
        return None
    scale = np.trace(np.diag(d) @ s_mat) / var_p
    t = mu_q - scale * (r @ mu_p)
    aligned = (scale * (r @ p.T)).T + t
    return aligned

adaptive = read_adaptive(adaptive_csv)
times = [safe_float(r.get("timestamp")) for r in adaptive if safe_float(r.get("timestamp")) > 0]
output_hz, adaptive_span = hz_from_times(times)
traj = read_trajectory(traj_file)
traj_times = [r[0] for r in traj]
traj_hz, traj_span = hz_from_times(traj_times)
completeness = 0.0
if played > 0:
    completeness = max(0.0, min(100.0, 100.0 * traj_span / played))

def col(name):
    return [safe_float(r.get(name)) for r in adaptive]

candidates = col("candidate_point_number")
selected = col("selected_point_number")
selection_ratio = col("selection_ratio")
kappa = col("kappa_top")
tau0 = col("tau0")
track_ms = col("tracking_time_ms")
tracked = col("tracked_map_points")
lost = col("tracking_lost_flag")
keyframe = col("keyframe_insertion_flag")

positive_candidates = [v for v in candidates if v > 0]
positive_selected = [v for v in selected if v > 0]
positive_tracked = [v for v in tracked if v > 0]
status = "PASS" if adaptive and (traj or adaptive_span > 0) else "EVAL_WARN"

ate_rmse = ""
rpe_rmse = ""
gt = read_trajectory(gt_file)
if gt and traj:
    pairs = associate(gt, traj)
    if len(pairs) >= 3:
        est_pts = [p[0] for p in pairs]
        gt_pts = [p[1] for p in pairs]
        aligned = umeyama_align(est_pts, gt_pts)
        if aligned is not None:
            import numpy as np
            q = np.asarray(gt_pts, dtype=float)
            err = aligned - q
            ate_rmse = f"{math.sqrt(float(np.mean(np.sum(err * err, axis=1)))):.6f}"
            if len(aligned) >= 2:
                de = np.diff(aligned, axis=0)
                dg = np.diff(q, axis=0)
                rpe = de - dg
                rpe_rmse = f"{math.sqrt(float(np.mean(np.sum(rpe * rpe, axis=1)))):.6f}"

metrics = {
    "dataset": dataset,
    "sensor": sensor,
    "aidvo_mode": mode,
    "bag": bag,
    "status": status,
    "adaptive_rows": len(adaptive),
    "adaptive_duration_sec": f"{adaptive_span:.6f}",
    "output_hz": f"{output_hz:.6f}",
    "trajectory_file": traj_file if traj_file and os.path.isfile(traj_file) else "",
    "trajectory_poses": len(traj),
    "trajectory_duration_sec": f"{traj_span:.6f}",
    "trajectory_hz": f"{traj_hz:.6f}",
    "trajectory_completeness_percent": f"{completeness:.3f}",
    "idps_frames": sum(1 for v in candidates if v > 0),
    "avg_candidates": f"{mean(positive_candidates):.3f}" if positive_candidates else "0.000",
    "avg_selected": f"{mean(positive_selected):.3f}" if positive_selected else "0.000",
    "avg_selection_ratio": f"{mean([v for v in selection_ratio if v > 0]):.6f}" if any(v > 0 for v in selection_ratio) else "0.000000",
    "avg_tracked_map_points": f"{mean(positive_tracked):.3f}" if positive_tracked else "0.000",
    "mean_tracking_time_ms": f"{mean(track_ms):.3f}" if track_ms else "0.000",
    "tracking_lost_frames": sum(1 for v in lost if int(v) == 1),
    "keyframe_insertions": sum(1 for v in keyframe if int(v) == 1),
    "kappa_unique": len(set(kappa)),
    "tau0_unique": len(set(tau0)),
    "gt_file": gt_file if gt_file and os.path.isfile(gt_file) else "",
    "ate_rmse_m": ate_rmse,
    "rpe_rmse_m": rpe_rmse,
}

fields = list(metrics.keys())
with open(out_csv, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerow(metrics)
print(",".join(str(metrics[k]) for k in fields))
PY
}

summary_header() {
  local file="$1"
  echo "dataset,sensor,aidvo_mode,bag,status,exit_code,elapsed_sec,played_duration_sec,result_dir,adaptive_csv,trajectory_file,roslaunch_log,validation,adaptive_rows,output_hz,trajectory_poses,trajectory_hz,trajectory_completeness_percent,idps_frames,avg_candidates,avg_selected,avg_selection_ratio,avg_tracked_map_points,mean_tracking_time_ms,kappa_unique,tau0_unique,ate_rmse_m,rpe_rmse_m" > "$file"
}

read_eval_value() {
  local file="$1"
  local key="$2"
  python3 - "$file" "$key" <<'PY'
import csv
import sys
path, key = sys.argv[1:3]
try:
    rows = list(csv.DictReader(open(path, newline="")))
    print(rows[0].get(key, "") if rows else "")
except Exception:
    print("")
PY
}

validate_behavior() {
  local mode="$1"
  local launch_log="$2"
  local eval_csv="$3"
  local errors=""
  local notes=""
  local expected_switch="YES"
  local expected_policy="Fixed"
  local kappa_unique tau0_unique output_hz completeness

  if [ "$mode" = "off" ]; then expected_switch="NO"; fi
  if [ "$mode" = "rule" ]; then expected_policy="RuleBased"; fi

  if [ ! -f "$launch_log" ]; then
    errors="$errors missing_launch_log"
  else
    grep -q "Adaptive IDVO params loaded" "$launch_log" || errors="$errors config_not_loaded"
    grep -q "EnableAdaptiveIDVO: ${expected_switch}" "$launch_log" || errors="$errors wrong_adaptive_switch"
    grep -q "AdaptivePolicyType: ${expected_policy}" "$launch_log" || errors="$errors wrong_policy"
  fi

  if [ ! -f "$eval_csv" ]; then
    errors="$errors missing_eval_csv"
  else
    kappa_unique="$(read_eval_value "$eval_csv" kappa_unique)"
    tau0_unique="$(read_eval_value "$eval_csv" tau0_unique)"
    output_hz="$(read_eval_value "$eval_csv" output_hz)"
    completeness="$(read_eval_value "$eval_csv" trajectory_completeness_percent)"
    notes="$notes output_hz=${output_hz} completeness=${completeness}% kappa_unique=${kappa_unique} tau0_unique=${tau0_unique}"

    if [ "$mode" = "off" ] || [ "$mode" = "fixed" ]; then
      if [ "${kappa_unique:-0}" -gt 1 ] || [ "${tau0_unique:-0}" -gt 1 ]; then
        errors="$errors fixed_params_changed"
      fi
    fi
    if [ "$mode" = "rule" ]; then
      if [ "${kappa_unique:-0}" -le 1 ] && [ "${tau0_unique:-0}" -le 1 ]; then
        errors="$errors rule_params_not_changed"
      fi
    fi
  fi

  if [ -z "$errors" ]; then
    echo "PASS${notes}"
  else
    echo "FAIL errors=${errors} notes=${notes}"
  fi
}

append_summary_from_eval() {
  local file="$1"
  local dataset="$2"
  local sensor="$3"
  local mode="$4"
  local bag="$5"
  local status="$6"
  local exit_code="$7"
  local elapsed="$8"
  local played_duration="$9"
  shift 9
  local run_dir="$1"
  local adaptive_csv="$2"
  local trajectory_file="$3"
  local launch_log="$4"
  local validation="$5"
  local eval_csv="$run_dir/evaluation_metrics.csv"

  write_csv_row "$file" \
    "$dataset" "$sensor" "$mode" "$bag" "$status" "$exit_code" "$elapsed" "$played_duration" \
    "$run_dir" "$adaptive_csv" "$trajectory_file" "$launch_log" "$validation" \
    "$(read_eval_value "$eval_csv" adaptive_rows)" \
    "$(read_eval_value "$eval_csv" output_hz)" \
    "$(read_eval_value "$eval_csv" trajectory_poses)" \
    "$(read_eval_value "$eval_csv" trajectory_hz)" \
    "$(read_eval_value "$eval_csv" trajectory_completeness_percent)" \
    "$(read_eval_value "$eval_csv" idps_frames)" \
    "$(read_eval_value "$eval_csv" avg_candidates)" \
    "$(read_eval_value "$eval_csv" avg_selected)" \
    "$(read_eval_value "$eval_csv" avg_selection_ratio)" \
    "$(read_eval_value "$eval_csv" avg_tracked_map_points)" \
    "$(read_eval_value "$eval_csv" mean_tracking_time_ms)" \
    "$(read_eval_value "$eval_csv" kappa_unique)" \
    "$(read_eval_value "$eval_csv" tau0_unique)" \
    "$(read_eval_value "$eval_csv" ate_rmse_m)" \
    "$(read_eval_value "$eval_csv" rpe_rmse_m)"
}

run_case() {
  local dataset="$1"
  local sensor="$2"
  local launch_name="$3"
  local base_config="$4"
  local mode="$5"
  local root override bag ran

  root="$(dataset_root "$dataset")"
  override="$(bag_override "$dataset")"
  if [ -n "$override" ]; then
    run_case_one "$dataset" "$sensor" "$launch_name" "$base_config" "$mode" "$override"
    return $?
  fi

  if [ "$RUN_ALL_BAGS" = "1" ]; then
    ran=0
    while IFS= read -r bag; do
      [ -n "$bag" ] || continue
      ran=1
      run_case_one "$dataset" "$sensor" "$launch_name" "$base_config" "$mode" "$bag"
    done < <(all_bags "$root")
    if [ "$ran" = "0" ]; then
      run_case_one "$dataset" "$sensor" "$launch_name" "$base_config" "$mode" ""
    fi
    return 0
  fi

  run_case_one "$dataset" "$sensor" "$launch_name" "$base_config" "$mode" "$(first_bag "$root")"
}

run_case_one() {
  local dataset="$1"
  local sensor="$2"
  local launch_name="$3"
  local base_config="$4"
  local mode="$5"
  local forced_bag="$6"
  local root bag bag_name dataset_results run_dir adaptive_csv launch_log generated_config
  local start end elapsed exit_code status validation case_duration played_duration save_prefix traj_file gt_file

  root="$(dataset_root "$dataset")"
  bag="$forced_bag"
  dataset_results="$root/results/aidvo_full_${RUN_TAG}"
  mkdir -p "$dataset_results"

  if [ ! -f "$bag" ]; then
    log "[SKIP] missing bag for $dataset: $bag"
    write_csv_row "$dataset_results/summary_${dataset}.csv" "$dataset" "$sensor" "$mode" "$bag" "SKIP" "0" "0" "0" "" "" "" "" "missing_bag" "" "" "" "" "" "" "" "" "" "" "" "" "" "" ""
    write_csv_row "$GLOBAL_SUMMARY" "$dataset" "$sensor" "$mode" "$bag" "SKIP" "0" "0" "0" "" "" "" "" "missing_bag" "" "" "" "" "" "" "" "" "" "" "" "" "" "" ""
    return 0
  fi
  if [ ! -f "$base_config" ]; then
    log "[SKIP] missing config: $base_config"
    write_csv_row "$dataset_results/summary_${dataset}.csv" "$dataset" "$sensor" "$mode" "$bag" "SKIP" "0" "0" "0" "" "" "" "" "missing_config" "" "" "" "" "" "" "" "" "" "" "" "" "" "" ""
    write_csv_row "$GLOBAL_SUMMARY" "$dataset" "$sensor" "$mode" "$bag" "SKIP" "0" "0" "0" "" "" "" "" "missing_config" "" "" "" "" "" "" "" "" "" "" "" "" "" "" ""
    return 0
  fi

  bag_name="$(basename "$bag" .bag)"
  run_dir="$dataset_results/$mode/$sensor/$bag_name"
  mkdir -p "$run_dir"
  adaptive_csv="$run_dir/adaptive_frames.csv"
  launch_log="$run_dir/roslaunch.log"
  generated_config="$run_dir/settings_${mode}.yaml"
  save_prefix="aidvo_full_${dataset}_${sensor}_${mode}_${bag_name}"
  traj_file="$run_dir/${save_prefix}_cam_traj.txt"
  rm -f "$adaptive_csv" "$launch_log" "$traj_file"
  prepare_aidvo_config "$base_config" "$generated_config" "$adaptive_csv" "$mode" || return 1

  case_duration="$BAG_DURATION"
  if [ "$dataset" = "tank" ] && [ "$sensor" = "mono-inertial" ] && \
     duration_less_than "$case_duration" "$TANK_MONO_INERTIAL_MIN_DURATION"; then
    case_duration="$TANK_MONO_INERTIAL_MIN_DURATION"
  fi
  played_duration="$(case_play_duration "$bag" "$case_duration")"

  log "============================================================"
  log "[RUN] dataset=$dataset sensor=$sensor aidvo_mode=$mode"
  log "[RUN] bag=$bag"
  log "[RUN] launch=$launch_name"
  log "[RUN] config=$generated_config"
  log "[RUN] result_dir=$run_dir"
  log "[RUN] bag_duration=$case_duration played_duration_est=$played_duration"
  log "============================================================"

  start="$(date +%s)"
  roslaunch orb_slam3_ros "$launch_name" voc_file:="$WS/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin" settings_file:="$generated_config" > "$launch_log" 2>&1 &
  launch_pid=$!

  cleanup_case() {
    rosnode kill /orb_slam3 >/dev/null 2>&1 || true
    kill "$launch_pid" >/dev/null 2>&1 || true
    wait "$launch_pid" 2>/dev/null || true
  }
  trap cleanup_case EXIT INT TERM

  exit_code=0
  wait_for_orb_slam3_node "$launch_pid" "$launch_log" || exit_code=1
  if [ "$exit_code" = "0" ]; then
    wait_for_launch_log_pattern "$launch_pid" "$launch_log" "Adaptive IDVO params loaded" || exit_code=1
  fi
  if [ "$exit_code" = "0" ]; then
    play_args=(play "$bag" --clock -r "$BAG_RATE")
    if [ "$case_duration" != "0" ]; then play_args+=(--duration "$case_duration"); fi
    timeout --preserve-status "$RUN_TIMEOUT" rosbag "${play_args[@]}"
    exit_code=$?
  fi
  sleep 3
  if [ "$exit_code" = "0" ] && rosservice info /orb_slam3/save_traj >/dev/null 2>&1; then
    rosservice call /orb_slam3/save_traj "$save_prefix" || true
  fi
  cleanup_case
  trap - EXIT INT TERM
  end="$(date +%s)"
  elapsed="$((end - start))"

  for f in "$ROS_HOME_DIR/${save_prefix}"*; do
    [ -f "$f" ] || continue
    cp -f "$f" "$run_dir/"
  done
  if [ ! -f "$traj_file" ]; then
    found_traj="$(find "$run_dir" -maxdepth 1 -type f -name "${save_prefix}*cam_traj*.txt" | sort | head -n 1)"
    if [ -n "$found_traj" ]; then traj_file="$found_traj"; fi
  fi

  if [ "$exit_code" = "124" ]; then
    status="TIMEOUT"
    validation="timeout"
  elif [ "$exit_code" != "0" ]; then
    status="RUN_FAIL"
    validation="run_exit_${exit_code}"
  elif [ ! -s "$adaptive_csv" ]; then
    status="VALIDATION_FAIL"
    validation="missing_or_empty_adaptive_csv"
  else
    status="PASS"
    validation="run_completed"
  fi

  gt_file="$(gt_file_for_case "$dataset" "$root" "$bag")"
  evaluate_case "$dataset" "$sensor" "$mode" "$bag" "$played_duration" "$run_dir" "$adaptive_csv" "$traj_file" "$gt_file" > "$run_dir/evaluation_stdout.txt" || true
  if [ "$status" = "PASS" ]; then
    validation="$(validate_behavior "$mode" "$launch_log" "$run_dir/evaluation_metrics.csv")"
    if ! echo "$validation" | grep -q '^PASS'; then
      status="VALIDATION_FAIL"
    fi
  fi
  append_summary_from_eval "$dataset_results/summary_${dataset}.csv" "$dataset" "$sensor" "$mode" "$bag" "$status" "$exit_code" "$elapsed" "$played_duration" "$run_dir" "$adaptive_csv" "$traj_file" "$launch_log" "$validation"
  append_summary_from_eval "$GLOBAL_SUMMARY" "$dataset" "$sensor" "$mode" "$bag" "$status" "$exit_code" "$elapsed" "$played_duration" "$run_dir" "$adaptive_csv" "$traj_file" "$launch_log" "$validation"

  log "[RESULT] dataset=$dataset sensor=$sensor mode=$mode status=$status elapsed=$elapsed"
  log "[EVAL] output_hz=$(read_eval_value "$run_dir/evaluation_metrics.csv" output_hz) trajectory_hz=$(read_eval_value "$run_dir/evaluation_metrics.csv" trajectory_hz) completeness=$(read_eval_value "$run_dir/evaluation_metrics.csv" trajectory_completeness_percent)%"
}

unsupported_case() {
  local dataset="$1"
  local sensor="$2"
  local mode="$3"
  local reason="$4"
  local root dataset_results
  root="$(dataset_root "$dataset")"
  dataset_results="$root/results/aidvo_full_${RUN_TAG}"
  mkdir -p "$dataset_results"
  write_csv_row "$dataset_results/summary_${dataset}.csv" "$dataset" "$sensor" "$mode" "" "UNSUPPORTED" "0" "0" "0" "" "" "" "" "$reason" "" "" "" "" "" "" "" "" "" "" "" "" "" "" ""
  write_csv_row "$GLOBAL_SUMMARY" "$dataset" "$sensor" "$mode" "" "UNSUPPORTED" "0" "0" "0" "" "" "" "" "$reason" "" "" "" "" "" "" "" "" "" "" "" "" "" "" ""
}

build_workspace() {
  if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi
  if [ "$DO_BUILD" != "1" ]; then
    log "[BUILD] skipped"
    return 0
  fi

  local build_tool="$BUILD_TOOL"
  if [ "$build_tool" = "auto" ]; then
    if [ -d "$WS/.catkin_tools" ] || command -v catkin >/dev/null 2>&1; then
      build_tool="catkin_build"
    else
      build_tool="catkin_make"
    fi
  fi

  if [ "$build_tool" = "catkin_build" ]; then
    log "[BUILD] catkin build --cmake-args -DORB3_USE_INFOSEL=ON"
    catkin build --cmake-args -DORB3_USE_INFOSEL=ON
  elif [ "$build_tool" = "catkin_make" ]; then
    log "[BUILD] catkin_make -DORB3_USE_INFOSEL=ON"
    catkin_make -DORB3_USE_INFOSEL=ON
  else
    log "[ERROR] BUILD_TOOL must be auto, catkin_build, or catkin_make"
    return 1
  fi
}

preflight() {
  log "test_aidvo_full_20260607 started"
  log "WS=$WS"
  log "RUN_TAG=$RUN_TAG"
  log "AIDVO_MODES=$AIDVO_MODES"
  log "BAG_DURATION=$BAG_DURATION"
  log "RUN_ALL_BAGS=$RUN_ALL_BAGS"
  log "TANK_MONO_INERTIAL_MIN_DURATION=$TANK_MONO_INERTIAL_MIN_DURATION"
  log "GLOBAL_RESULT_ROOT=$GLOBAL_RESULT_ROOT"
  cd "$WS" || return 1
  command -v python3 >/dev/null || return 1
  command -v timeout >/dev/null || return 1
  build_workspace || return 1
  [ -f "$WS/devel/setup.bash" ] || { log "[ERROR] missing $WS/devel/setup.bash"; return 1; }
  source "$WS/devel/setup.bash"
  command -v roslaunch >/dev/null || return 1
  command -v rosbag >/dev/null || return 1
  command -v rosnode >/dev/null || return 1
  command -v rosservice >/dev/null || return 1
  log "[PREFLIGHT] OK"
}

init_summaries() {
  summary_header "$GLOBAL_SUMMARY"
  for dataset in euroc tank harbor; do
    local root dataset_results
    root="$(dataset_root "$dataset")"
    dataset_results="$root/results/aidvo_full_${RUN_TAG}"
    mkdir -p "$dataset_results"
    summary_header "$dataset_results/summary_${dataset}.csv"
  done
}

main() {
  preflight || exit 1
  init_summaries

  while IFS='|' read -r dataset sensor launch_name base_config; do
    [ -z "$dataset" ] && continue
    if [ -n "$ONLY_DATASET" ] && [ "$dataset" != "$ONLY_DATASET" ]; then continue; fi
    if [ -n "$ONLY_SENSOR" ] && [ "$sensor" != "$ONLY_SENSOR" ]; then continue; fi
    for mode in $AIDVO_MODES; do
      if [ "$mode" != "off" ] && [ "$mode" != "fixed" ] && [ "$mode" != "rule" ]; then
        log "[WARN] invalid mode skipped: $mode"
        continue
      fi
      run_case "$dataset" "$sensor" "$launch_name" "$base_config" "$mode"
    done
  done <<CASES
euroc|mono|euroc_mono.launch|$WS/src/orb_slam3_ros/config/Monocular/EuRoc/EuRoc_on_11.yaml
euroc|stereo|euroc_stereo.launch|$WS/src/orb_slam3_ros/config/Stereo/EuRoC.yaml
euroc|mono-inertial|euroc_mono_inertial.launch|$WS/src/orb_slam3_ros/config/Monocular-Inertial/EuRoC.yaml
euroc|stereo-inertial|euroc_stereo_inertial.launch|$WS/src/orb_slam3_ros/config/Stereo-Inertial/EuRoC.yaml
tank|mono|tank_mono.launch|$WS/src/orb_slam3_ros/config/Monocular/Tank/tank_on_11.yaml
tank|stereo|tank_stereo.launch|$WS/src/orb_slam3_ros/config/Stereo/Tank/Tank_stereo_on_11.yaml
tank|mono-inertial|tank_mono_inertial.launch|$WS/src/orb_slam3_ros/config/Monocular-Inertial/Tank/tank_on_11.yaml
tank|stereo-inertial|tank_stereo_inertial.launch|$WS/src/orb_slam3_ros/config/Stereo-Inertial/Tank_stereo_inertial_on_11.yaml
harbor|mono|aqualoc_harbor_mono.launch|$WS/src/orb_slam3_ros/config/Monocular/Aquacular_harbor/all/Aqualoc_harbor_on_11.yaml
harbor|mono-inertial|aqualoc_harbor_mono_inertial.launch|$WS/src/orb_slam3_ros/config/Monocular-Inertial/Aqualoc_harbor.yaml
CASES

  while IFS='|' read -r dataset sensor reason; do
    [ -z "$dataset" ] && continue
    if [ -n "$ONLY_DATASET" ] && [ "$dataset" != "$ONLY_DATASET" ]; then continue; fi
    if [ -n "$ONLY_SENSOR" ] && [ "$sensor" != "$ONLY_SENSOR" ]; then continue; fi
    for mode in $AIDVO_MODES; do
      unsupported_case "$dataset" "$sensor" "$mode" "$reason"
    done
  done <<'UNSUPPORTED'
harbor|stereo|no existing stereo launch/config/script for Harbor in this project
harbor|stereo-inertial|no existing stereo-inertial launch/config/script for Harbor in this project
UNSUPPORTED

  log "============================================================"
  log "test_aidvo_full_20260607 finished"
  log "Global summary: $GLOBAL_SUMMARY"
  python3 - "$GLOBAL_SUMMARY" <<'PY'
import csv
import sys
from collections import Counter
rows = list(csv.DictReader(open(sys.argv[1], newline="")))
counter = Counter(r["status"] for r in rows)
print("Status counts:")
for key in sorted(counter):
    print(f"  {key}: {counter[key]}")
PY
  log "============================================================"
}

main "$@"
