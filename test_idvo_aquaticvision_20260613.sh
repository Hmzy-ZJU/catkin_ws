#!/usr/bin/env bash
set -u

# AquaticVision IDVO/off test.
# This script uses ORB-SLAM3 offline EuRoC readers, not ROS playback.
# It prepares AquaticVision l1/r1 image folders into a EuRoC-like cache,
# runs mono/stereo with EnableAdaptiveIDVO=0, and saves evo evaluation plots.

WS="${WS:-$HOME/catkin_ws}"
AQUATIC_ROOT="${AQUATIC_ROOT:-$WS/dataset_AquaticVision}"
RUN_TAG="${RUN_TAG:-idvo_aquaticvision_$(date +%Y%m%d_%H%M%S)}"
SEQUENCES="${SEQUENCES:-01 02 03 04 05 06 07 08 09}"
SENSORS="${SENSORS:-mono stereo}"
RUNS_PER_CASE="${RUNS_PER_CASE:-1}"
DO_BUILD="${DO_BUILD:-0}"
MAX_FRAMES="${MAX_FRAMES:-0}"
FPS="${FPS:-30}"
TOPK="${TOPK:-95}"
EVO_TIMEOUT="${EVO_TIMEOUT:-300}"
EVO_CORRECT_SCALE="${EVO_CORRECT_SCALE:-1}"
MIN_TRAJECTORY_POSES="${MIN_TRAJECTORY_POSES:-20}"
COPY_IMAGES="${COPY_IMAGES:-0}"

VOCAB="${VOCAB:-$WS/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin}"
PREPARE_TOOL="$WS/tools/prepare_aquaticvision_euroc.py"
RESULT_ROOT="$AQUATIC_ROOT/results/idvo_aquaticvision_${RUN_TAG}"
CACHE_ROOT="${CACHE_ROOT:-$WS/offline_aquaticvision_cache}"
SUMMARY="$RESULT_ROOT/summary_aquaticvision_idvo.csv"
LOG_FILE="$RESULT_ROOT/idvo_aquaticvision.log"

mkdir -p "$RESULT_ROOT"
: > "$LOG_FILE"

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$LOG_FILE"
}

csv_quote() {
  printf '"%s"' "$(printf '%s' "$1" | sed 's/"/""/g')"
}

write_row() {
  local first=1 value
  for value in "$@"; do
    if [ "$first" = "1" ]; then first=0; else printf ',' >> "$SUMMARY"; fi
    csv_quote "$value" >> "$SUMMARY"
  done
  printf '\n' >> "$SUMMARY"
}

summary_header() {
  printf 'dataset,sequence,sensor,mode,run_id,status,exit_code,elapsed_sec,input_frames,trajectory_poses,completeness_percent,result_dir,settings_file,trajectory_file,gt_tum,evo_dir,ate_rmse_m,rpe_rmse_m,notes\n' > "$SUMMARY"
}

find_exe() {
  local name="$1"
  for p in \
    "$WS/devel/.private/orb_slam3_ros/lib/orb_slam3_ros/$name" \
    "$WS/devel/lib/orb_slam3_ros/$name" \
    "$WS/build/orb_slam3_ros/$name"; do
    if [ -x "$p" ]; then
      printf '%s\n' "$p"
      return 0
    fi
  done
  return 1
}

offline_exe() {
  case "$1" in
    mono) printf '%s\n' aidvo_offline_mono_euroc ;;
    stereo) printf '%s\n' aidvo_offline_stereo_euroc ;;
    *) return 1 ;;
  esac
}

count_lines() {
  local file="$1"
  [ -f "$file" ] || { printf '0\n'; return; }
  awk 'NF && $1 !~ /^#/ { c++ } END { print c+0 }' "$file"
}

extract_metric() {
  local file="$1"
  [ -f "$file" ] || { printf '\n'; return; }
  awk '/rmse/ {print $2; exit}' "$file"
}

prepare_sequence() {
  local seq="$1" cache="$2" mono_cfg="$3" stereo_cfg="$4"
  local copy_arg=""
  local baseline_arg=""
  if [ "$COPY_IMAGES" = "1" ]; then copy_arg="--copy-images"; fi
  if [ -n "${AQUATIC_BASELINE:-}" ]; then baseline_arg="--baseline $AQUATIC_BASELINE"; fi

  python3 "$PREPARE_TOOL" \
    --root "$AQUATIC_ROOT" \
    --sequence "$seq" \
    --out "$cache" \
    --mono-config "$mono_cfg" \
    --stereo-config "$stereo_cfg" \
    --fps "$FPS" \
    --max-frames "$MAX_FRAMES" \
    --topk "$TOPK" \
    $baseline_arg \
    $copy_arg
}

run_evo() {
  local gt="$1" traj="$2" evo_dir="$3"
  mkdir -p "$evo_dir"
  local scale_arg=""
  if [ "$EVO_CORRECT_SCALE" = "1" ]; then scale_arg="--correct_scale"; fi

  if [ ! -s "$gt" ]; then
    echo "EVO_SKIP missing gt.tum" > "$evo_dir/evo_status.txt"
    return 0
  fi
  if [ ! -s "$traj" ]; then
    echo "EVO_SKIP missing trajectory" > "$evo_dir/evo_status.txt"
    return 0
  fi
  if ! command -v evo_ape >/dev/null 2>&1 || ! command -v evo_rpe >/dev/null 2>&1 || ! command -v evo_traj >/dev/null 2>&1; then
    echo "EVO_SKIP evo tools not found. Install with: pip3 install evo" > "$evo_dir/evo_status.txt"
    return 0
  fi

  local gt_eval="$evo_dir/groundtruth_matched.tum"
  local traj_eval="$evo_dir/estimated_matched.tum"
  python3 - "$gt" "$traj" "$gt_eval" "$traj_eval" <<'PY' > "$evo_dir/match_timestamps_stdout.txt" 2>&1
import bisect
import sys

gt_in, est_in, gt_out, est_out = sys.argv[1:5]

def read_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            stamp = float(parts[0])
            if abs(stamp) > 1e12:
                stamp /= 1e9
            rows.append((stamp, parts))
    return rows

gt = read_tum(gt_in)
est = read_tum(est_in)
if len(gt) < 3 or len(est) < 3:
    raise SystemExit("not enough poses for evo matching")

gt_times = [r[0] for r in gt]
offset = gt_times[0] - est[0][0]
max_dt = max(0.05, 2.5 * (gt_times[min(20, len(gt_times)-1)] - gt_times[0]) / max(1, min(20, len(gt_times)-1)))
matches = []
for est_t, est_parts in est:
    target = est_t + offset
    j = bisect.bisect_left(gt_times, target)
    cand = []
    if j < len(gt): cand.append(j)
    if j > 0: cand.append(j - 1)
    if not cand:
        continue
    best = min(cand, key=lambda i: abs(gt_times[i] - target))
    dt = abs(gt_times[best] - target)
    if dt <= max_dt:
        matches.append((est_t, gt[best][1], est_parts, dt))

if len(matches) < 3:
    raise SystemExit(f"not enough matched poses: {len(matches)}, offset={offset}, max_dt={max_dt}")

with open(gt_out, "w") as fg, open(est_out, "w") as fe:
    for stamp, gt_parts, est_parts, _ in matches:
        # Use the estimated trajectory timestamp for both files so evo sees
        # exact synchronized pairs after nearest-neighbor association.
        fg.write(" ".join([f"{stamp:.9f}"] + gt_parts[1:8]) + "\n")
        fe.write(" ".join([f"{stamp:.9f}"] + est_parts[1:8]) + "\n")

mean_dt = sum(m[3] for m in matches) / len(matches)
print(f"matches={len(matches)}")
print(f"offset={offset:.9f}")
print(f"max_dt={max_dt:.9f}")
print(f"mean_dt={mean_dt:.9f}")
PY

  if [ ! -s "$gt_eval" ] || [ ! -s "$traj_eval" ]; then
    echo "EVO_SKIP failed to create matched TUM files" > "$evo_dir/evo_status.txt"
    return 0
  fi

  export MPLBACKEND=Agg
  echo "EVO_RUNNING" > "$evo_dir/evo_status.txt"
  timeout --preserve-status "$EVO_TIMEOUT" evo_ape tum "$gt_eval" "$traj_eval" -a --align $scale_arg -s -v \
    --save_results "$evo_dir/evo_ape.zip" \
    --save_plot "$evo_dir/evo_ape_plot_xy.pdf" \
    --plot_mode xy --no_warnings > "$evo_dir/evo_ape.txt" 2>&1 || echo "evo_ape failed or timed out" >> "$evo_dir/evo_status.txt"
  timeout --preserve-status "$EVO_TIMEOUT" evo_rpe tum "$gt_eval" "$traj_eval" -a --align $scale_arg -s -v -r trans_part -d 1 -u f \
    --save_results "$evo_dir/evo_rpe.zip" \
    --save_plot "$evo_dir/evo_rpe_plot_xy.pdf" \
    --plot_mode xy --no_warnings > "$evo_dir/evo_rpe.txt" 2>&1 || echo "evo_rpe failed or timed out" >> "$evo_dir/evo_status.txt"
  timeout --preserve-status "$EVO_TIMEOUT" evo_traj tum "$traj_eval" --ref "$gt_eval" --align $scale_arg \
    --save_plot "$evo_dir/evo_traj_plot_xy.pdf" \
    --plot_mode xy --no_warnings > "$evo_dir/evo_traj.txt" 2>&1 || echo "evo_traj failed or timed out" >> "$evo_dir/evo_status.txt"
  echo "EVO_DONE" >> "$evo_dir/evo_status.txt"
}

run_one() {
  local seq="$1" sensor="$2" run_id="$3"
  local seq_tag cache result_dir mono_cfg stereo_cfg cfg exe_name exe tag traj gt start_time end_time elapsed exit_code
  seq_tag="$(printf '%s' "$seq" | tr ' /' '__')"
  cache="$CACHE_ROOT/$seq_tag"
  result_dir="$RESULT_ROOT/idvo_off/$sensor/$seq_tag/run_${run_id}"
  mono_cfg="$result_dir/aquaticvision_mono_idvo_off.yaml"
  stereo_cfg="$result_dir/aquaticvision_stereo_idvo_off.yaml"
  mkdir -p "$result_dir"

  log "[PREPARE] sequence=$seq sensor=$sensor cache=$cache"
  if ! prepare_sequence "$seq" "$cache" "$mono_cfg" "$stereo_cfg" > "$result_dir/prepare_stdout.txt" 2>&1; then
    log "[ERROR] prepare failed sequence=$seq sensor=$sensor"
    write_row "aquaticvision" "$seq" "$sensor" "idvo_off" "$run_id" "PREPARE_FAIL" "0" "0" "0" "0" "0.000" "$result_dir" "" "" "" "" "" "" "prepare_failed"
    return 0
  fi

  case "$sensor" in
    mono) cfg="$mono_cfg" ;;
    stereo) cfg="$stereo_cfg" ;;
    *) write_row "aquaticvision" "$seq" "$sensor" "idvo_off" "$run_id" "UNSUPPORTED" "0" "0" "0" "0" "0.000" "$result_dir" "" "" "" "" "" "" "unsupported_sensor"; return 0 ;;
  esac

  exe_name="$(offline_exe "$sensor")"
  if ! exe="$(find_exe "$exe_name")"; then
    log "[ERROR] missing executable: $exe_name"
    write_row "aquaticvision" "$seq" "$sensor" "idvo_off" "$run_id" "SKIP" "0" "0" "0" "0" "0.000" "$result_dir" "$cfg" "" "$cache/gt.tum" "" "" "" "missing_executable_$exe_name"
    return 0
  fi

  tag="idvo_aquaticvision_${seq_tag}_${sensor}_r${run_id}"
  traj="$result_dir/f_${tag}.txt"
  gt="$cache/gt.tum"

  log "[RUN] sequence=$seq sensor=$sensor run=$run_id"
  start_time="$(date +%s)"
  set +e
  (
    cd "$result_dir"
    AIDVO_OFFLINE_REALTIME=0 "$exe" "$VOCAB" "$cfg" "$cache" "$cache/timestamps.txt" "$tag" > offline_stdout.txt 2>&1
  )
  exit_code=$?
  set -e
  end_time="$(date +%s)"
  elapsed=$((end_time - start_time))

  local input_frames traj_poses completeness status notes
  input_frames="$(count_lines "$cache/timestamps.txt")"
  traj_poses="$(count_lines "$traj")"
  completeness="0.000"
  if [ "$input_frames" -gt 0 ]; then
    completeness="$(python3 - "$traj_poses" "$input_frames" <<'PY'
import sys
print(f"{100.0 * int(sys.argv[1]) / max(1, int(sys.argv[2])):.3f}")
PY
)"
  fi

  run_evo "$gt" "$traj" "$result_dir/evo"
  local ate rpe
  ate="$(extract_metric "$result_dir/evo/evo_ape.txt")"
  rpe="$(extract_metric "$result_dir/evo/evo_rpe.txt")"

  status="PASS"
  notes="ok"
  if [ "$traj_poses" -lt "$MIN_TRAJECTORY_POSES" ]; then
    status="VALIDATION_FAIL"
    notes="too_few_trajectory_poses"
  elif [ "$exit_code" -ne 0 ]; then
    status="PASS_WITH_EXIT_${exit_code}"
    notes="trajectory_saved_but_runner_exit_$exit_code"
  fi

  log "[RESULT] sequence=$seq sensor=$sensor status=$status traj_poses=$traj_poses input_frames=$input_frames completeness=${completeness}% ate_rmse=${ate:-NA} rpe_rmse=${rpe:-NA}"
  write_row "aquaticvision" "$seq" "$sensor" "idvo_off" "$run_id" "$status" "$exit_code" "$elapsed" "$input_frames" "$traj_poses" "$completeness" "$result_dir" "$cfg" "$traj" "$gt" "$result_dir/evo" "$ate" "$rpe" "$notes"
}

build_workspace() {
  if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi
  if [ "$DO_BUILD" = "1" ]; then
    log "[BUILD] catkin build --cmake-args -DORB3_USE_INFOSEL=ON"
    catkin build --cmake-args -DORB3_USE_INFOSEL=ON
  fi
  if [ -f "$WS/devel/setup.bash" ]; then source "$WS/devel/setup.bash"; fi
}

main() {
  summary_header
  log "test_idvo_aquaticvision_20260613 started"
  log "WS=$WS"
  log "AQUATIC_ROOT=$AQUATIC_ROOT"
  log "RUN_TAG=$RUN_TAG"
  log "SEQUENCES=$SEQUENCES"
  log "SENSORS=$SENSORS"
  log "RUNS_PER_CASE=$RUNS_PER_CASE"
  log "MAX_FRAMES=$MAX_FRAMES"
  log "FPS=$FPS"
  log "TOPK=$TOPK"
  log "RESULT_ROOT=$RESULT_ROOT"

  if [ ! -f "$PREPARE_TOOL" ]; then
    log "[ERROR] missing prepare tool: $PREPARE_TOOL"
    exit 2
  fi
  if [ ! -d "$AQUATIC_ROOT" ]; then
    log "[ERROR] missing AquaticVision root: $AQUATIC_ROOT"
    log "Set AQUATIC_ROOT=/path/to/AquaticVision"
    exit 3
  fi

  build_workspace

  log "[DATASET] available sequences:"
  python3 "$PREPARE_TOOL" --root "$AQUATIC_ROOT" --list | tee -a "$LOG_FILE"

  local seq sensor run_id
  for seq in $SEQUENCES; do
    for sensor in $SENSORS; do
      for run_id in $(seq 1 "$RUNS_PER_CASE"); do
        run_one "$seq" "$sensor" "$run_id"
      done
    done
  done

  log "============================================================"
  log "test_idvo_aquaticvision_20260613 finished"
  log "Summary: $SUMMARY"
  log "Result root: $RESULT_ROOT"
  log "Evo plot files:"
  find "$RESULT_ROOT" -type f \( -name "evo_ape_plot_xy.pdf" -o -name "evo_rpe_plot_xy.pdf" -o -name "evo_traj_plot_xy.pdf" \) | sort | tee "$RESULT_ROOT/evo_plots.txt" | tee -a "$LOG_FILE"
}

main "$@"
