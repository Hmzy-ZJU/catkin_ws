#!/usr/bin/env bash
set -euo pipefail

# Offline direct-reader AIDVO test.
# It converts rosbag sequences into EuRoC-like folders, then runs ORB-SLAM3
# direct-reader examples without ROS playback/callback timing.

WS="${WS:-$HOME/catkin_ws}"
RUN_TAG="${RUN_TAG:-orb3_offline_main_$(date +%Y%m%d_%H%M%S)}"
AIDVO_MODES="${AIDVO_MODES:-off fixed rule}"
AIDVO_MODES="$(printf '%s' "$AIDVO_MODES" | tr ',' ' ')"
RUNS_PER_CASE="${RUNS_PER_CASE:-1}"
RUN_ALL_BAGS="${RUN_ALL_BAGS:-1}"
RUN_PROFILE="${RUN_PROFILE:-full}"
DO_BUILD="${DO_BUILD:-0}"
BAG_START="${BAG_START:-0}"
BAG_DURATION="${BAG_DURATION:-0}"
MAX_FRAMES="${MAX_FRAMES:-0}"
ENABLE_ADAPTIVE_LOGGING="${ENABLE_ADAPTIVE_LOGGING:-1}"
MIN_TRAJECTORY_COMPLETENESS="${MIN_TRAJECTORY_COMPLETENESS:-70.0}"

VOCAB="${VOCAB:-$WS/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin}"
RESULT_ROOT="$WS/results/aidvo_offline_main_${RUN_TAG}"
SUMMARY="$RESULT_ROOT/offline_summary.csv"
LOG_FILE="$RESULT_ROOT/offline_main.log"
EXTRACT_ROOT="${EXTRACT_ROOT:-$WS/offline_euroc_cache}"

mkdir -p "$RESULT_ROOT"

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
  printf 'dataset,sensor,aidvo_mode,bag,sequence,run_id,status,exit_code,elapsed_sec,extract_dir,result_dir,settings_file,trajectory_file,adaptive_csv,trajectory_poses,input_frames,trajectory_completeness_percent,notes\n' > "$SUMMARY"
}

dataset_root() {
  case "$1" in
    euroc) printf '%s\n' "$WS/dataset_EuRoc" ;;
    tank) printf '%s\n' "$WS/dataset_tank" ;;
    harbor) printf '%s\n' "$WS/dataset_harbor" ;;
    *) return 1 ;;
  esac
}

base_config() {
  local dataset="$1" sensor="$2"
  case "$dataset/$sensor" in
    euroc/mono) printf '%s\n' "$WS/src/orb_slam3_ros/config/Monocular/EuRoc/EuRoc_on_11.yaml" ;;
    euroc/stereo) printf '%s\n' "$WS/src/orb_slam3_ros/config/Stereo/EuRoC.yaml" ;;
    euroc/mono-inertial) printf '%s\n' "$WS/src/orb_slam3_ros/config/Monocular-Inertial/EuRoC.yaml" ;;
    euroc/stereo-inertial) printf '%s\n' "$WS/src/orb_slam3_ros/config/Stereo-Inertial/EuRoC.yaml" ;;
    tank/mono) printf '%s\n' "$WS/src/orb_slam3_ros/config/Monocular/Tank/tank_on_11.yaml" ;;
    tank/stereo) printf '%s\n' "$WS/src/orb_slam3_ros/config/Stereo/Tank/Tank_stereo_on_11.yaml" ;;
    tank/mono-inertial) printf '%s\n' "$WS/src/orb_slam3_ros/config/Monocular-Inertial/Tank/tank_on_11.yaml" ;;
    tank/stereo-inertial) printf '%s\n' "$WS/src/orb_slam3_ros/config/Stereo-Inertial/Tank_stereo_inertial_on_11.yaml" ;;
    harbor/mono) printf '%s\n' "$WS/src/orb_slam3_ros/config/Monocular/Aquacular_harbor/all/Aqualoc_harbor_on_11.yaml" ;;
    harbor/mono-inertial) printf '%s\n' "$WS/src/orb_slam3_ros/config/Monocular-Inertial/Aqualoc_harbor.yaml" ;;
    *) return 1 ;;
  esac
}

offline_exe() {
  case "$1" in
    mono) printf '%s\n' aidvo_offline_mono_euroc ;;
    stereo) printf '%s\n' aidvo_offline_stereo_euroc ;;
    mono-inertial) printf '%s\n' aidvo_offline_mono_inertial_euroc ;;
    stereo-inertial) printf '%s\n' aidvo_offline_stereo_inertial_euroc ;;
    *) return 1 ;;
  esac
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

bag_list() {
  local dataset="$1" root
  root="$(dataset_root "$dataset")"
  if [ "$RUN_ALL_BAGS" = "1" ]; then
    find "$root/data" -maxdepth 1 -type f -name '*.bag' | sort
  else
    case "$dataset" in
      euroc) printf '%s\n' "${EUROC_BAG:-$root/data/MH_01.bag}" ;;
      tank) printf '%s\n' "${TANK_BAG:-$root/data/HalfTank_Easy.bag}" ;;
      harbor) printf '%s\n' "${HARBOR_BAG:-$root/data/harbor_sequence_1.bag}" ;;
    esac
  fi
}

case_supported() {
  local dataset="$1" sensor="$2"
  if [ "$dataset" = "harbor" ] && { [ "$sensor" = "stereo" ] || [ "$sensor" = "stereo-inertial" ]; }; then
    return 1
  fi
  return 0
}

matrix_cases() {
  if [ "$RUN_PROFILE" = "meaningful" ]; then
    cat <<CASES
euroc mono
euroc stereo
euroc mono-inertial
euroc stereo-inertial
harbor mono
CASES
  elif [ "$RUN_PROFILE" = "full" ]; then
    cat <<CASES
euroc mono
euroc stereo
euroc mono-inertial
euroc stereo-inertial
tank mono
tank stereo
tank mono-inertial
tank stereo-inertial
harbor mono
harbor mono-inertial
harbor stereo
harbor stereo-inertial
CASES
  else
    log "[ERROR] RUN_PROFILE must be full or meaningful"
    exit 1
  fi
}

generate_config() {
  local base="$1" out="$2" mode="$3" adaptive_csv="$4"
  local enable=1 policy=RuleBased
  case "$mode" in
    off) enable=0; policy=Fixed ;;
    fixed) enable=1; policy=Fixed ;;
    rule) enable=1; policy=RuleBased ;;
    *) return 1 ;;
  esac
  awk '
    !/^[[:space:]]*(InfoSelector\.TopK|EnableAdaptiveIDVO|AdaptivePolicyType|MinKappaTop|MaxKappaTop|MinTau0|MaxTau0|TrackingTimeBudget|SmoothFactor|EnableAdaptiveLogging|AdaptiveLogPath|Adaptive\.DisableBeforeImuReady)[[:space:]]*:/
  ' "$base" > "$out"
  cat >> "$out" <<EOF

# Generated by test_aidvo_orb3_offline_main_20260608.sh
EnableAdaptiveIDVO: ${enable}
AdaptivePolicyType: "${policy}"
MinKappaTop: ${MIN_KAPPA_TOP:-120}
MaxKappaTop: ${MAX_KAPPA_TOP:-520}
MinTau0: ${MIN_TAU0:-0.1}
MaxTau0: ${MAX_TAU0:-5.0}
TrackingTimeBudget: ${TRACKING_TIME_BUDGET:-30.0}
SmoothFactor: ${SMOOTH_FACTOR:-0.8}
Adaptive.DisableBeforeImuReady: ${ADAPTIVE_DISABLE_BEFORE_IMU_READY:-1}
EnableAdaptiveLogging: ${ENABLE_ADAPTIVE_LOGGING}
AdaptiveLogPath: "${adaptive_csv}"
EOF
}

extract_bag() {
  local dataset="$1" bag="$2" seq="$3" out="$4"
  if [ -f "$out/timestamps.txt" ] && [ -d "$out/mav0/cam0/data" ]; then
    log "[EXTRACT] cache hit dataset=$dataset seq=$seq out=$out"
    return 0
  fi
  log "[EXTRACT] dataset=$dataset bag=$bag out=$out"
  mkdir -p "$out"
  python3 "$WS/tools/extract_rosbag_to_euroc.py" --dataset "$dataset" --bag "$bag" --out "$out" --start "$BAG_START" --duration "$BAG_DURATION" --max-frames "$MAX_FRAMES"
}

count_lines() {
  local file="$1"
  [ -f "$file" ] || { printf '0\n'; return; }
  awk 'NF && $1 !~ /^#/ { c++ } END { print c+0 }' "$file"
}

run_one() {
  local dataset="$1" sensor="$2" mode="$3" bag="$4" run_id="$5"
  local seq extract_dir dataset_results result_dir cfg_base cfg exe_name exe adaptive_csv tag traj_file status exit_code elapsed input_frames traj_poses completeness notes

  seq="$(basename "$bag" .bag)"
  extract_dir="$EXTRACT_ROOT/$dataset/$seq"
  dataset_results="$(dataset_root "$dataset")/results/aidvo_offline_${RUN_TAG}"
  result_dir="$dataset_results/$mode/$sensor/$seq/run_${run_id}"
  mkdir -p "$result_dir"

  if ! case_supported "$dataset" "$sensor"; then
    write_row "$dataset" "$sensor" "$mode" "$bag" "$seq" "$run_id" "UNSUPPORTED" "0" "0" "$extract_dir" "$result_dir" "" "" "" "0" "0" "0.000" "unsupported_dataset_sensor"
    return 0
  fi

  cfg_base="$(base_config "$dataset" "$sensor")" || {
    write_row "$dataset" "$sensor" "$mode" "$bag" "$seq" "$run_id" "UNSUPPORTED" "0" "0" "$extract_dir" "$result_dir" "" "" "" "0" "0" "0.000" "missing_base_config_mapping"
    return 0
  }
  exe_name="$(offline_exe "$sensor")"
  if ! exe="$(find_exe "$exe_name")"; then
    write_row "$dataset" "$sensor" "$mode" "$bag" "$seq" "$run_id" "SKIP" "0" "0" "$extract_dir" "$result_dir" "" "" "" "0" "0" "0.000" "missing_executable_$exe_name"
    return 0
  fi

  extract_bag "$dataset" "$bag" "$seq" "$extract_dir"

  if [ "$sensor" = "stereo" ] || [ "$sensor" = "stereo-inertial" ]; then
    if [ ! -d "$extract_dir/mav0/cam1/data" ]; then
      write_row "$dataset" "$sensor" "$mode" "$bag" "$seq" "$run_id" "SKIP" "0" "0" "$extract_dir" "$result_dir" "" "" "" "0" "0" "0.000" "missing_right_camera_extraction"
      return 0
    fi
  fi

  adaptive_csv="$result_dir/adaptive_frames.csv"
  cfg="$result_dir/settings_${mode}.yaml"
  generate_config "$cfg_base" "$cfg" "$mode" "$adaptive_csv"

  tag="aidvo_offline_${dataset}_${sensor}_${mode}_${seq}_r${run_id}"
  log "[RUN] dataset=$dataset sensor=$sensor mode=$mode seq=$seq run=$run_id"

  local start_time end_time
  start_time="$(date +%s)"
  set +e
  (
    cd "$result_dir"
    AIDVO_OFFLINE_REALTIME="${AIDVO_OFFLINE_REALTIME:-0}" "$exe" "$VOCAB" "$cfg" "$extract_dir" "$extract_dir/timestamps.txt" "$tag" > offline_stdout.txt 2>&1
  )
  exit_code=$?
  set -e
  end_time="$(date +%s)"
  elapsed=$((end_time - start_time))

  traj_file="$result_dir/f_${tag}.txt"
  input_frames="$(count_lines "$extract_dir/timestamps.txt")"
  traj_poses="$(count_lines "$traj_file")"
  completeness="0.000"
  if [ "$input_frames" -gt 0 ]; then
    completeness="$(python3 - "$traj_poses" "$input_frames" <<'PY'
import sys
print(f"{100.0 * int(sys.argv[1]) / max(1, int(sys.argv[2])):.3f}")
PY
)"
  fi

  status="PASS"
  notes="ok"
  if [ "$traj_poses" -lt "${MIN_TRAJECTORY_POSES:-20}" ]; then
    status="VALIDATION_FAIL"
    notes="too_few_trajectory_poses"
  elif ! python3 - "$completeness" "$MIN_TRAJECTORY_COMPLETENESS" <<'PY'
import sys
sys.exit(0 if float(sys.argv[1]) >= float(sys.argv[2]) else 1)
PY
  then
    status="VALIDATION_FAIL"
    notes="low_trajectory_completeness_${completeness}_lt_${MIN_TRAJECTORY_COMPLETENESS}"
  elif [ "$exit_code" -ne 0 ]; then
    status="PASS_WITH_EXIT_${exit_code}"
    notes="trajectory_saved_but_runner_exit_$exit_code"
  fi

  write_row "$dataset" "$sensor" "$mode" "$bag" "$seq" "$run_id" "$status" "$exit_code" "$elapsed" "$extract_dir" "$result_dir" "$cfg" "$traj_file" "$adaptive_csv" "$traj_poses" "$input_frames" "$completeness" "$notes"
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
  : > "$LOG_FILE"
  summary_header
  log "test_aidvo_orb3_offline_main_20260608 started"
  log "WS=$WS"
  log "RUN_TAG=$RUN_TAG"
  log "RUN_PROFILE=$RUN_PROFILE"
  log "RUN_ALL_BAGS=$RUN_ALL_BAGS"
  log "RUNS_PER_CASE=$RUNS_PER_CASE"
  log "BAG_START=$BAG_START"
  log "BAG_DURATION=$BAG_DURATION"
  log "MAX_FRAMES=$MAX_FRAMES"
  log "MIN_TRAJECTORY_COMPLETENESS=$MIN_TRAJECTORY_COMPLETENESS"
  log "RESULT_ROOT=$RESULT_ROOT"
  build_workspace

  while read -r dataset sensor; do
    [ -z "$dataset" ] && continue
    if [ -n "${ONLY_DATASET:-}" ] && [ "$dataset" != "$ONLY_DATASET" ]; then continue; fi
    if [ -n "${ONLY_SENSOR:-}" ] && [ "$sensor" != "$ONLY_SENSOR" ]; then continue; fi
    while read -r bag; do
      [ -z "$bag" ] && continue
      for mode in $AIDVO_MODES; do
        for run_id in $(seq 1 "$RUNS_PER_CASE"); do
          run_one "$dataset" "$sensor" "$mode" "$bag" "$run_id"
        done
      done
    done < <(bag_list "$dataset")
  done < <(matrix_cases)

  log "Offline summary: $SUMMARY"
  if command -v column >/dev/null 2>&1; then
    column -s, -t "$SUMMARY" || true
  fi
}

main "$@"
