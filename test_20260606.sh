#!/usr/bin/env bash
set -o pipefail

# UW-AIDVO full regression runner, 2026-06-06.
# Ubuntu usage:
#   cd ~/catkin_ws
#   bash test_20260606.sh
#
# Optional examples:
#   DO_BUILD=0 bash test_20260606.sh
#   BUILD_TOOL=catkin_build bash test_20260606.sh
#   BUILD_TOOL=catkin_make bash test_20260606.sh
#   BAG_DURATION=60 bash test_20260606.sh
#   AIDVO_MODES="off fixed rule" bash test_20260606.sh
#   EUROC_BAG=/path/to/MH_01.bag bash test_20260606.sh

if [ -z "$WS" ]; then WS="$HOME/catkin_ws"; fi
if [ -z "$DO_BUILD" ]; then DO_BUILD=1; fi
if [ -z "$BUILD_TOOL" ]; then BUILD_TOOL=auto; fi
if [ -z "$AIDVO_MODES" ]; then AIDVO_MODES="off fixed rule"; fi
AIDVO_MODES="$(printf '%s' "$AIDVO_MODES" | tr ',' ' ')"
if [ -z "$BAG_DURATION" ]; then BAG_DURATION=0; fi
if [ -z "$BAG_RATE" ]; then BAG_RATE=1.0; fi
if [ -z "$RUN_TIMEOUT" ]; then RUN_TIMEOUT=3600; fi
if [ -z "$STARTUP_WAIT" ]; then STARTUP_WAIT=8; fi
if [ -z "$ENABLE_ADAPTIVE_LOGGING" ]; then ENABLE_ADAPTIVE_LOGGING=1; fi
if [ -z "$TEST_ROOT" ]; then TEST_ROOT="$WS/test_20260606"; fi

MASTER_LOG="$TEST_ROOT/test_20260606.log"
SUMMARY_CSV="$TEST_ROOT/test_20260606_summary.csv"

mkdir -p "$TEST_ROOT"
: > "$MASTER_LOG"
echo "dataset,sensor,mode,bag,status,exit_code,elapsed_sec,result_dir,adaptive_csv,roslaunch_log,validation" > "$SUMMARY_CSV"
exec > >(tee -a "$MASTER_LOG") 2>&1

ts() { date "+%Y-%m-%d %H:%M:%S"; }
log() { echo "[$(ts)] $*"; }

csv_escape() {
  printf '"%s"' "$1"
}

summary_row() {
  local dataset="$1"
  local sensor="$2"
  local mode="$3"
  local bag="$4"
  local status="$5"
  local exit_code="$6"
  local elapsed="$7"
  local result_dir="$8"
  local adaptive_csv="$9"
  shift 9
  local launch_log="$1"
  local validation="$2"
  {
    csv_escape "$dataset"; printf ','
    csv_escape "$sensor"; printf ','
    csv_escape "$mode"; printf ','
    csv_escape "$bag"; printf ','
    csv_escape "$status"; printf ','
    csv_escape "$exit_code"; printf ','
    csv_escape "$elapsed"; printf ','
    csv_escape "$result_dir"; printf ','
    csv_escape "$adaptive_csv"; printf ','
    csv_escape "$launch_log"; printf ','
    csv_escape "$validation"; printf '\n'
  } >> "$SUMMARY_CSV"
}

dataset_root() {
  case "$1" in
    euroc) if [ -n "$EUROC_ROOT" ]; then echo "$EUROC_ROOT"; else echo "$WS/dataset_EuRoc"; fi ;;
    tank) if [ -n "$TANK_ROOT" ]; then echo "$TANK_ROOT"; else echo "$WS/dataset_tank"; fi ;;
    harbor) if [ -n "$HARBOR_ROOT" ]; then echo "$HARBOR_ROOT"; else echo "$WS/dataset_harbor"; fi ;;
    archaeo) if [ -n "$ARCHAEO_ROOT" ]; then echo "$ARCHAEO_ROOT"; else echo "$WS/dataset_archaeo"; fi ;;
    cave) if [ -n "$CAVE_ROOT" ]; then echo "$CAVE_ROOT"; else echo "$WS/dataset_cave"; fi ;;
  esac
}

bag_override() {
  case "$1" in
    euroc) echo "$EUROC_BAG" ;;
    tank) echo "$TANK_BAG" ;;
    harbor) echo "$HARBOR_BAG" ;;
    archaeo) echo "$ARCHAEO_BAG" ;;
    cave) echo "$CAVE_BAG" ;;
  esac
}

first_bag() {
  find "$1/data" -maxdepth 1 -type f -name "*.bag" | sort | head -n 1
}

expected_switch() {
  if [ "$1" = "off" ]; then echo "NO"; else echo "YES"; fi
}

expected_policy() {
  if [ "$1" = "rule" ]; then echo "RuleBased"; else echo "Fixed"; fi
}

validate_csv() {
  python3 - "$1" "$2" "$3" <<'PY'
import csv
import math
import sys

path, mode, sensor = sys.argv[1:4]
rows = list(csv.DictReader(open(path, newline="")))
required = [
    "candidate_point_number", "selected_point_number", "selection_ratio",
    "logdet_H", "condition_number_H", "image_contrast", "blur_score",
    "tracked_map_points", "kappa_top", "alpha", "tau0", "theta_drop",
    "keyframe_insertion_flag", "tracking_lost_flag",
]
if not rows:
    print("FAIL no_rows")
    sys.exit(2)
missing = [c for c in required if c not in rows[0]]
if missing:
    print("FAIL missing_columns=" + "|".join(missing))
    sys.exit(2)

def vals(name):
    out = []
    for row in rows:
        try:
            out.append(float(row[name]))
        except Exception:
            out.append(float("nan"))
    return out

cand = vals("candidate_point_number")
sel = vals("selected_point_number")
ratio = vals("selection_ratio")
kappa = vals("kappa_top")
alpha = vals("alpha")
tau0 = vals("tau0")
theta = vals("theta_drop")
contrast = vals("image_contrast")
blur = vals("blur_score")
cond = vals("condition_number_H")
tracked = vals("tracked_map_points")
inserted = vals("keyframe_insertion_flag")

errors = []
idps = 0
removed = 0
for c, s, r in zip(cand, sel, ratio):
    if c > 0:
        idps += 1
        if not (0 <= s <= c):
            errors.append("selected_gt_candidate")
        if abs(r - s / c) > 1e-3:
            errors.append("bad_selection_ratio")
        if s < c:
            removed += 1

if not all(60 <= v <= 180 for v in kappa if math.isfinite(v)):
    errors.append("kappa_out_of_bounds")
if not all(0.1 <= v <= 5.0 for v in tau0 if math.isfinite(v)):
    errors.append("tau0_out_of_bounds")
if not all(0 <= v <= 1 for v in alpha if math.isfinite(v)):
    errors.append("alpha_out_of_bounds")
if not all(v >= 0 for v in theta if math.isfinite(v)):
    errors.append("theta_negative")
if not any(v > 0 for v in contrast) or not any(v > 0 for v in blur):
    errors.append("image_quality_not_collected")
if not any(v > 0 for v in cond):
    errors.append("fim_not_collected")
if not any(v > 0 for v in tracked):
    errors.append("tracked_points_not_collected")
if sensor not in ("mono-inertial", "stereo-inertial") and idps == 0:
    errors.append("no_visual_idps_frames")
if mode in ("off", "fixed"):
    if len(set(kappa)) > 1 or len(set(tau0)) > 1 or len(set(alpha)) > 1 or len(set(theta)) > 1:
        errors.append("fixed_params_changed")

rule_changed = len(set(kappa)) > 1 or len(set(tau0)) > 1 or len(set(alpha)) > 1 or len(set(theta)) > 1
warn = ""
if mode == "rule" and not rule_changed:
    warn = " WARN rule_params_not_changed"

if errors:
    print("FAIL " + "|".join(sorted(set(errors))))
    sys.exit(2)
print(
    f"PASS rows={len(rows)} idps_frames={idps} removed_frames={removed} "
    f"kappa_unique={len(set(kappa))} tau0_unique={len(set(tau0))} "
    f"keyframes_inserted={sum(1 for v in inserted if v == 1)}{warn}"
)
PY
}

validate_run() {
  mode="$1"
  sensor="$2"
  launch_log="$3"
  adaptive_csv="$4"
  errors=""
  notes=""

  if [ ! -f "$launch_log" ]; then
    errors="$errors missing_roslaunch_log"
  else
    grep -q "Adaptive IDVO params loaded" "$launch_log" || errors="$errors config_not_loaded"
    grep -q "EnableAdaptiveIDVO: $(expected_switch "$mode")" "$launch_log" || errors="$errors wrong_adaptive_switch"
    grep -q "AdaptivePolicyType: $(expected_policy "$mode")" "$launch_log" || errors="$errors wrong_policy"
    if grep -q "\[InfoSel\]" "$launch_log"; then notes="$notes infosel_log_seen"; else notes="$notes infosel_log_not_seen"; fi
  fi

  if [ ! -s "$adaptive_csv" ]; then
    errors="$errors missing_or_empty_adaptive_csv"
  else
    csv_check="$(validate_csv "$adaptive_csv" "$mode" "$sensor")"
    csv_rc=$?
    notes="$notes $csv_check"
    if [ "$csv_rc" != "0" ]; then errors="$errors csv_validation_failed"; fi
  fi

  if [ -z "$errors" ]; then
    echo "PASS$notes"
  else
    echo "FAIL errors=$errors notes=$notes"
  fi
}

run_case() {
  local dataset="$1"
  local sensor="$2"
  local script_rel="$3"
  local mode="$4"
  local root bag script bag_name result_dir adaptive_csv launch_log
  local start end elapsed exit_code status validation

  root="$(dataset_root "$dataset")"
  bag="$(bag_override "$dataset")"
  if [ -z "$bag" ]; then bag="$(first_bag "$root")"; fi
  script="$WS/$script_rel"

  if [ ! -f "$script" ]; then
    log "[SKIP] missing script: $script"
    summary_row "$dataset" "$sensor" "$mode" "$bag" "SKIP" "0" "0" "" "" "" "missing_script"
    return 0
  fi
  if [ ! -f "$bag" ]; then
    log "[SKIP] missing bag for $dataset: $bag"
    summary_row "$dataset" "$sensor" "$mode" "$bag" "SKIP" "0" "0" "" "" "" "missing_bag"
    return 0
  fi

  bag_name="$(basename "$bag" .bag)"
  result_dir="$root/aidvo_results/$sensor/$dataset/$mode/$bag_name"
  adaptive_csv="$result_dir/adaptive_frames.csv"
  launch_log="$result_dir/roslaunch.log"

  log "============================================================"
  log "[RUN] dataset=$dataset sensor=$sensor mode=$mode"
  log "[RUN] bag=$bag"
  log "[RUN] script=$script"
  log "[RUN] result_dir=$result_dir"
  log "============================================================"

  start="$(date +%s)"
  timeout --preserve-status "$RUN_TIMEOUT" env WS="$WS" AIDVO_MODE="$mode" BAG_FILE="$bag" BAG_DURATION="$BAG_DURATION" BAG_RATE="$BAG_RATE" STARTUP_WAIT="$STARTUP_WAIT" ENABLE_ADAPTIVE_LOGGING="$ENABLE_ADAPTIVE_LOGGING" RUN_ALL_BAGS=0 bash "$script" < /dev/null
  exit_code=$?
  end="$(date +%s)"
  elapsed="$((end - start))"

  if [ "$exit_code" = "124" ]; then
    status="TIMEOUT"
    validation="timeout"
  elif [ "$exit_code" != "0" ]; then
    status="RUN_FAIL"
    validation="run_exit_$exit_code"
  else
    validation="$(validate_run "$mode" "$sensor" "$launch_log" "$adaptive_csv")"
    if echo "$validation" | grep -q "^PASS"; then status="PASS"; else status="VALIDATION_FAIL"; fi
  fi

  log "[RESULT] dataset=$dataset sensor=$sensor mode=$mode status=$status elapsed=$elapsed"
  log "[CHECK] $validation"
  summary_row "$dataset" "$sensor" "$mode" "$bag" "$status" "$exit_code" "$elapsed" "$result_dir" "$adaptive_csv" "$launch_log" "$validation"
}

preflight() {
  log "test_20260606 started"
  log "WS=$WS"
  log "TEST_ROOT=$TEST_ROOT"
  log "AIDVO_MODES=$AIDVO_MODES"
  log "BAG_DURATION=$BAG_DURATION"
  log "RUN_TIMEOUT=$RUN_TIMEOUT"
  log "BUILD_TOOL=$BUILD_TOOL"

  cd "$WS" || return 1
  command -v python3 >/dev/null || return 1
  command -v timeout >/dev/null || return 1

  if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi

  if [ "$DO_BUILD" = "1" ]; then
    build_tool="$BUILD_TOOL"
    if [ "$build_tool" = "auto" ]; then
      if [ -d "$WS/.catkin_tools" ] || [ -f "$WS/build/.catkin_tools.yaml" ]; then
        build_tool="catkin_build"
      elif command -v catkin >/dev/null; then
        build_tool="catkin_build"
      else
        build_tool="catkin_make"
      fi
    fi

    if [ "$build_tool" = "catkin_build" ]; then
      command -v catkin >/dev/null || { log "[ERROR] catkin command not found. Install catkin_tools or run DO_BUILD=0 if already built."; return 1; }
      log "[BUILD] catkin build --cmake-args -DORB3_USE_INFOSEL=ON"
      catkin build --cmake-args -DORB3_USE_INFOSEL=ON || return 1
    elif [ "$build_tool" = "catkin_make" ]; then
      command -v catkin_make >/dev/null || return 1
      log "[BUILD] catkin_make -DORB3_USE_INFOSEL=ON"
      catkin_make -DORB3_USE_INFOSEL=ON || return 1
    else
      log "[ERROR] BUILD_TOOL must be auto, catkin_build, or catkin_make"
      return 1
    fi
  else
    log "[BUILD] skipped"
  fi

  if [ ! -f "$WS/devel/setup.bash" ]; then log "[ERROR] missing $WS/devel/setup.bash"; return 1; fi
  source "$WS/devel/setup.bash"
  command -v roslaunch >/dev/null || return 1
  command -v rosbag >/dev/null || return 1
  command -v rosservice >/dev/null || return 1
  command -v rosnode >/dev/null || return 1

  for node in "$WS/devel/lib/orb_slam3_ros/ros_mono" "$WS/devel/lib/orb_slam3_ros/ros_stereo" "$WS/devel/lib/orb_slam3_ros/ros_mono_inertial" "$WS/devel/lib/orb_slam3_ros/ros_stereo_inertial"; do
    if [ ! -x "$node" ]; then log "[ERROR] missing executable: $node"; return 1; fi
  done
  log "[PREFLIGHT] OK"
}

print_plan() {
  log "Dataset plan:"
  for dataset in euroc tank harbor archaeo cave; do
    root="$(dataset_root "$dataset")"
    bag="$(bag_override "$dataset")"
    if [ -z "$bag" ] && [ -d "$root/data" ]; then bag="$(first_bag "$root")"; fi
    if [ -z "$bag" ]; then bag="missing"; fi
    log "  $dataset root=$root selected_bag=$bag"
  done
  log "Supported matrix: EuRoC and Tank run all four sensor modes; Harbor runs mono and mono-inertial; Archaeo/Cave run mono."
}

main() {
  preflight || exit 1
  print_plan

  while IFS='|' read -r dataset sensor script_rel; do
    [ -z "$dataset" ] && continue
    for mode in $AIDVO_MODES; do
      if [ "$mode" != "off" ] && [ "$mode" != "fixed" ] && [ "$mode" != "rule" ]; then
        log "[WARN] invalid mode skipped: $mode"
        continue
      fi
      run_case "$dataset" "$sensor" "$script_rel" "$mode"
    done
  done <<'CASES'
euroc|mono|Parameters_test/V-slam/mono/aidvo_euroc.sh
euroc|stereo|Parameters_test/V-slam/stereo/aidvo_euroc.sh
euroc|mono-inertial|Parameters_test/VI-slam/mono-inertial/aidvo_euroc.sh
euroc|stereo-inertial|Parameters_test/VI-slam/stereo-inertial/aidvo_euroc.sh
tank|mono|Parameters_test/V-slam/mono/aidvo_tank.sh
tank|stereo|Parameters_test/V-slam/stereo/aidvo_tank.sh
tank|mono-inertial|Parameters_test/VI-slam/mono-inertial/aidvo_tank.sh
tank|stereo-inertial|Parameters_test/VI-slam/stereo-inertial/aidvo_tank.sh
harbor|mono|Parameters_test/V-slam/mono/aidvo_harbor.sh
harbor|mono-inertial|Parameters_test/VI-slam/mono-inertial/aidvo_harbor.sh
archaeo|mono|Parameters_test/V-slam/mono/aidvo_archaeo.sh
cave|mono|Parameters_test/V-slam/mono/aidvo_cave.sh
CASES

  while IFS='|' read -r dataset sensor reason; do
    [ -z "$dataset" ] && continue
    for mode in $AIDVO_MODES; do
      summary_row "$dataset" "$sensor" "$mode" "" "UNSUPPORTED" "0" "0" "" "" "" "$reason"
    done
  done <<'UNSUPPORTED'
harbor|stereo|no existing stereo launch/config/script for Harbor in this project
harbor|stereo-inertial|no existing stereo-inertial launch/config/script for Harbor in this project
archaeo|stereo|no existing stereo launch/config/script for Archaeo in this project
archaeo|mono-inertial|no existing mono-inertial launch/config/script for Archaeo in this project
archaeo|stereo-inertial|no existing stereo-inertial launch/config/script for Archaeo in this project
cave|stereo|no existing stereo launch/config/script for Cave in this project
cave|mono-inertial|no existing mono-inertial launch/config/script for Cave in this project
cave|stereo-inertial|no existing stereo-inertial launch/config/script for Cave in this project
UNSUPPORTED

  log "============================================================"
  log "test_20260606 finished"
  log "Master log: $MASTER_LOG"
  log "Summary CSV: $SUMMARY_CSV"
  python3 - "$SUMMARY_CSV" <<'PY'
import csv
import sys
from collections import Counter
rows = list(csv.DictReader(open(sys.argv[1], newline='')))
counter = Counter(r["status"] for r in rows)
print("Status counts:")
for key in sorted(counter):
    print(f"  {key}: {counter[key]}")
PY
  log "============================================================"
}

main "$@"
