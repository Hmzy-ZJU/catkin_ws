#!/usr/bin/env bash
set -u

# Focused ROS online test for Tank mono/stereo AIDVO evaluation.
# It runs four Tank sequences with mono and stereo sensors, keeps evo enabled,
# and collects a combined summary for quick inspection.

WS="${WS:-$HOME/catkin_ws}"
FULL_SCRIPT="$WS/test_aidvo_full_20260607.sh"

RUN_TAG="${RUN_TAG:-tank_4seq_mono_stereo_evo_$(date +%Y%m%d_%H%M%S)}"
AIDVO_MODES="${AIDVO_MODES:-off,fixed,rule}"
RUNS_PER_CASE="${RUNS_PER_CASE:-1}"
BAG_START="${BAG_START:-0}"
BAG_DURATION="${BAG_DURATION:-0}"
DO_BUILD="${DO_BUILD:-0}"

RUN_EVO="${RUN_EVO:-1}"
RUN_TIMEOUT="${RUN_TIMEOUT:-7200}"
SAVE_TRAJ_TIMEOUT="${SAVE_TRAJ_TIMEOUT:-180}"
EVO_TIMEOUT="${EVO_TIMEOUT:-600}"
ENABLE_ADAPTIVE_LOGGING="${ENABLE_ADAPTIVE_LOGGING:-1}"

SEQUENCES="${SEQUENCES:-HalfTank_Easy Structure_Easy Structure_Medium Structure_Hard}"
SENSORS="${SENSORS:-mono stereo}"

RESULT_ROOT="$WS/dataset_tank/results/aidvo_tank_4seq_mono_stereo_${RUN_TAG}"
MASTER_LOG="$RESULT_ROOT/test_tank_4seq_mono_stereo.log"
COMBINED_SUMMARY="$RESULT_ROOT/summary_tank_4seq_mono_stereo.csv"
EVO_DIR_LIST="$RESULT_ROOT/evo_dirs.txt"
EVO_PLOT_LIST="$RESULT_ROOT/evo_plots.txt"

mkdir -p "$RESULT_ROOT"
: > "$MASTER_LOG"
: > "$EVO_DIR_LIST"
: > "$EVO_PLOT_LIST"

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$MASTER_LOG"
}

append_summary() {
  local summary_file="$1"
  if [ ! -f "$summary_file" ]; then
    log "[WARN] missing child summary: $summary_file"
    return 0
  fi

  if [ ! -s "$COMBINED_SUMMARY" ]; then
    head -n 1 "$summary_file" > "$COMBINED_SUMMARY"
  fi
  tail -n +2 "$summary_file" >> "$COMBINED_SUMMARY"
}

log "test_tank_4seq_mono_stereo_evo_20260610 started"
log "WS=$WS"
log "RUN_TAG=$RUN_TAG"
log "AIDVO_MODES=$AIDVO_MODES"
log "RUNS_PER_CASE=$RUNS_PER_CASE"
log "BAG_START=$BAG_START"
log "BAG_DURATION=$BAG_DURATION"
log "RUN_EVO=$RUN_EVO"
log "RUN_TIMEOUT=$RUN_TIMEOUT"
log "SAVE_TRAJ_TIMEOUT=$SAVE_TRAJ_TIMEOUT"
log "EVO_TIMEOUT=$EVO_TIMEOUT"
log "SEQUENCES=$SEQUENCES"
log "SENSORS=$SENSORS"
log "RESULT_ROOT=$RESULT_ROOT"

if [ ! -f "$FULL_SCRIPT" ]; then
  log "[ERROR] missing main test script: $FULL_SCRIPT"
  exit 2
fi

if [ "$RUN_EVO" != "1" ]; then
  log "[WARN] RUN_EVO is not 1. Evo files and plots may not be generated."
fi

if ! command -v evo_ape >/dev/null 2>&1; then
  log "[WARN] evo_ape not found in PATH. The main script may skip or fail evo evaluation."
fi

missing=0
for seq in $SEQUENCES; do
  bag="$WS/dataset_tank/data/${seq}.bag"
  if [ ! -f "$bag" ]; then
    log "[ERROR] missing bag: $bag"
    missing=1
  fi
done
if [ "$missing" = "1" ]; then
  exit 3
fi

child_do_build="$DO_BUILD"
total=0
failed=0

for seq in $SEQUENCES; do
  bag="$WS/dataset_tank/data/${seq}.bag"
  for sensor in $SENSORS; do
    total=$((total + 1))
    child_tag="${RUN_TAG}_${sensor}_${seq}"
    child_summary="$WS/dataset_tank/results/aidvo_full_${child_tag}/summary_tank.csv"

    log "============================================================"
    log "[CASE] seq=$seq sensor=$sensor bag=$bag child_tag=$child_tag"

    env \
      WS="$WS" \
      RUN_TAG="$child_tag" \
      DO_BUILD="$child_do_build" \
      RUN_ALL_BAGS=0 \
      ONLY_DATASET=tank \
      ONLY_SENSOR="$sensor" \
      TANK_BAG="$bag" \
      RUNS_PER_CASE="$RUNS_PER_CASE" \
      BAG_START="$BAG_START" \
      BAG_DURATION="$BAG_DURATION" \
      AIDVO_MODES="$AIDVO_MODES" \
      RUN_EVO="$RUN_EVO" \
      RUN_TIMEOUT="$RUN_TIMEOUT" \
      SAVE_TRAJ_TIMEOUT="$SAVE_TRAJ_TIMEOUT" \
      EVO_TIMEOUT="$EVO_TIMEOUT" \
      ENABLE_ADAPTIVE_LOGGING="$ENABLE_ADAPTIVE_LOGGING" \
      bash "$FULL_SCRIPT" 2>&1 | tee -a "$MASTER_LOG"

    rc=${PIPESTATUS[0]}
    child_do_build=0

    if [ "$rc" != "0" ]; then
      log "[WARN] child test returned non-zero rc=$rc seq=$seq sensor=$sensor"
      failed=$((failed + 1))
    fi

    append_summary "$child_summary"
  done
done

find "$WS/dataset_tank/results" -type d -path "*aidvo_full_${RUN_TAG}_*/*/evo" | sort > "$EVO_DIR_LIST" 2>/dev/null || true
find "$WS/dataset_tank/results" -type f \
  \( -name "matched_trajectory_xy.png" \
  -o -name "matched_trajectory_xz.png" \
  -o -name "matched_trajectory_yz.png" \
  -o -name "matched_trajectory_3d.png" \
  -o -name "matched_position_error.png" \
  -o -name "matched_position_components.png" \
  -o -name "evo_ape_plot_xy.pdf" \
  -o -name "evo_rpe_plot_xy.pdf" \
  -o -name "evo_traj_plot_xy.pdf" \) \
  -path "*aidvo_full_${RUN_TAG}_*" | sort > "$EVO_PLOT_LIST" 2>/dev/null || true

log "============================================================"
log "test_tank_4seq_mono_stereo_evo_20260610 finished"
log "Total sensor-sequence groups: $total"
log "Child non-zero returns: $failed"
log "Master log: $MASTER_LOG"
log "Combined summary: $COMBINED_SUMMARY"
log "Evo directories: $EVO_DIR_LIST"
log "Evo plots: $EVO_PLOT_LIST"

if [ -s "$COMBINED_SUMMARY" ]; then
  log "Combined status counts:"
  awk -F, 'NR>1 {gsub(/"/,"",$6); c[$6]++} END {for (k in c) print "  " k ": " c[k]}' "$COMBINED_SUMMARY" | tee -a "$MASTER_LOG"
else
  log "[WARN] combined summary is empty"
fi

exit 0
