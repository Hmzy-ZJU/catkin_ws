#!/usr/bin/env bash
set -euo pipefail

# ROS online deployment validation wrapper.
# This keeps the full online/rosbag test as an engineering validation path.
# It can run the complete matrix, or only the ORB3-meaningful subset.

WS="${WS:-$HOME/catkin_ws}"
RUN_TAG="${RUN_TAG:-orb3_ros_deploy_$(date +%Y%m%d_%H%M%S)}"
AIDVO_MODES="${AIDVO_MODES:-off,fixed,rule}"
RUNS_PER_CASE="${RUNS_PER_CASE:-3}"
RUN_ALL_BAGS="${RUN_ALL_BAGS:-1}"
BAG_START="${BAG_START:-0}"
BAG_DURATION="${BAG_DURATION:-0}"
RUN_PROFILE="${RUN_PROFILE:-meaningful}"
DO_BUILD="${DO_BUILD:-0}"

run_one() {
  local dataset="$1"
  local sensor="$2"
  local tag="${RUN_TAG}_${dataset}_${sensor}"
  echo "===== ROS deploy case: dataset=$dataset sensor=$sensor tag=$tag ====="
  DO_BUILD="$DO_BUILD" \
  RUN_TAG="$tag" \
  RUN_ALL_BAGS="$RUN_ALL_BAGS" \
  RUNS_PER_CASE="$RUNS_PER_CASE" \
  BAG_START="$BAG_START" \
  BAG_DURATION="$BAG_DURATION" \
  AIDVO_MODES="$AIDVO_MODES" \
  ONLY_DATASET="$dataset" \
  ONLY_SENSOR="$sensor" \
  bash "$WS/test_aidvo_full_20260607.sh"
  DO_BUILD=0
}

if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi
if [ -f "$WS/devel/setup.bash" ]; then source "$WS/devel/setup.bash"; fi

cd "$WS"

if [ "$RUN_PROFILE" = "full" ]; then
  echo "===== ROS deploy full matrix tag=$RUN_TAG ====="
  DO_BUILD="$DO_BUILD" \
  RUN_TAG="$RUN_TAG" \
  RUN_ALL_BAGS="$RUN_ALL_BAGS" \
  RUNS_PER_CASE="$RUNS_PER_CASE" \
  BAG_START="$BAG_START" \
  BAG_DURATION="$BAG_DURATION" \
  AIDVO_MODES="$AIDVO_MODES" \
  bash "$WS/test_aidvo_full_20260607.sh"
  exit $?
fi

if [ "$RUN_PROFILE" != "meaningful" ]; then
  echo "[ERROR] RUN_PROFILE must be meaningful or full"
  exit 1
fi

# Meaningful ORB-SLAM3 evaluation subset based on current observations:
# - EuRoC: all four modes are stable enough for AIDVO policy comparison.
# - Harbor/AQUALOC: mono only is stable enough for AIDVO policy comparison.
# - Tank and water inertial modes are kept for AQUA-SLAM migration/diagnostics,
#   not for the main ORB3 paper comparison.
run_one euroc mono
run_one euroc stereo
run_one euroc mono-inertial
run_one euroc stereo-inertial
run_one harbor mono

SUMMARY_ROOT="$WS/results/aidvo_ros_deploy_${RUN_TAG}"
mkdir -p "$SUMMARY_ROOT"
COMBINED="$SUMMARY_ROOT/combined_summary.csv"
first=1
for f in "$WS"/results/aidvo_full_${RUN_TAG}_*/test_aidvo_full_summary.csv; do
  [ -f "$f" ] || continue
  if [ "$first" = "1" ]; then
    cat "$f" > "$COMBINED"
    first=0
  else
    tail -n +2 "$f" >> "$COMBINED"
  fi
done

echo "Combined ROS deploy summary: $COMBINED"
