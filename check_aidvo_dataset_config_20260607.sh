#!/usr/bin/env bash
set -o pipefail

# Check dataset topics and launch/config coverage before running full AIDVO tests.
#
# Usage:
#   cd ~/catkin_ws
#   bash check_aidvo_dataset_config_20260607.sh

if [ -z "$WS" ]; then WS="$HOME/catkin_ws"; fi

RESULT_DIR="$WS/results/aidvo_dataset_config_check_$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$RESULT_DIR/check.log"
mkdir -p "$RESULT_DIR"
exec > >(tee -a "$LOG_FILE") 2>&1

check_file() {
  if [ -f "$1" ]; then echo "[OK] $1"; else echo "[MISSING] $1"; fi
}

check_topic() {
  local bag="$1"
  local topic="$2"
  if rosbag info "$bag" 2>/dev/null | grep -q "$topic"; then
    echo "[OK] topic $topic"
  else
    echo "[MISSING] topic $topic"
  fi
}

check_dataset() {
  local name="$1"
  local root="$2"
  shift 2
  echo "============================================================"
  echo "[DATASET] $name"
  echo "[ROOT] $root"
  if [ ! -d "$root/data" ]; then
    echo "[MISSING] $root/data"
    return 0
  fi
  echo "[BAGS]"
  find "$root/data" -maxdepth 1 -type f -name "*.bag" | sort
  echo "[GT]"
  find "$root/GT" -maxdepth 1 -type f 2>/dev/null | sort || true
  local first_bag
  first_bag="$(find "$root/data" -maxdepth 1 -type f -name "*.bag" | sort | head -n 1)"
  if [ -z "$first_bag" ]; then
    echo "[MISSING] no bag found"
    return 0
  fi
  echo "[FIRST_BAG] $first_bag"
  echo "[TOPIC_CHECK]"
  for topic in "$@"; do
    check_topic "$first_bag" "$topic"
  done
}

cd "$WS" || exit 1

echo "[INFO] WS=$WS"
echo "[INFO] output=$LOG_FILE"
command -v rosbag >/dev/null 2>&1 || { echo "[ERROR] rosbag not found"; exit 1; }

echo "============================================================"
echo "[LAUNCH/CONFIG CHECK]"
check_file "$WS/src/orb_slam3_ros/launch/euroc_mono.launch"
check_file "$WS/src/orb_slam3_ros/launch/euroc_stereo.launch"
check_file "$WS/src/orb_slam3_ros/launch/euroc_mono_inertial.launch"
check_file "$WS/src/orb_slam3_ros/launch/euroc_stereo_inertial.launch"
check_file "$WS/src/orb_slam3_ros/config/Monocular/EuRoc/EuRoc_on_11.yaml"
check_file "$WS/src/orb_slam3_ros/config/Stereo/EuRoC.yaml"
check_file "$WS/src/orb_slam3_ros/config/Monocular-Inertial/EuRoC.yaml"
check_file "$WS/src/orb_slam3_ros/config/Stereo-Inertial/EuRoC.yaml"

check_file "$WS/src/orb_slam3_ros/launch/tank_mono.launch"
check_file "$WS/src/orb_slam3_ros/launch/tank_stereo.launch"
check_file "$WS/src/orb_slam3_ros/launch/tank_mono_inertial.launch"
check_file "$WS/src/orb_slam3_ros/launch/tank_stereo_inertial.launch"
check_file "$WS/src/orb_slam3_ros/config/Monocular/Tank/tank_on_11.yaml"
check_file "$WS/src/orb_slam3_ros/config/Stereo/Tank/Tank_stereo_on_11.yaml"
check_file "$WS/src/orb_slam3_ros/config/Monocular-Inertial/Tank/tank_on_11.yaml"
check_file "$WS/src/orb_slam3_ros/config/Stereo-Inertial/Tank_stereo_inertial_on_11.yaml"

check_file "$WS/src/orb_slam3_ros/launch/aqualoc_harbor_mono.launch"
check_file "$WS/src/orb_slam3_ros/launch/aqualoc_harbor_mono_inertial.launch"
check_file "$WS/src/orb_slam3_ros/config/Monocular/Aquacular_harbor/all/Aqualoc_harbor_on_11.yaml"
check_file "$WS/src/orb_slam3_ros/config/Monocular-Inertial/Aqualoc_harbor.yaml"

check_dataset "EuRoC" "$WS/dataset_EuRoc" "/cam0/image_raw" "/cam1/image_raw" "/imu0"
check_dataset "Tank" "$WS/dataset_tank" "/camera/left/image_dehazed/compressed" "/camera/right/image_dehazed/compressed" "/imu/data"
check_dataset "Harbor" "$WS/dataset_harbor" "/camera/image_raw" "/rtimulib_node/imu"

echo "============================================================"
echo "[DONE] dataset/config check"
echo "[LOG] $LOG_FILE"
