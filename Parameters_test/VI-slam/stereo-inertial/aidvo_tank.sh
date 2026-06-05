#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/aidvo_smoke_common.sh"
ROOT="${TANK_ROOT:-$WS/dataset_tank}"
run_aidvo_dataset stereo-inertial tank tank_stereo_inertial.launch \
  "$WS/src/orb_slam3_ros/config/Stereo-Inertial/Tank_stereo_inertial_on_11.yaml" \
  "$ROOT/data" "*.bag" "$ROOT/aidvo_results"
