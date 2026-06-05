#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/aidvo_smoke_common.sh"
ROOT="${TANK_ROOT:-$WS/dataset_tank}"
run_aidvo_dataset mono tank tank_mono.launch \
  "$WS/src/orb_slam3_ros/config/Monocular/Tank/tank_on_11.yaml" \
  "$ROOT/data" "*.bag" "$ROOT/aidvo_results"
