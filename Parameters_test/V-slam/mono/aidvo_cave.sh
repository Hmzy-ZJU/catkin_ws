#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/aidvo_smoke_common.sh"
ROOT="${CAVE_ROOT:-$WS/dataset_cave}"
run_aidvo_dataset mono cave cave_mono.launch \
  "$WS/src/orb_slam3_ros/config/Monocular/cave/cave_on_11.yaml" \
  "$ROOT/data" "*.bag" "$ROOT/aidvo_results"
