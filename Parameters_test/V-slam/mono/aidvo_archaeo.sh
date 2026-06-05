#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/aidvo_smoke_common.sh"
ROOT="${ARCHAEO_ROOT:-$WS/dataset_archaeo}"
run_aidvo_dataset mono archaeo aqualoc_archaeo_mono.launch \
  "$WS/src/orb_slam3_ros/config/Monocular/Aquacular_archaeo/Aqualoc_archaeo_on_11.yaml" \
  "$ROOT/data" "*.bag" "$ROOT/aidvo_results"
