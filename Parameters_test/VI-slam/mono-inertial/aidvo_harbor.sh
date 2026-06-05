#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/aidvo_smoke_common.sh"
ROOT="${HARBOR_ROOT:-$WS/dataset_harbor}"
run_aidvo_dataset mono-inertial harbor aqualoc_harbor_mono_inertial.launch \
  "$WS/src/orb_slam3_ros/config/Monocular-Inertial/Aqualoc_harbor.yaml" \
  "$ROOT/data" "*.bag" "$ROOT/aidvo_results"
