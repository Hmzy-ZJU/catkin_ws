#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/aidvo_smoke_common.sh"
ROOT="${EUROC_ROOT:-$WS/dataset_EuRoc}"
run_aidvo_dataset mono-inertial euroc euroc_mono_inertial.launch \
  "$WS/src/orb_slam3_ros/config/Monocular-Inertial/EuRoC.yaml" \
  "$ROOT/data" "*.bag" "$ROOT/aidvo_results"
