#!/usr/bin/env bash
set -euo pipefail

NVC_STDERR_LEVEL="${NVC_STDERR_LEVEL:-none}"
NVC_STOP_TIME="${NVC_STOP_TIME:-4ms}"
NVC_WAVE_FILE="${NVC_WAVE_FILE:-sim/gps_l1_ca_phase1_tb.fst}"

./lint/lint_vhdl.sh

if command -v nvc >/dev/null 2>&1; then
  echo "==> Running VHDL smoke simulation"
  echo "    NVC stderr level: ${NVC_STDERR_LEVEL}"
  nvc --std=2008 \
    --stderr="${NVC_STDERR_LEVEL}" \
    -r gps_l1_ca_phase1_tb \
    --stop-time="${NVC_STOP_TIME}" \
    --wave="${NVC_WAVE_FILE}"
else
  echo "error: nvc not found. Cannot run VHDL smoke simulation."
  exit 1
fi

echo "VHDL smoke simulation passed."
