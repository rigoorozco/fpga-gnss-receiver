#!/usr/bin/env bash
set -euo pipefail

VHDL_TB_TOP="${VHDL_TB_TOP:-gps_l1_ca_phase2_tb}"
NVC_STDERR_LEVEL="${NVC_STDERR_LEVEL:-none}"
NVC_STOP_TIME="${NVC_STOP_TIME:-40ms}"
NVC_WAVE_FILE="${NVC_WAVE_FILE:-sim/gps_l1_ca_phase2_tb.fst}"
TB_GENERIC_ARGS="${TB_GENERIC_ARGS:-}"

./lint/lint_vhdl.sh

if command -v nvc >/dev/null 2>&1; then
  echo "==> Running VHDL smoke simulation"
  echo "    NVC stderr level: ${NVC_STDERR_LEVEL}"
  if [[ -n "${TB_GENERIC_ARGS}" ]]; then
    # Optional, space-delimited nvc generic args such as:
    # TB_GENERIC_ARGS="-gG_MAX_FILE_SAMPLES=50000"
    read -r -a tb_generic_argv <<< "${TB_GENERIC_ARGS}"
  else
    tb_generic_argv=()
  fi
  nvc --std=2008 \
    --stderr="${NVC_STDERR_LEVEL}" \
    -r "${VHDL_TB_TOP}" \
    "${tb_generic_argv[@]}" \
    --stop-time="${NVC_STOP_TIME}" \
    --wave="${NVC_WAVE_FILE}"
else
  echo "error: nvc not found. Cannot run VHDL smoke simulation."
  exit 1
fi

echo "VHDL smoke simulation passed."
