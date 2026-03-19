#!/usr/bin/env bash
set -u -o pipefail

UNIT_TBS=(
  gps_l1_ca_gain_ctrl_tb
  gps_l1_ca_acq_sched_tb
  gps_l1_ca_acq_tb
  gps_l1_ca_chan_bank_tb
  gps_l1_ca_chan_bank_nav_store_tb
  gps_l1_ca_nav_store_tb
  gps_l1_ca_observables_tb
  gps_l1_ca_pvt_tb
)

UNIT_TB_STOP_TIME="${UNIT_TB_STOP_TIME:-3ms}"
NVC_STDERR_LEVEL="${NVC_STDERR_LEVEL:-none}"
UNIT_RUN_LINT="${UNIT_RUN_LINT:-1}"
UNIT_ACQ_WAVE_FILE="${UNIT_ACQ_WAVE_FILE:-sim/gps_l1_ca_acq_tb.fst}"
UNIT_ACQ_IMPL_FFT="${UNIT_ACQ_IMPL_FFT:-false}"

if ! command -v nvc >/dev/null 2>&1; then
  echo "error: nvc not found. Cannot run unit TBs."
  exit 1
fi

if [[ "${UNIT_RUN_LINT}" == "1" || "${UNIT_RUN_LINT}" == "true" || "${UNIT_RUN_LINT}" == "TRUE" ]]; then
  ./lint/lint_vhdl.sh
fi

echo "==> Running VHDL unit testbenches"
echo "    stop-time: ${UNIT_TB_STOP_TIME}"
echo "    stderr level: ${NVC_STDERR_LEVEL}"

pass_count=0
fail_count=0
failed_tbs=()

for tb in "${UNIT_TBS[@]}"; do
  echo "-- ${tb}"
  elab_cmd=(nvc --std=2008 --stderr="${NVC_STDERR_LEVEL}" -e)
  run_cmd=(
    nvc --std=2008
    --stderr="${NVC_STDERR_LEVEL}"
    -r "${tb}"
    --stop-time="${UNIT_TB_STOP_TIME}"
  )

  if [[ "${tb}" == "gps_l1_ca_acq_tb" ]]; then
    elab_cmd+=("-gG_DUT_ACQ_IMPL_FFT=${UNIT_ACQ_IMPL_FFT}")
    mkdir -p "$(dirname "${UNIT_ACQ_WAVE_FILE}")"
    run_cmd+=(--wave="${UNIT_ACQ_WAVE_FILE}")
    echo "   wave: ${UNIT_ACQ_WAVE_FILE}"
    echo "   acq impl fft: ${UNIT_ACQ_IMPL_FFT}"
  fi
  elab_cmd+=("${tb}")

  if "${elab_cmd[@]}" && \
     "${run_cmd[@]}"; then
    pass_count=$((pass_count + 1))
  else
    fail_count=$((fail_count + 1))
    failed_tbs+=("${tb}")
  fi
done

echo "==> Unit TB summary: pass=${pass_count} fail=${fail_count}"
if [[ ${fail_count} -ne 0 ]]; then
  echo "Failed TBs: ${failed_tbs[*]}"
  exit 1
fi

echo "All unit TBs passed."
