#!/usr/bin/env bash
set -euo pipefail

NVC_STDERR_LEVEL="${NVC_STDERR_LEVEL:-none}"
ACQ_TB_STOP_TIME="${ACQ_TB_STOP_TIME:-70000ms}"
ACQ_INPUT_FILE="${ACQ_INPUT_FILE:-2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN_2msps.dat}"
ACQ_FILE_SAMPLE_RATE_SPS="${ACQ_FILE_SAMPLE_RATE_SPS:-2000000}"
ACQ_DUT_SAMPLE_RATE_SPS="${ACQ_DUT_SAMPLE_RATE_SPS:-2000000}"
ACQ_MAX_FILE_SAMPLES="${ACQ_MAX_FILE_SAMPLES:-0}"
ACQ_WAVE_FILE="${ACQ_WAVE_FILE:-}"
ACQ_IMPL_FFT="${ACQ_IMPL_FFT:-false}"
NVC_HEAP_SIZE="${NVC_HEAP_SIZE:-512m}"

./lint/lint_vhdl.sh

tb_generic_argv=(
  "-gG_USE_FILE_INPUT=true"
  "-gG_INPUT_FILE=${ACQ_INPUT_FILE}"
  "-gG_FILE_SAMPLE_RATE_SPS=${ACQ_FILE_SAMPLE_RATE_SPS}"
  "-gG_DUT_SAMPLE_RATE_SPS=${ACQ_DUT_SAMPLE_RATE_SPS}"
  "-gG_MAX_FILE_SAMPLES=${ACQ_MAX_FILE_SAMPLES}"
  "-gG_DUT_ACQ_IMPL_FFT=${ACQ_IMPL_FFT}"
)

echo "==> Running gps_l1_ca_acq_tb with GNSS stimulus file"
echo "    input: ${ACQ_INPUT_FILE}"
echo "    file Fs: ${ACQ_FILE_SAMPLE_RATE_SPS}"
echo "    DUT  Fs: ${ACQ_DUT_SAMPLE_RATE_SPS}"
echo "    max injected samples: ${ACQ_MAX_FILE_SAMPLES}"
echo "    acq impl fft: ${ACQ_IMPL_FFT}"
echo "    stop-time: ${ACQ_TB_STOP_TIME}"
echo "    heap: ${NVC_HEAP_SIZE}"
if [[ -n "${ACQ_WAVE_FILE}" ]]; then
  echo "    wave: ${ACQ_WAVE_FILE}"
  mkdir -p "$(dirname "${ACQ_WAVE_FILE}")"
else
  echo "    wave: (disabled)"
fi

nvc_cmd=(
  nvc --std=2008
  -H "${NVC_HEAP_SIZE}"
  --stderr="${NVC_STDERR_LEVEL}"
  -e
  "${tb_generic_argv[@]}"
  gps_l1_ca_acq_tb
  -r
  gps_l1_ca_acq_tb
  --stop-time="${ACQ_TB_STOP_TIME}"
)

if [[ -n "${ACQ_WAVE_FILE}" ]]; then
  nvc_cmd+=(--wave="${ACQ_WAVE_FILE}")
fi

"${nvc_cmd[@]}"

echo "gps_l1_ca_acq_tb file-replay simulation passed."
