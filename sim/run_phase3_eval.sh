#!/usr/bin/env bash
set -euo pipefail

PHASE3_LOG_FILE="${PHASE3_LOG_FILE:-sim/phase3_eval.log}"
PHASE3_EXPECTED_FILE="${PHASE3_EXPECTED_FILE:-tb/txt/expected_output.txt}"
PHASE3_STRICT="${PHASE3_STRICT:-0}"

mkdir -p "$(dirname "${PHASE3_LOG_FILE}")"

echo "==> Running Phase 3 evaluation smoke"
./sim/run_smoke.sh 2>&1 | tee "${PHASE3_LOG_FILE}"

strict_args=()
if [[ "${PHASE3_STRICT}" == "1" ]]; then
  strict_args+=(--strict)
fi

echo "==> Comparing against GNSS-SDR reference"
python3 sim/phase3_compare.py \
  --sim-log "${PHASE3_LOG_FILE}" \
  --expected "${PHASE3_EXPECTED_FILE}" \
  "${strict_args[@]}"
