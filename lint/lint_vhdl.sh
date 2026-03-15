#!/usr/bin/env bash
set -euo pipefail

if ! command -v nvc >/dev/null 2>&1; then
  echo "error: nvc not found. Install NVC to run VHDL checks."
  exit 1
fi

if [[ ! -f vhdl.files ]]; then
  echo "error: vhdl.files not found."
  exit 1
fi

mapfile -t vhdl_sources < <(grep -vE '^\s*(#|$)' vhdl.files)

if [[ ${#vhdl_sources[@]} -eq 0 ]]; then
  echo "error: vhdl.files has no source entries."
  exit 1
fi

echo "==> NVC analyze"
nvc --std=2008 -a "${vhdl_sources[@]}"

echo "==> NVC elaborate gps_l1_ca_phase1_tb"
nvc --std=2008 -e gps_l1_ca_phase1_tb

echo "VHDL lint/check passed."
