#!/usr/bin/env bash
set -euo pipefail

./lint/lint_vhdl.sh
./lint/lint_sv.sh

if command -v nvc >/dev/null 2>&1; then
  echo "==> Running VHDL smoke simulation"
  nvc --std=2008 -r gps_l1_ca_phase1_tb --stop-time=4ms --wave=sim/gps_l1_ca_phase1_tb.fst
else
  echo "error: nvc not found. Cannot run VHDL smoke simulation."
  exit 1
fi

if command -v verilator >/dev/null 2>&1; then
  echo "==> Building/running SV smoke simulation"
  verilator -Wall --binary -f sv.f --top-module top_tb --Mdir sim/obj_dir
  sim/obj_dir/Vtop_tb
else
  echo "error: verilator not found. Cannot run SV smoke simulation."
  exit 1
fi

echo "Smoke simulations passed."
