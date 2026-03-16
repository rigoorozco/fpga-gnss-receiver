#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

TOPS_ENV="${TOPS:-gps_l1_ca_phase1_top gps_l1_ca_phase2_top}"
OUT_DIR="${OUT_DIR:-build/schematics}"
VHDL_STD="${VHDL_STD:-08}"
SHOW_FORMAT="${SHOW_FORMAT:-dot}"
RUN_DOT="${RUN_DOT:-1}"

if ! command -v yosys >/dev/null 2>&1; then
  echo "error: yosys not found."
  exit 1
fi

if ! yosys -m ghdl -p "help ghdl" >/dev/null 2>&1; then
  echo "error: yosys ghdl plugin not available (expected: yosys -m ghdl)."
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

IFS=' ' read -r -a tops <<< "${TOPS_ENV}"
if [[ ${#tops[@]} -eq 0 ]]; then
  echo "error: TOPS resolved to empty list."
  exit 1
fi

mkdir -p "${OUT_DIR}"

src_args=""
for f in "${vhdl_sources[@]}"; do
  src_args+=" ${f}"
done

echo "==> Generating GHDL+Yosys schematics"
echo "    tops: ${tops[*]}"
echo "    out:  ${OUT_DIR}"
echo "    std:  ${VHDL_STD}"
echo "    fmt:  ${SHOW_FORMAT}"

for top in "${tops[@]}"; do
  prefix="${OUT_DIR}/${top}"
  echo "-- ${top}"
  yosys -m ghdl -p "ghdl --std=${VHDL_STD}${src_args} -e ${top}; prep -top ${top}; show -format ${SHOW_FORMAT} -viewer none -prefix ${prefix}"

  if [[ "${SHOW_FORMAT}" == "dot" ]] && [[ "${RUN_DOT}" == "1" || "${RUN_DOT}" == "true" || "${RUN_DOT}" == "TRUE" ]]; then
    if command -v dot >/dev/null 2>&1; then
      dot -Tsvg "${prefix}.dot" -o "${prefix}.svg"
      echo "   wrote ${prefix}.dot and ${prefix}.svg"
    else
      echo "   wrote ${prefix}.dot (graphviz 'dot' not found; SVG skipped)"
    fi
  else
    echo "   wrote ${prefix}.${SHOW_FORMAT}"
  fi
done

echo "Schematic generation complete."
