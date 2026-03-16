#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-hpretl/iic-osic-tools:latest}"

# If docker group permissions are not refreshed, run this in your shell first:
#   newgrp docker
# We keep this as a pre-step instead of running newgrp here because non-interactive
# scripts cannot reliably switch the caller's group context.

echo "==> Running schematic generation in Docker"
echo "    image: ${DOCKER_IMAGE}"
echo "    repo:  ${REPO_ROOT}"
echo "    note: run 'newgrp docker' first if needed for permissions"

docker_cmd=(
  docker run --rm -it
  --network host
  -v "${REPO_ROOT}:/work"
  -w /work
)

for var_name in TOPS OUT_DIR VHDL_STD SHOW_FORMAT RUN_DOT; do
  if [[ -n "${!var_name:-}" ]]; then
    docker_cmd+=( -e "${var_name}=${!var_name}" )
  fi
done

docker_cmd+=(
  "${DOCKER_IMAGE}"
  --skip
  bash -lc "./synth/gen_schematics_ghdl_yosys.sh"
)

"${docker_cmd[@]}"
