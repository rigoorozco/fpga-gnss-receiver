# HDL Starter Kit (GSD-Driven)

This repository is a concrete starter implementation of the workflow described in `Phase-1-Plans-and-Goal.md`, `Assumptions.md`, and `Outline.md`:

- GSD-2 as project manager/orchestrator
- HDL tools as ground truth (NVC + vendor synthesis flow)
- Mechanical verification commands through scripts and Make targets

This now includes a Phase 2 VHDL-first GPS L1 C/A receiver scaffold from `Phase-2-Plans-and-Goal.md`:

- fixed `2 MSPS` `cs16` ingress
- shared acquisition scheduler + shared acquisition engine
- configurable multi-channel tracking bank (default `5`)
- per-channel nav-bit handling and shared nav store
- observables engine and first FPGA PVT placeholder solver
- UART report path with channel / observables / PVT packet types
- simulation PVT logs in human-readable `Lat/Long/Height` with receiver-time / epoch tags
- expanded control/status register bank

## Layout

```text
rtl/
  vhdl/
tb/
  vhdl/
sim/
  scripts/
lint/
  scripts/
synth/
constraints/
docs/
```

## Canonical Commands

```bash
make lint-vhdl
make sim-smoke
make sim-regress
make synth-check
make waves
```

See `docs/hdl-workflow.md` for details.
