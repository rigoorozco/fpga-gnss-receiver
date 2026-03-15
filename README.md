# HDL Starter Kit (GSD-Driven)

This repository is a concrete starter implementation of the workflow described in `Intro.md`:

- GSD-2 as project manager/orchestrator
- HDL tools as ground truth (NVC, slang, Verilator, optional synthesis)
- Mechanical verification commands through scripts and Make targets

This now includes a Phase 1 VHDL-first GNSS receiver scaffold from `Phase-1-Plans-and-Goal.md`:

- fixed `2 MSPS` `cs16` ingress
- shared acquisition block + one tracking channel
- nav-bit extraction
- UART report path
- simple control/status register bank

## Layout

```text
rtl/
  vhdl/
  sv/
tb/
  vhdl/
  sv/
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
make lint-sv
make sim-smoke
make sim-regress
make synth-check
make waves
```

See `docs/hdl-workflow.md` for details.
