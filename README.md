# FPGA GNSS Receiver (GPS L1 C/A, VHDL)

> [!WARNING]
> This repository has a **high potential for AI slop**. Treat docs, plans, and generated code as draft material until validated by simulation, synthesis, and testbench evidence.

This project is an FPGA-oriented, VHDL-first GPS L1 C/A receiver effort with:

- fixed-rate `cs16` sample ingress (target `2 MSPS`)
- shared acquisition scheduler + acquisition engine
- configurable multi-channel tracking bank
- per-channel nav-bit extraction + shared navigation store
- observables + placeholder PVT/reporting pipeline
- phase-based verification flow through `make` targets

Current planning and scope documents:

- `Outline.md`
- `Assumptions.md`
- `Phase-1-Plans-and-Goal.md`
- `Phase-2-Plans-and-Goal.md`
- `Phase-3-Plans-and-Goal.md`

## Repository Layout

```text
rtl/vhdl/      # RTL modules
tb/vhdl/       # VHDL testbenches
sim/           # simulation and regression scripts
lint/          # lint/check scripts
synth/         # synthesis/schematic scripts
docs/          # register maps, packet formats, verification notes
vhdl.files     # compile order source of truth
```

## Quick Start

```bash
make help
make lint-vhdl
make sim-smoke
make sim-regress
```

## GNSS Sample Data

Some test flows use the GNSS-SDR sample capture. Fetch and prepare it with:

```bash
make fetch-gnss-data
```

This creates/updates:

- `2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/`
- `2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat` (symlink to the extracted raw `.dat`, 4 MSPS)
- `2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN_2msps.dat` (symlink to pre-decimated replay `.dat`, 2 MSPS)

## Canonical Verification / Build Commands

```bash
make lint-vhdl
make sim-smoke
make sim-unit
make sim-chan-bank
make sim-chan-bank-nav-store
make sim-acq-file
make sim-acq-equiv
make sim-regress
make phase3-eval
make phase3-gate
make synth-check
make schematic
make schematic-local
make waves
```

## Notes

- `vhdl.files` defines compile order and should be treated as the source of truth.
- Main workflow details: `docs/hdl-workflow.md`
- Register maps and packet formats: `docs/phase1_register_map.md`, `docs/phase2_register_map.md`, `docs/packet_definition.md`
