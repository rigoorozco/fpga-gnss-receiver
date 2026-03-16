# HDL Workflow

## Source Locations

- VHDL RTL: `rtl/vhdl/`
- VHDL testbenches: `tb/vhdl/`
- Lint scripts: `lint/`
- Simulation scripts: `sim/`
- Synthesis check script: `synth/`

## Compile Order

- VHDL compile order is defined in `vhdl.files`.
- The scripts consume these files directly and should be treated as source of truth.

## Top Units

- VHDL TB top: `gps_l1_ca_phase2_tb`

## Verification Source of Truth

- VHDL lint/check: `make lint-vhdl`
- Smoke simulations: `make sim-smoke`
- Basic regression: `make sim-regress`
- Phase 3 metrics (non-gating): `make phase3-eval`
- Phase 3 metrics (gating thresholds): `make phase3-gate`
- Schematic generation (Docker): `make schematic`
- Schematic generation (local tools): `make schematic-local`
- Optional smoke-time TB overrides: set `TB_GENERIC_ARGS`, for example
  - `TB_GENERIC_ARGS="-gG_MAX_FILE_SAMPLES=50000" make sim-smoke`

Supporting docs:
- `docs/phase1_register_map.md`
- `docs/phase2_register_map.md`
- `docs/packet_definition.md`
- `docs/fixed_point_and_loop_notes.md`
- `docs/phase1_verification_notes.md`
- `docs/phase2_verification_notes.md`
- `docs/phase3_verification_notes.md`

## Definition of Done

- `make lint-vhdl` passes
- `make sim-smoke` passes
- No new latch/multi-driver/width/uninitialized warnings introduced
- Any interface changes are reflected in docs and testbenches

## Phase 2 Module Map

- Top integration: `gps_l1_ca_phase2_top`
- Shared package: `gps_l1_ca_pkg`
- Control/status: `gps_l1_ca_ctrl_phase2`
- Sample ingress: `axis_sample_ingress`
- Shared acquisition scheduler: `gps_l1_ca_acq_sched`
- Shared acquisition engine: `gps_l1_ca_acq`
- Channel bank: `gps_l1_ca_chan_bank`
- Per-channel tracking: `gps_l1_ca_track_chan`
- Per-channel nav-bit extraction: `gps_l1_ca_nav`
- Navigation store: `gps_l1_ca_nav_store`
- Observables engine: `gps_l1_ca_observables`
- PVT engine: `gps_l1_ca_pvt`
- Report packing: `gps_l1_ca_report_phase2`
- UART transport: `uart_tx`
