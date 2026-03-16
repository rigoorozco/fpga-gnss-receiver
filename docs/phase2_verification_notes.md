# Phase 2 Verification Notes

## Implemented Verification Hooks
- Multi-channel channel-bank integration (`gps_l1_ca_chan_bank`) with default 5 channels.
- Acquisition scheduling and assignment (`gps_l1_ca_acq_sched`).
- Nav store with word framing hooks, subframe tracking, and ephemeris-state placeholders.
- Observables engine with TOF-based pseudorange model and clock-correction hook.
- Iterative least-squares PVT backend (`gps_l1_ca_pvt`) using per-satellite state from observables.
- End-to-end Phase 2 testbench: `tb/vhdl/gps_l1_ca_phase2_tb.vhd`.

## Smoke Flow
- `make lint-vhdl`
- `make sim-smoke`

Default smoke simulation:
- top TB: `gps_l1_ca_phase2_tb`
- waveform: `sim/gps_l1_ca_phase2_tb.fst`
- default stop-time: `40ms` (override with `NVC_STOP_TIME=...`)

## Expected Bring-Up Behavior
- Scheduler allocates channels and drives per-channel handoff.
- Channel status packets appear first during pull-in.
- Observables packets appear once channels are locked.
- PVT solver updates appear once at least 4 observables are valid.
- Simulation logs print PVT as `Lat/Long/Height` with receiver runtime and observables epoch.

## Notes
- The backend structure now matches Phase-2 decomposition (nav store -> observables -> PVT LS), but still uses scaffold-quality signal models and not production-grade GNSS decode/tracking math yet.
- Packetization and report records are separated so precision and packet formats can evolve independently.
