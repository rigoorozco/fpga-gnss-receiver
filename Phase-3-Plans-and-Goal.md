# Phase 3 Plans and Goal

## 1. Phase 3 Goal

The goal of Phase 3 is to remove scaffold-quality logic from the existing GPS L1 C/A FPGA receiver and replace it with physically meaningful GNSS processing so results move as close as practical to GNSS-SDR on the same dataset.

Phase 3 should transform the current design from:

- architecture-valid but placeholder-heavy backend behavior

into:

- measurement-valid and navigation-valid receiver behavior with quantified alignment to GNSS-SDR outputs.

Primary Phase 3 target:

- keep the current Phase 2 module boundaries and channelized architecture
- replace placeholder acquisition/tracking/nav/observables/PVT math with true GNSS equations
- validate against `tb/txt/expected_output.txt` style outputs and GNSS-SDR-derived references on the same input file

## 2. Why Phase 3 Exists

Phase 2 established a working architectural decomposition:

- channel bank
- nav store
- observables engine
- PVT engine
- report path

But several blocks still use scaffold logic that prevents realistic position output. Phase 3 exists to remove those scaffolds while preserving the structure needed for later features from `Outline.md` (more signals, more constellations, richer navigation).

## 3. Compliance Targets

### 3.1 Required Compliance with Assumptions.md
- Keep processing in FPGA logic for receiver pipeline blocks.
- Keep `cs16` `2 MSPS` ingress assumptions.
- Keep simple control/status bus model.
- Keep AXI-Stream style datapath practices where practical.
- Keep VHDL-first implementation for RTL.
- Keep UART-oriented reporting path.

### 3.2 Required Compliance with Outline.md
- Preserve block split: ingress, acquisition, tracking, telemetry decode, observables, PVT.
- Preserve channel-oriented, scalable architecture.
- Keep shared vs per-channel boundaries explicit.
- Keep clean interfaces so future bands/constellations can reuse the same backend pattern.

### 3.3 Required Continuity with Phase 1 and Phase 2
- Do not collapse modular boundaries into a monolithic core.
- Keep current top-level partition (`gps_l1_ca_phase2_top`) and evolve internals.
- Keep packetization separated from internal numeric records.

## 4. Phase 3 Scope

### 4.1 In Scope
- GPS L1 C/A only (still single-signal, single-band).
- Remove placeholder logic in:
  - acquisition metric generation
  - tracking lock/discriminator behavior
  - nav word handling and ephemeris extraction
  - observables generation (TOF/pseudorange)
  - PVT estimation
- Add quality control and outlier rejection hooks.
- Add systematic GNSS-SDR comparison harness.

### 4.2 Explicitly Out of Scope
- Galileo/E1, GPS L5, Galileo E5a processing.
- RTK/PPP/SBAS.
- Full carrier-phase ambiguity resolution.
- DMA, host-offload redesign, or SoC integration redesign.

## 5. Definition of Scaffold Removal

A block is considered scaffold-free only when the following are true:

- No synthetic constants are used as stand-ins for real measurements (except physical constants).
- No deterministic placeholders are used to emulate ephemeris or pseudorange.
- All reported outputs are derived from physically consistent inputs and equations.

Block-level criteria:

- Acquisition: PRN/code/Doppler result must come from actual correlation search, not signal-magnitude heuristic.
- Tracking: code/carrier lock must be discriminator-driven (DLL/FLL/PLL), not time-count transitions.
- Nav decode: word/subframe timing, parity, and field extraction must be true GPS NAV decode.
- Observables: pseudorange/rate must be based on receiver/satellite timing and code/carrier measurements.
- PVT: solution must be iterative least-squares (or better) over real satellite states and corrected pseudoranges.

## 6. Architecture for Phase 3 (Keep Boundaries Stable)

### 6.1 Top-Level Blocks (Unchanged Partition)
- `gps_l1_ca_phase2_top` remains integration point.
- `gps_l1_ca_chan_bank` remains channel container.
- `gps_l1_ca_nav_store` remains shared nav database.
- `gps_l1_ca_observables` remains epoch-align and measurement producer.
- `gps_l1_ca_pvt` remains solver stage.

### 6.2 Recommended Internal Refactoring (New/Updated Modules)

Acquisition:
- Upgrade `gps_l1_ca_acq` to true code-delay and Doppler search.
- Add coherent/non-coherent accumulation control.

Tracking:
- Upgrade `gps_l1_ca_track_chan`:
  - early/prompt/late discriminator path
  - DLL loop filter
  - FLL pull-in and PLL lock mode
  - lock indicators from metrics, not counters

Navigation decode:
- Split nav processing into explicit sub-blocks:
  - word sync / preamble detect
  - parity check
  - subframe collector
  - field extractor
- Feed decoded fields to `gps_l1_ca_nav_store`.

Ephemeris and satellite state:
- Add true broadcast ephemeris parameter storage per PRN.
- Add satellite position/clock propagation helper (from ephemeris + transmit time).

Observables:
- Generate true per-epoch observables:
  - pseudorange (meters)
  - pseudorange-rate (m/s)
  - carrier phase placeholder refined toward cycles
  - CN0 / lock-quality metadata
- Correct pseudorange with at least:
  - satellite clock correction
  - Earth rotation (Sagnac) first-order term
  - iono/tropo hook interfaces (optional in this phase, but interface-ready)

PVT:
- Keep iterative LS engine and upgrade to weighted LS.
- Add residual-based outlier rejection (single-pass robust gate).
- Add solution quality outputs (RMS residual, DOP approximations).

## 7. Data and Time Model Requirements

### 7.1 Required Time Quantities
- Receiver measurement epoch in GPS time.
- Per-channel transmit time estimate.
- TOW-consistent common-epoch alignment.

### 7.2 Required Satellite State Inputs per Solve
- Satellite ECEF position at transmit time.
- Satellite clock correction at transmit time.
- Measurement validity and quality weight.

### 7.3 Required Observable Outputs per Channel
- `rho_meas_m`
- `rho_rate_mps`
- PRN
- epoch tag
- validity and quality bits

## 8. Quantitative Alignment Targets vs GNSS-SDR

All comparisons are on the same input capture and aligned epochs.

### 8.1 Phase 3 Bring-Up Targets (Initial)
- First-fix presence: both systems produce a fix in the same run.
- Satellite overlap: >= 4 common PRNs during solve windows.
- Position sanity: converges to plausible Earth location and stable trajectory.

### 8.2 Phase 3 Exit Targets (Practical)
- Horizontal error vs GNSS-SDR reference: <= 25 m RMS.
- Vertical error vs GNSS-SDR reference: <= 40 m RMS.
- 3D error percentile (95th): <= 75 m.
- Epoch-to-epoch jitter (when static): <= 15 m RMS.
- Clock bias trend consistent with reference (same sign and similar slope).

### 8.3 Stretch Targets (If Schedule Allows)
- Horizontal <= 10 m RMS.
- Vertical <= 20 m RMS.
- Residual RMS reduced across major satellites without manual tuning per run.

### 8.4 Pass/Fail Table (Primary)

| Metric | Pass Threshold | Fail Threshold |
| --- | --- | --- |
| First-fix presence | FPGA and GNSS-SDR both produce at least 1 fix | Either side has no fix |
| Horizontal RMS error | `<= 25 m` | `> 25 m` |
| Vertical RMS error | `<= 40 m` | `> 40 m` |
| 3D error p95 | `<= 75 m` | `> 75 m` |
| Static jitter RMS | `<= 15 m` | `> 15 m` |
| Common-satellite count per solve epoch | `>= 4` | `< 4` |

### 8.5 Pass/Fail Table (Block De-Scaffolding)

| Block | Pass Criteria | Fail Criteria |
| --- | --- | --- |
| Acquisition (`gps_l1_ca_acq`) | PRN/code/Doppler from correlation search with configurable dwell | PRN selected from magnitude-only heuristic |
| Tracking (`gps_l1_ca_track_chan`) | DLL/FLL/PLL discriminator-driven lock and NCO updates | Counter-driven lock transitions without discriminator closure |
| NAV decode (`gps_l1_ca_nav`, `gps_l1_ca_nav_store`) | Valid preamble/word/parity/subframe decode and ephemeris field extraction | Bit-bucket or synthetic nav states |
| Observables (`gps_l1_ca_observables`) | Pseudorange/rate from timing-consistent TOF model + corrections | Synthetic range model unrelated to transmit/receive timing |
| PVT (`gps_l1_ca_pvt`) | Weighted iterative LS with residual checks and quality flags | Unweighted/arbitrary mapping without residual gating |

## 9. Verification Strategy

### 9.1 Golden Reference Flow
- Keep GNSS-SDR output as external truth source for this dataset.
- Export reference epochs/PRNs/LLH to machine-readable text or CSV.
- Compare FPGA logs to reference via scripted diff metrics.

### 9.2 Unit Verification (Mandatory)
- Acquisition peak detection and PRN/Doppler/code correctness.
- Tracking loop convergence and lock behavior.
- NAV parity and subframe decode correctness.
- Ephemeris field decode correctness.
- Satellite propagation math sanity.
- Pseudorange correction steps and residual checks.
- LS update step numerical stability.

### 9.3 Block Verification
- Multi-channel epoch alignment under lock transitions.
- Nav-data continuity through channel drops/reacquisition.
- Observables consistency and stale/outlier rejection.

### 9.4 System Verification
- `make lint-vhdl`
- `make sim-smoke`
- Add Phase 3 regression target comparing LLH timeseries against reference thresholds.

## 10. Implementation Roadmap

### 10.1 Step A: Instrumentation and Baseline Freeze
- Freeze current Phase 2 behavior for baseline metrics.
- Add structured trace outputs for:
  - per-channel code phase/Doppler/lock metrics
  - decoded nav words/subframes
  - per-satellite observables and residuals

Deliverable:
- baseline comparison report against GNSS-SDR.

### 10.2 Step B: Tracking and Acquisition De-Scaffolding
- Replace placeholder acquisition and lock logic with discriminator-driven loops.
- Validate stable multi-channel lock on replay file.

Deliverable:
- repeatable lock + improved PRN/Doppler/code estimates.

### 10.3 Step C: NAV Decode and Ephemeris Correctness
- Implement robust preamble/word/subframe/parity decode.
- Extract and store required ephemeris/clock terms.
- Add satellite position/clock propagation block.

Deliverable:
- per-PRN ephemeris-valid states and satellite state outputs from true decode.

### 10.4 Step D: True Observables
- Replace synthetic pseudorange model with timing-consistent measurement.
- Add correction pipeline and quality flags.

Deliverable:
- corrected pseudorange/rate with physically plausible residuals.

### 10.5 Step E: Weighted Iterative PVT
- Upgrade LS to weighted LS with residual gating.
- Add quality and residual outputs.
- Tune fixed-point scaling and numeric guards.

Deliverable:
- stable LLH with GNSS-SDR-close behavior and tracked error metrics.

### 10.6 Step F: Regression and Hardening
- Add deterministic nightly/CI-style regression scripts for phase metrics.
- Document known error budget and remaining deltas.

Deliverable:
- Phase 3 closure report with pass/fail against targets.

## 11. Register and Packet Evolution Guidance

- Keep existing packet types (`0x10`, `0x20`, `0x30`) but extend payload semantics.
- Add optional fields (or additional packet IDs) for:
  - residual RMS
  - DOP-like quality
  - per-satellite measurement quality
- Keep control/status map grouped by function:
  - acquisition
  - tracking
  - nav decode/ephemeris
  - observables
  - PVT

## 12. Numeric Strategy

- Use fixed-point in real-time loops where feasible.
- Allow sequential math engines for matrix operations at lower update rate.
- Keep one authoritative scaling table for all new math paths.
- Include overflow/underflow guards and saturation counters as status signals.

## 13. Risks and Mitigations

Risk: loop instability during real-data replay.
- Mitigation: staged loop bandwidth enable and lock hysteresis.

Risk: NAV decode fragility under marginal CN0.
- Mitigation: parity gating, subframe confidence, and hold-last-valid policy.

Risk: LS divergence from poor observables.
- Mitigation: weighted residual gating and solve conditioning checks.

Risk: fixed-point precision loss.
- Mitigation: reference-model cross-check + scaling sweeps + saturation telemetry.

## 14. Success Criteria

Phase 3 is complete when:

- scaffold logic is removed from acquisition/tracking/nav/observables/PVT paths
- all position outputs are derived from physically consistent measurement flow
- GNSS-SDR comparison meets Phase 3 exit targets
- design remains modular and ready for future features from `Outline.md`

## 15. How Phase 3 Enables Remaining Features

A completed Phase 3 provides a reusable backbone for:

- additional constellations and bands
- stronger observables and quality models
- advanced navigation filters
- eventual precision positioning modes

Critical rule for future phases:
- keep stable interfaces between channel bank, nav store, observables, and PVT so feature growth does not require architectural rewrites.
