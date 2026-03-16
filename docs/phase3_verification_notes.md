# Phase 3 Verification Notes

## Commands
- `make phase3-eval`
- `make phase3-gate`

`phase3-eval` always prints the metric table.
`phase3-gate` returns non-zero when one or more primary thresholds fail.

## Pass/Fail Tables

### GNSS-SDR Alignment

| Metric | Target | Current | Status |
| --- | --- | --- | --- |
| First-fix presence | both have >=1 fix | PASS (sim=17, ref=13) | PASS |
| Horizontal RMS error | <= 25 m | 17515301.418 m | FAIL |
| Vertical RMS error | <= 40 m | 1093.383 m | FAIL |
| 3D error p95 | <= 75 m | 17515310.170 m | FAIL |
| Static jitter RMS | <= 15 m | 0.000 m | PASS |
| Min observations used | >= 4 | 4 | PASS |

### Block De-Scaffolding

| Block | Pass Condition | Current Status |
| --- | --- | --- |
| Acquisition | Correlation-based PRN/code/doppler output | PASS (Step B complete) |
| Tracking | DLL/FLL/PLL discriminator-driven lock | PASS (Step B complete) |
| NAV decode | Word/parity/subframe decode + ephemeris fields | PASS (Step C implemented) |
| Observables | Timing-consistent pseudorange/rate + corrections | PASS (Step D implemented) |
| PVT | Weighted iterative LS + residual gating | PASS (Step E implemented) |

## Notes
- This file is the Phase 3 scoreboard companion to `Phase-3-Plans-and-Goal.md`.
- The authoritative numeric values are produced by `sim/phase3_compare.py` from simulation logs and `tb/txt/expected_output.txt`.
- Current values above were produced on 2026-03-16 with `make phase3-eval` after Step E updates.
- Step D now applies TOF + correction pipeline and Step E applies robust weighted LS, but Phase 3 metrics are still dominated by pre-ephemeris fallback geometry in short smoke runs.
