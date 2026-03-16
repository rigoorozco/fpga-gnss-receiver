# FFT-Based GPS L1 C/A Acquisition RTL Plan

## 1. Goal and Constraints

1. Replace the current time-domain acquisition inner loop in `gps_l1_ca_acq` with an FFT-based correlator.
2. Keep existing external interfaces stable so `gps_l1_ca_acq_sched`, `gps_l1_ca_phase2_top`, and channel handoff are unchanged.
3. Preserve runtime controls already exposed today:
   - PRN range
   - Doppler min/max/step and active Doppler-bin count
   - coherent ms and noncoherent dwells
   - detect threshold
   - code-bin count/step semantics
4. Maintain output contract: `{result_prn, result_code, result_dopp, result_metric}` plus `acq_done/acq_success/result_valid`.

## 2. Algorithm Choice

Use standard PCPS-style circular correlation per PRN and Doppler bin:

1. Capture 1 ms of complex samples (`N=2000`) and zero-pad to `NFFT=2048`.
2. Doppler wipeoff in time domain for one Doppler hypothesis.
3. FFT of wiped signal: `X[k]`.
4. Multiply by conjugate local-code spectrum for current PRN: `P[k] = X[k] * conj(C_prn[k])`.
5. IFFT to get correlation over all code phases: `r[n]`.
6. Evaluate configured code bins from `r[n]`, update coherent/noncoherent accumulators, and keep the PRN-local best peak.

Why this choice:

1. It gives all code phases from one IFFT (major speedup when search is wide).
2. It maps well to FPGA streaming/pipelined FFT cores.
3. It aligns with the existing Python FFT cross-check flow in `sim/verify_acq_fft_crosscheck.py`.

## 3. Proposed RTL Partition

Keep entity name and top-level ports for compatibility. Refactor internals into sub-blocks.

### 3.1 New Internal Blocks (inside `gps_l1_ca_acq` or as sub-entities)

1. `acq_capture_buf`
   - Dual-port RAM for 2048 complex samples (I/Q, signed).
   - Stores 2000 live samples and zero-pads 48 samples.

2. `acq_prn_code_fft`
   - Builds time-domain local PRN code vector for current PRN.
   - Generates/stores `conj(C_prn[k])` for `k=0..NFFT-1`.
   - Option A (recommended first): compute on PRN change and store in BRAM.
   - Option B (later optimization): ROM of precomputed spectra for PRN 1..32.

3. `acq_dopp_mixer_fft`
   - Replays capture buffer with carrier wipeoff for one Doppler bin.
   - Streams into FFT engine and stores `X[k]`.

4. `acq_spec_mult_ifft`
   - Complex multiply `X[k] * conj(C_prn[k])`.
   - IFFT to produce correlation sequence `r[n]`.

5. `acq_peak_eval`
   - Converts configured code bins (`code_bin_count`, `code_bin_step`) to correlation indices.
   - Metric per bin: `abs(I) + abs(Q)` (same style as current implementation).
   - Updates coherent/noncoherent sums and PRN/global best bins.

### 3.2 FFT Engine Strategy

1. Wrap FFT/IFFT behind an internal interface:
   - `fft_start`, `fft_dir`, streaming sample in/out, `fft_done`.
2. Provide two implementations:
   - Simulation/reference: pure-VHDL radix-2 iterative core.
   - Synthesis/perf: optional vendor FFT wrapper (guarded by generic).
3. Keep arithmetic fixed-point, deterministic rounding/saturation.

## 4. Acquisition State Machine (Replacement of Current Search Loop)

1. `IDLE`
   - Latch runtime config and build Doppler/bin tables (same behavior as current logic).

2. `PRN_PREP`
   - Build local PRN sequence for `prn_cur_r` and compute `conj(C_prn[k])`.

3. `CAPTURE_MS`
   - Capture 2000 input samples, zero-pad to 2048.

4. `DOPP_PROCESS`
   - For each active Doppler bin:
     - time-domain wipeoff replay
     - FFT
     - spectral multiply
     - IFFT
     - code-bin metric extraction for this Doppler

5. `COH_COMMIT`
   - After all Doppler bins for one coherent epoch, fold coherent accumulators.

6. `NONCOH_ADVANCE`
   - Repeat `CAPTURE_MS`..`COH_COMMIT` until `noncoh_dwells` complete.

7. `PRN_EVAL`
   - Compare PRN-best vs global best.
   - Advance PRN in configured range.

8. `FINALIZE`
   - Threshold compare, output result, pulse done.

## 5. Numeric Plan (Initial Fixed-Point Budget)

1. Input samples: `s16` (existing).
2. Mixer output: `s18` internal.
3. FFT/IFFT data path: `s24` complex (with stage scaling schedule).
4. Spectrum multiply: `s24 x s24 -> s48`, then scaled back to `s24` for IFFT.
5. Correlation outputs: `s24` or `s28` depending on FFT scaling.
6. Metric accumulator: keep `u32` saturated add to match existing result path.

Notes:

1. Use explicit per-stage right-shifts in FFT to avoid overflow; record total shift exponent.
2. Keep one normalization convention for both FFT and IFFT and reflect it in threshold tuning.

## 6. Preserving Existing Config Semantics

1. `doppler_min/max/step` + `doppler_bin_count_i` stay unchanged.
2. `code_bin_count_i` + `code_bin_step_i` stay unchanged:
   - still evaluate only those bins, but now sampled from the full IFFT correlation vector.
3. `coh_ms_i`, `noncoh_dwells_i` stay unchanged.
4. `detect_thresh` stays unchanged (but expected absolute values may shift; re-tune defaults if needed).

Optional extension after parity is proven:

1. Add a mode bit for full 1023-code-phase peak search (ignoring code-bin decimation).
2. Add optional peak-ratio output/status for stronger false-lock rejection.

## 7. Throughput and Memory Estimate (2048-point design)

Per acquisition core instance:

1. Capture RAM: `2048 * (16+16)` bits.
2. PRN spectrum RAM: `2048 * (Wre+Wim)` bits (about 96 kbits at 24+24).
3. Signal spectrum RAM: same order.
4. Correlation scan buffer: optional if evaluating bins on IFFT stream (preferred to reduce BRAM).

Latency per PRN is approximately:

1. `coh_ms * noncoh_dwells * doppler_bins * (FFT + IFFT + replay overhead)`
2. plus one-time PRN spectrum generation cost.

Even with serial reuse of one FFT/IFFT engine, runtime should scale much better than per-code-bin time-domain correlation for wide searches.

## 8. Integration Plan (Low-Risk Steps)

1. Step A: Internal scaffolding
   - Add FFT wrapper modules and compile under a new generic (default old path).

2. Step B: Single-PRN parity
   - Implement FFT path for one PRN, one dwell, one coherent ms.
   - Match `{code,dopp,metric}` against current TD core in `gps_l1_ca_acq_tb` reduced cases.

3. Step C: Full runtime parity
   - Enable PRN loops, coherent/noncoherent accumulation, and threshold finalize.
   - Ensure scheduler/channel-bank tests still pass unmodified.

4. Step D: Performance mode default
   - Flip default implementation to FFT after parity and threshold re-baselining.

5. Step E: Cleanup
   - Remove dead TD-only logic or keep as debug fallback generic.

## 9. Verification Plan

1. Keep existing `gps_l1_ca_acq_tb` and compare pass/fail behavior on all runs.
2. Add direct equivalence check in TB logs:
   - TD mode vs FFT mode for same seeds/config.
3. Reuse Python model checks:
   - `sim/verify_acq_fft_crosscheck.py`
   - `sim/validate_acq_fullspace_prn*.py`
4. Re-run Phase 2/3 regressions:
   - `gps_l1_ca_acq_sched_tb`
   - `gps_l1_ca_chan_bank_tb`
   - `gps_l1_ca_phase2_tb`
   - `make phase3-eval`
5. Add corner tests:
   - Doppler range inversion (`min > max`)
   - bin-count clipping
   - very low/high threshold behavior
   - no-signal input and tie-break reproducibility.

## 10. Recommended First RTL Drop

For quickest value with manageable risk:

1. Implement FFT mode behind generic `G_ACQ_IMPL_FFT : boolean := false`.
2. Keep all registers/ports unchanged.
3. Preserve code-bin decimated search semantics first.
4. Defer full 1023-bin peak search and peak-ratio detector to a second patch.

This gives a controlled migration path: same integration contract, measurable acceleration potential, and direct comparability to current acquisition behavior.
