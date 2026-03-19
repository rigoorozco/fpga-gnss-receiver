#!/usr/bin/env python3
"""Cross-check gps_l1_ca acquisition model against an FFT-based correlator.

This script compares two independent estimators over the same stimulus/config:
1) Time-domain reference model from validate_acq_fullspace_prn1.py
2) FFT-domain correlator (spectral dot-product at lag 0 per code bin)

It is intended to sanity-check result_code/result_dopp for Run4 PRN1-style setup.
"""

from __future__ import annotations

import argparse
import cmath
import math
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

import validate_acq_fullspace_prn1 as ref


def fft_inplace(a: List[complex], inverse: bool = False) -> None:
    """Iterative radix-2 FFT in place."""
    n = len(a)
    if n == 0 or (n & (n - 1)) != 0:
        raise ValueError("FFT size must be a power of two")

    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit
        if i < j:
            a[i], a[j] = a[j], a[i]

    length = 2
    sign = 1.0 if inverse else -1.0
    while length <= n:
        half = length // 2
        w_len = cmath.exp(sign * 2j * math.pi / length)
        for i in range(0, n, length):
            w = 1.0 + 0.0j
            for k in range(i, i + half):
                u = a[k]
                v = a[k + half] * w
                a[k] = u + v
                a[k + half] = u - v
                w *= w_len
        length <<= 1

    if inverse:
        inv_n = 1.0 / n
        for i in range(n):
            a[i] *= inv_n


def fft(vec: Sequence[complex], n_fft: int) -> List[complex]:
    out = list(vec)[:n_fft]
    if len(out) < n_fft:
        out.extend([0.0 + 0.0j] * (n_fft - len(out)))
    fft_inplace(out, inverse=False)
    return out


def trunc_div_toward_zero(num: int, den: int) -> int:
    return int(num / den)


def build_code_sign_vector(prn_seq: Sequence[int], code_start: int, n_samp: int) -> List[complex]:
    chip_idx = code_start
    code_nco = (code_start << 21) & 0xFFFFFFFF

    out: List[complex] = []
    for _ in range(n_samp):
        sign = -1.0 if prn_seq[chip_idx] == 1 else 1.0
        out.append(sign + 0.0j)

        next_code_nco = (code_nco + ref.C_CODE_NCO_FCW) & 0xFFFFFFFF
        if next_code_nco < code_nco:
            chip_idx = 0 if chip_idx == 1022 else chip_idx + 1
        code_nco = next_code_nco

    return out


def build_mixed_signal(cap_i: Sequence[int], cap_q: Sequence[int], dopp_hz: int) -> List[complex]:
    carr_phase = 0
    carr_fcw = ref.wrap_s32(dopp_hz * ref.C_CARR_FCW_PER_HZ)

    cos_lut = [int(round(math.cos(2.0 * math.pi * k / 1024.0) * 32767.0)) for k in range(1024)]
    sin_lut = [int(round(math.sin(2.0 * math.pi * k / 1024.0) * 32767.0)) for k in range(1024)]

    z: List[complex] = []
    for si, sq in zip(cap_i, cap_q):
        phase_idx = ((carr_phase & 0xFFFFFFFF) >> 22) & 0x3FF
        lo_i = cos_lut[phase_idx]
        lo_q = -sin_lut[phase_idx]

        mix_i = trunc_div_toward_zero(si * lo_i - sq * lo_q, 32768)
        mix_q = trunc_div_toward_zero(si * lo_q + sq * lo_i, 32768)
        z.append(complex(float(mix_i), float(mix_q)))

        carr_phase = ref.wrap_s32(carr_phase + carr_fcw)

    return z


def build_bin_axes_and_metrics(
    cap_i: Sequence[int],
    cap_q: Sequence[int],
    prn: int,
    bins: Sequence[Tuple[int, int]],
    code_bin_count: int,
    dopp_bin_count: int,
) -> Tuple[int, int, int]:
    n_samp = len(cap_i)
    n_fft = 1
    while n_fft < n_samp:
        n_fft <<= 1

    prn_seq = ref.build_prn_sequence(prn)
    code_axis = [bins[c][0] for c in range(code_bin_count)]
    dopp_axis = [bins[d * code_bin_count][1] for d in range(dopp_bin_count)]

    # Precompute FFT(code_sign) for each code bin (independent of doppler).
    code_fft_conj: List[List[complex]] = []
    for code_start in code_axis:
        sign_vec = build_code_sign_vector(prn_seq, code_start, n_samp)
        y = fft(sign_vec, n_fft)
        code_fft_conj.append([v.conjugate() for v in y])

    best_metric = -1
    best_code = code_axis[0]
    best_dopp = dopp_axis[0]

    for dopp_hz in dopp_axis:
        z = build_mixed_signal(cap_i, cap_q, dopp_hz)
        x = fft(z, n_fft)

        for c_idx, code_start in enumerate(code_axis):
            yc = code_fft_conj[c_idx]
            # r[0] of circular correlation = (1/N) * sum_k X[k]*conj(Y[k])
            acc = 0.0 + 0.0j
            for k in range(n_fft):
                acc += x[k] * yc[k]
            corr0 = acc / n_fft

            corr_i = int(round(corr0.real))
            corr_q = int(round(corr0.imag))
            metric = abs(corr_i) + abs(corr_q)

            # Match RTL tie-breaker style (>=)
            if metric >= best_metric:
                best_metric = metric
                best_code = code_start
                best_dopp = dopp_hz

    return best_code, best_dopp, best_metric


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--input-file",
        default="2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN_2msps.dat",
    )
    p.add_argument("--file-sample-rate", type=int, default=2_000_000)
    p.add_argument("--dut-sample-rate", type=int, default=2_000_000)
    p.add_argument("--samples-per-ms", type=int, default=ref.C_SAMPLES_PER_MS)

    p.add_argument("--prn", type=int, default=1)
    p.add_argument("--doppler-min", type=int, default=-2000)
    p.add_argument("--doppler-max", type=int, default=2000)
    p.add_argument("--doppler-step", type=int, default=250)
    p.add_argument("--code-bins", type=int, default=64)
    p.add_argument("--code-step", type=int, default=16)
    p.add_argument("--doppler-bins", type=int, default=17)
    p.add_argument("--metric-tol", type=int, default=2)
    return p.parse_args()


def main() -> int:
    args = parse_args()

    cap_i, cap_q = ref.read_decimated_iq(
        path=Path(args.input_file),
        file_fs=args.file_sample_rate,
        dut_fs=args.dut_sample_rate,
        samples_needed=args.samples_per_ms,
    )

    bins, code_bin_count, dopp_bin_count = ref.build_search_bins(
        doppler_min=args.doppler_min,
        doppler_max=args.doppler_max,
        doppler_step=args.doppler_step,
        doppler_bin_count=args.doppler_bins,
        code_bin_count=args.code_bins,
        code_bin_step=args.code_step,
    )

    ref_ns = argparse.Namespace(**vars(args), progress=False)
    td_result = ref.run_reference(ref_ns)

    fft_code, fft_dopp, fft_metric = build_bin_axes_and_metrics(
        cap_i=cap_i,
        cap_q=cap_q,
        prn=args.prn,
        bins=bins,
        code_bin_count=code_bin_count,
        dopp_bin_count=dopp_bin_count,
    )

    print(f"time_domain: code={td_result.best_code} dopp={td_result.best_dopp} metric={td_result.best_metric}")
    print(f"fft_based  : code={fft_code} dopp={fft_dopp} metric={fft_metric}")

    ok = True
    if fft_code != td_result.best_code:
        print("ERROR: result_code mismatch between time-domain and FFT-based models", file=sys.stderr)
        ok = False
    if fft_dopp != td_result.best_dopp:
        print("ERROR: result_dopp mismatch between time-domain and FFT-based models", file=sys.stderr)
        ok = False
    if abs(fft_metric - td_result.best_metric) > args.metric_tol:
        print(
            f"ERROR: metric mismatch exceeds tolerance {args.metric_tol}",
            file=sys.stderr,
        )
        ok = False

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
