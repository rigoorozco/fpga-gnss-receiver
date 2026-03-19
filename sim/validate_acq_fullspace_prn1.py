#!/usr/bin/env python3
"""Reference model for gps_l1_ca_acq_tb Run4 (PRN 1 full code-Doppler search).

The script reads the same IQ stimulus file used by `gps_l1_ca_acq_tb`, applies the
same decimation, and emulates the acquisition bin search for one PRN using the
same key constants/updates as rtl/vhdl/gps_l1_ca_acq.vhd.

It reports best {result_code, result_dopp, metric} and can optionally validate
against expected values.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Tuple

C_CODE_NCO_FCW = 0x82EF9DB2
C_CARR_FCW_PER_HZ = 2147
C_SAMPLES_PER_MS = 2000
C_MAX_CODE_BINS = 64
C_MAX_DOPP_BINS = 33
C_DEF_CODE_BINS = 16
C_DEF_CODE_STEP = 64
C_DEF_DOPP_BINS = 9


@dataclass
class RankedBin:
    code: int
    dopp: int
    metric: int
    search_idx: int


@dataclass
class AcqSearchResult:
    best_code: int
    best_dopp: int
    best_metric: int
    top_bins: List[RankedBin]
    avg_metric: float
    code_axis: List[int]
    dopp_axis: List[int]
    metric_grid: List[List[int]]


G2_TAP_A = {
    1: 2,
    2: 3,
    3: 4,
    4: 5,
    5: 1,
    6: 2,
    7: 1,
    8: 2,
    9: 3,
    10: 2,
    11: 3,
    12: 5,
    13: 6,
    14: 7,
    15: 8,
    16: 9,
    17: 1,
    18: 2,
    19: 3,
    20: 4,
    21: 5,
    22: 6,
    23: 1,
    24: 4,
    25: 5,
    26: 6,
    27: 7,
    28: 8,
    29: 1,
    30: 2,
    31: 3,
    32: 4,
}

G2_TAP_B = {
    1: 6,
    2: 7,
    3: 8,
    4: 9,
    5: 9,
    6: 10,
    7: 8,
    8: 9,
    9: 10,
    10: 3,
    11: 4,
    12: 6,
    13: 7,
    14: 8,
    15: 9,
    16: 10,
    17: 4,
    18: 5,
    19: 6,
    20: 7,
    21: 8,
    22: 9,
    23: 3,
    24: 6,
    25: 7,
    26: 8,
    27: 9,
    28: 10,
    29: 6,
    30: 7,
    31: 8,
    32: 9,
}


def trunc_div_toward_zero(num: int, den: int) -> int:
    return int(num / den)


def clamp_s16(x: int) -> int:
    return max(-32768, min(32767, x))


def wrap_s32(x: int) -> int:
    x &= 0xFFFFFFFF
    if x & 0x80000000:
        return x - 0x100000000
    return x


def sat_u32(x: int) -> int:
    return min(0xFFFFFFFF, max(0, x))


def sat_add_u32(a: int, b: int) -> int:
    return sat_u32(a + b)


def abs_s48_sat_u32(x: int) -> int:
    ax = abs(x)
    return sat_u32(ax)


def build_prn_sequence(prn: int) -> List[int]:
    ta = G2_TAP_A.get(prn, 4)
    tb = G2_TAP_B.get(prn, 9)

    g1 = [1] * 10  # index matches bit index (0..9)
    g2 = [1] * 10
    seq: List[int] = [0] * 1023

    for chip in range(1023):
        g1_out = g1[9]
        g2_out = g2[10 - ta] ^ g2[10 - tb]
        seq[chip] = g1_out ^ g2_out

        fb1 = g1[2] ^ g1[9]
        fb2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]

        for i in range(9, 0, -1):
            g1[i] = g1[i - 1]
            g2[i] = g2[i - 1]
        g1[0] = fb1
        g2[0] = fb2

    return seq


def build_anti_alias_fir(decim: int) -> List[float]:
    if decim <= 1:
        return [1.0]

    # Use an odd-length Hamming-windowed sinc. Cutoff scales with decimation ratio.
    num_taps = max(17, 16 * decim + 1)
    if (num_taps % 2) == 0:
        num_taps += 1
    half = num_taps // 2
    cutoff_cycles_per_sample = 0.45 / float(decim)

    coeffs: List[float] = []
    for n in range(num_taps):
        x = n - half
        if x == 0:
            sinc = 2.0 * cutoff_cycles_per_sample
        else:
            sinc = math.sin(2.0 * math.pi * cutoff_cycles_per_sample * x) / (math.pi * x)
        window = 0.54 - 0.46 * math.cos(2.0 * math.pi * n / float(num_taps - 1))
        coeffs.append(sinc * window)

    coeff_sum = sum(coeffs)
    if coeff_sum == 0.0:
        return [1.0]
    return [c / coeff_sum for c in coeffs]


def read_decimated_iq(
    path: Path,
    file_fs: int,
    dut_fs: int,
    start_sample: int,
    samples_needed: int,
    anti_alias: bool = True,
    require_full: bool = True,
) -> Tuple[List[int], List[int]]:
    if file_fs % dut_fs != 0:
        raise ValueError("file_fs must be an integer multiple of dut_fs")
    if start_sample < 0:
        raise ValueError("start_sample must be >= 0")
    if samples_needed <= 0:
        raise ValueError("samples_needed must be > 0")
    decim = file_fs // dut_fs

    out_i: List[int] = []
    out_q: List[int] = []
    in_idx = start_sample * decim

    if (not anti_alias) or decim <= 1:
        with path.open("rb") as f:
            f.seek(in_idx * 4)
            while len(out_i) < samples_needed:
                b = f.read(4)
                if len(b) < 4:
                    if not require_full:
                        break
                    raise RuntimeError(
                        "EOF while reading stimulus. "
                        f"Need {samples_needed} decimated samples from offset {start_sample}, got {len(out_i)}"
                    )
                i_s, q_s = struct.unpack("<hh", b)
                if (in_idx % decim) == 0:
                    out_i.append(i_s)
                    out_q.append(q_s)
                in_idx += 1

        return out_i, out_q

    total_samples = path.stat().st_size // 4
    last_center = in_idx + (samples_needed - 1) * decim
    if last_center >= total_samples:
        if not require_full:
            samples_needed = max(0, (total_samples - in_idx + decim - 1) // decim)
            last_center = in_idx + (samples_needed - 1) * decim if samples_needed > 0 else in_idx
        else:
            got = max(0, (total_samples - in_idx + decim - 1) // decim)
            raise RuntimeError(
                "EOF while reading stimulus. "
                f"Need {samples_needed} decimated samples from offset {start_sample}, got {got}"
            )
    if samples_needed == 0:
        return out_i, out_q

    fir = build_anti_alias_fir(decim)
    half = len(fir) // 2

    read_start = max(0, in_idx - half)
    read_end = min(total_samples - 1, last_center + half)
    read_count = read_end - read_start + 1

    with path.open("rb") as f:
        f.seek(read_start * 4)
        raw = f.read(read_count * 4)
    if len(raw) < read_count * 4:
        if require_full:
            raise RuntimeError("EOF while reading stimulus for anti-alias filtering")
        read_count = len(raw) // 4
        raw = raw[: read_count * 4]
        read_end = read_start + read_count - 1

    chunk_i: List[int] = []
    chunk_q: List[int] = []
    for i_s, q_s in struct.iter_unpack("<hh", raw):
        chunk_i.append(i_s)
        chunk_q.append(q_s)

    for k in range(samples_needed):
        center = in_idx + k * decim
        acc_i = 0.0
        acc_q = 0.0
        for tap_idx, tap in enumerate(fir):
            src_idx = center + tap_idx - half
            if read_start <= src_idx <= read_end:
                local = src_idx - read_start
                acc_i += tap * chunk_i[local]
                acc_q += tap * chunk_q[local]
        out_i.append(clamp_s16(int(round(acc_i))))
        out_q.append(clamp_s16(int(round(acc_q))))

    return out_i, out_q


def build_search_bins(
    doppler_min: int,
    doppler_max: int,
    doppler_step: int,
    doppler_bin_count: int,
    code_bin_count: int,
    code_bin_step: int,
) -> Tuple[List[Tuple[int, int]], int, int]:
    code_bins_cfg = code_bin_count
    if code_bins_cfg <= 0:
        code_bins_cfg = C_DEF_CODE_BINS
    code_bins_cfg = min(C_MAX_CODE_BINS, code_bins_cfg)

    code_step_cfg = code_bin_step
    if code_step_cfg <= 0:
        code_step_cfg = C_DEF_CODE_STEP
    code_step_cfg = min(1022, code_step_cfg)

    step_i = abs(doppler_step)
    if step_i < 1:
        step_i = 1

    dopp_min_i = doppler_min
    dopp_max_i = doppler_max
    if dopp_max_i < dopp_min_i:
        dopp_min_i, dopp_max_i = dopp_max_i, dopp_min_i

    dopp_span_i = dopp_max_i - dopp_min_i
    full_dopp_bins_i = (dopp_span_i // step_i) + 1
    full_dopp_bins_i = max(1, full_dopp_bins_i)

    dopp_bins_cfg_i = doppler_bin_count
    if dopp_bins_cfg_i <= 0:
        active_dopp_i = C_DEF_DOPP_BINS
    else:
        active_dopp_i = dopp_bins_cfg_i
    active_dopp_i = min(C_MAX_DOPP_BINS, active_dopp_i)
    active_dopp_i = min(full_dopp_bins_i, active_dopp_i)
    active_dopp_i = max(1, active_dopp_i)

    start_dopp_idx_i = (full_dopp_bins_i - active_dopp_i) // 2

    bins: List[Tuple[int, int]] = []
    for d in range(active_dopp_i):
        d_i = start_dopp_idx_i + d
        dopp_hz_i = clamp_s16(dopp_min_i + d_i * step_i)
        for c in range(code_bins_cfg):
            code_i = (c * code_step_cfg) % 1023
            bins.append((code_i, dopp_hz_i))

    return bins, code_bins_cfg, active_dopp_i


def evaluate_bin(
    cap_i: List[int],
    cap_q: List[int],
    prn_seq: List[int],
    code_start: int,
    dopp_hz: int,
    cos_lut: List[int],
    sin_lut: List[int],
) -> int:
    chip_idx = code_start
    code_nco = (code_start << 21) & 0xFFFFFFFF
    carr_phase = 0
    carr_fcw = wrap_s32(dopp_hz * C_CARR_FCW_PER_HZ)

    corr_i = 0
    corr_q = 0

    for si, sq in zip(cap_i, cap_q):
        phase_idx = ((carr_phase & 0xFFFFFFFF) >> 22) & 0x3FF
        lo_i = cos_lut[phase_idx]
        lo_q = -sin_lut[phase_idx]

        mix_i = trunc_div_toward_zero(si * lo_i - sq * lo_q, 32768)
        mix_q = trunc_div_toward_zero(si * lo_q + sq * lo_i, 32768)

        if prn_seq[chip_idx] == 1:
            corr_i -= mix_i
            corr_q -= mix_q
        else:
            corr_i += mix_i
            corr_q += mix_q

        carr_phase = wrap_s32(carr_phase + carr_fcw)

        next_code_nco = (code_nco + C_CODE_NCO_FCW) & 0xFFFFFFFF
        if next_code_nco < code_nco:
            chip_idx = 0 if chip_idx == 1022 else chip_idx + 1
        code_nco = next_code_nco

    coh_metric = sat_add_u32(abs_s48_sat_u32(corr_i), abs_s48_sat_u32(corr_q))
    return coh_metric


def run_reference(args: argparse.Namespace) -> AcqSearchResult:
    if args.time_offset < 0.0:
        raise ValueError("time_offset must be >= 0 ms")
    if args.window_size <= 0:
        raise ValueError("window_size must be > 0 samples")

    # Convert ms offset to decimated-sample offset at DUT sample rate.
    start_sample = int(math.floor((args.time_offset * args.dut_sample_rate / 1000.0) + 1e-12))

    cap_i, cap_q = read_decimated_iq(
        path=Path(args.input_file),
        file_fs=args.file_sample_rate,
        dut_fs=args.dut_sample_rate,
        start_sample=start_sample,
        samples_needed=args.window_size,
        anti_alias=args.anti_alias,
    )

    bins, code_bin_count_eff, dopp_bin_count_eff = build_search_bins(
        doppler_min=args.doppler_min,
        doppler_max=args.doppler_max,
        doppler_step=args.doppler_step,
        doppler_bin_count=args.doppler_bins,
        code_bin_count=args.code_bins,
        code_bin_step=args.code_step,
    )

    prn_seq = build_prn_sequence(args.prn)

    cos_lut = [int(round(math.cos(2.0 * math.pi * k / 1024.0) * 32767.0)) for k in range(1024)]
    sin_lut = [int(round(math.sin(2.0 * math.pi * k / 1024.0) * 32767.0)) for k in range(1024)]

    all_metrics: List[int] = []

    for idx, (code_bin, dopp_bin) in enumerate(bins):
        metric = evaluate_bin(cap_i, cap_q, prn_seq, code_bin, dopp_bin, cos_lut, sin_lut)
        all_metrics.append(metric)

        if args.progress and ((idx + 1) % 64 == 0 or (idx + 1) == len(bins)):
            print(f"processed {idx + 1}/{len(bins)} bins", file=sys.stderr)

    ranked = sorted(
        (
            RankedBin(code=bins[idx][0], dopp=bins[idx][1], metric=metric, search_idx=idx)
            for idx, metric in enumerate(all_metrics)
        ),
        key=lambda x: (x.metric, x.search_idx),
        reverse=True,
    )
    top_bins = ranked[:3]
    best = top_bins[0]
    avg_metric = sum(all_metrics) / float(len(all_metrics))

    code_axis = [bins[c][0] for c in range(code_bin_count_eff)]
    dopp_axis = [bins[d * code_bin_count_eff][1] for d in range(dopp_bin_count_eff)]
    metric_grid = [
        all_metrics[d * code_bin_count_eff : (d + 1) * code_bin_count_eff]
        for d in range(dopp_bin_count_eff)
    ]

    return AcqSearchResult(
        best_code=best.code,
        best_dopp=best.dopp,
        best_metric=best.metric,
        top_bins=top_bins,
        avg_metric=avg_metric,
        code_axis=code_axis,
        dopp_axis=dopp_axis,
        metric_grid=metric_grid,
    )


def import_plot_libs(show_plots: bool) -> Tuple[Any, Any]:
    try:
        import numpy as np
        import matplotlib

        if not show_plots:
            matplotlib.use("Agg")

        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Plotting requires numpy and matplotlib. Install them to use --plot-3d."
        ) from exc
    return np, plt


def plot_code_doppler_surface(
    np: Any,
    plt: Any,
    result: AcqSearchResult,
    out_path: Path,
    show_plots: bool,
) -> None:
    x_codes = np.array(result.code_axis, dtype=np.float64)
    y_dopps = np.array(result.dopp_axis, dtype=np.float64)
    z_metric = np.array(result.metric_grid, dtype=np.float64)

    x_mesh, y_mesh = np.meshgrid(x_codes, y_dopps)

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    surface = ax.plot_surface(x_mesh, y_mesh, z_metric, cmap="viridis", linewidth=0, antialiased=True)
    marker_cfg = [
        ("Best Bin", "red", 45),
        ("2nd Best", "orange", 40),
        ("3rd Best", "gold", 35),
    ]
    for rank_idx, bin_rank in enumerate(result.top_bins):
        label, color, size = marker_cfg[rank_idx]
        ax.scatter(
            [bin_rank.code],
            [bin_rank.dopp],
            [bin_rank.metric],
            color=color,
            s=size,
            label=label,
        )
    ax.set_title("Code-Doppler Search Space Metric")
    ax.set_xlabel("Code Bin (chips)")
    ax.set_ylabel("Doppler Bin (Hz)")
    ax.set_zlabel("Metric")
    ax.legend(loc="upper right")
    fig.colorbar(surface, shrink=0.65, aspect=16, pad=0.08, label="Metric")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    print(f"saved_3d_plot={out_path}")
    if show_plots:
        plt.show()
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--input-file",
        default="2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.dat",
        help="Path to stimulus file (I16,Q16 little-endian interleaved)",
    )
    p.add_argument("--file-sample-rate", type=int, default=4_000_000)
    p.add_argument("--dut-sample-rate", type=int, default=2_000_000)
    p.add_argument(
        "--time-offset",
        type=float,
        default=0.0,
        help="Start of the acquisition window in ms from beginning of input file.",
    )
    p.add_argument(
        "--window-size",
        type=int,
        default=C_SAMPLES_PER_MS,
        help="Number of decimated input samples to use for acquisition.",
    )
    p.add_argument("--samples-per-ms", dest="window_size", type=int, help=argparse.SUPPRESS)
    p.add_argument(
        "--anti-alias",
        dest="anti_alias",
        action="store_true",
        default=True,
        help="Apply FIR anti-alias filtering before decimation (enabled by default).",
    )
    p.add_argument(
        "--no-anti-alias",
        dest="anti_alias",
        action="store_false",
        help="Disable anti-alias filtering and use sample-drop decimation.",
    )

    p.add_argument("--prn", type=int, default=1)
    p.add_argument("--doppler-min", type=int, default=-2000)
    p.add_argument("--doppler-max", type=int, default=2000)
    p.add_argument("--doppler-step", type=int, default=250)

    p.add_argument("--code-bins", type=int, default=64)
    p.add_argument("--code-step", type=int, default=16)
    p.add_argument(
        "--doppler-bins",
        type=int,
        default=17,
        help="Set to 17 to match the full-span Run4 PRN1 case",
    )

    p.add_argument("--expected-result-code", type=int, default=None)
    p.add_argument("--expected-result-dopp", type=int, default=None)
    p.add_argument("--require-nonzero", action="store_true", default=True)
    p.add_argument("--no-require-nonzero", dest="require_nonzero", action="store_false")
    p.add_argument("--progress", action="store_true")
    p.add_argument("--plot-3d", action="store_true", help="Save 3D code-doppler metric surface plot.")
    p.add_argument("--plots-dir", default="sim/plots", help="Directory to write plot PNGs.")
    p.add_argument("--show-plots", action="store_true", help="Display plots interactively.")

    return p.parse_args()


def main() -> int:
    args = parse_args()

    try:
        result = run_reference(args)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"result_code={result.best_code}")
    print(f"result_dopp={result.best_dopp}")
    print(f"result_metric={result.best_metric}")
    if len(result.top_bins) >= 2:
        print(f"result2_code={result.top_bins[1].code}")
        print(f"result2_dopp={result.top_bins[1].dopp}")
        print(f"result2_metric={result.top_bins[1].metric}")
    if len(result.top_bins) >= 3:
        print(f"result3_code={result.top_bins[2].code}")
        print(f"result3_dopp={result.top_bins[2].dopp}")
        print(f"result3_metric={result.top_bins[2].metric}")
    print(f"avg_metric_window={result.avg_metric:.3f}")

    failed = False

    if args.expected_result_code is not None and result.best_code != args.expected_result_code:
        print(
            f"ERROR: expected result_code={args.expected_result_code}, got {result.best_code}",
            file=sys.stderr,
        )
        failed = True

    if args.expected_result_dopp is not None and result.best_dopp != args.expected_result_dopp:
        print(
            f"ERROR: expected result_dopp={args.expected_result_dopp}, got {result.best_dopp}",
            file=sys.stderr,
        )
        failed = True

    if args.require_nonzero and result.best_code == 0 and result.best_dopp == 0:
        print(
            "ERROR: expected non-zero code or Doppler estimate, but both are zero",
            file=sys.stderr,
        )
        failed = True

    if args.plot_3d:
        try:
            np, plt = import_plot_libs(args.show_plots)
        except RuntimeError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        plots_dir = Path(args.plots_dir)
        plots_dir.mkdir(parents=True, exist_ok=True)

        plot_code_doppler_surface(
            np=np,
            plt=plt,
            result=result,
            out_path=plots_dir / "code_doppler_metric_surface.png",
            show_plots=args.show_plots,
        )

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
