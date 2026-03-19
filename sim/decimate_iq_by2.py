#!/usr/bin/env python3
"""
Decimate interleaved int16 IQ samples by 2 with an anti-alias FIR filter.

Input format:
  [I0, Q0, I1, Q1, ...] as signed 16-bit little-endian integers
  Sample rate: 4 MSPS (complex)

Output format:
  Same interleaved int16 IQ format at 2 MSPS (complex), after LPF + /2 decimation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def design_lpf(num_taps: int, cutoff_norm: float) -> np.ndarray:
    """
    Windowed-sinc low-pass FIR with unity DC gain.

    cutoff_norm is normalized to input Nyquist:
      1.0 -> input Nyquist (Fs/2)
      0.5 -> Fs/4
    For decimation by 2, use cutoff_norm < 0.5.
    """
    if num_taps < 3:
        raise ValueError("num_taps must be >= 3")
    if num_taps % 2 == 0:
        raise ValueError("num_taps must be odd")
    if not (0.0 < cutoff_norm < 0.5):
        raise ValueError("cutoff_norm must be between 0 and 0.5 for decimate-by-2")

    n = np.arange(num_taps, dtype=np.float64) - (num_taps - 1) / 2.0
    h = cutoff_norm * np.sinc(cutoff_norm * n)
    h *= np.hamming(num_taps)
    h /= np.sum(h)
    return h.astype(np.float64, copy=False)


def interleaved_iq_to_complex(raw_iq: np.ndarray) -> np.ndarray:
    """Convert int16 interleaved IQ -> complex64."""
    i = raw_iq[0::2].astype(np.float32, copy=False)
    q = raw_iq[1::2].astype(np.float32, copy=False)
    return i + 1j * q


def complex_to_interleaved_iq(x: np.ndarray) -> np.ndarray:
    """Convert complex -> clipped int16 interleaved IQ."""
    out = np.empty(x.size * 2, dtype=np.int16)
    out[0::2] = np.clip(np.rint(np.real(x)), -32768, 32767).astype(np.int16)
    out[1::2] = np.clip(np.rint(np.imag(x)), -32768, 32767).astype(np.int16)
    return out


def decimate_file_by2(
    in_path: Path,
    out_path: Path,
    num_taps: int,
    cutoff_norm: float,
    chunk_iq: int,
) -> None:
    h = design_lpf(num_taps=num_taps, cutoff_norm=cutoff_norm)

    state = np.zeros(num_taps - 1, dtype=np.complex64)
    phase = 0  # keep y[phase::2] per chunk so global decimation stays aligned

    total_in_iq = 0
    total_out_iq = 0

    with in_path.open("rb") as fin, out_path.open("wb") as fout:
        while True:
            raw = np.fromfile(fin, dtype="<i2", count=2 * chunk_iq)
            if raw.size == 0:
                break

            if raw.size % 2 != 0:
                raw = raw[:-1]
                if raw.size == 0:
                    break

            x = interleaved_iq_to_complex(raw)
            total_in_iq += x.size

            x_ext = np.concatenate((state, x))
            y = np.convolve(x_ext, h, mode="valid")
            state = x_ext[-(num_taps - 1) :].astype(np.complex64, copy=False)

            y_dec = y[phase::2]
            phase = (phase + y.size) % 2

            out_raw = complex_to_interleaved_iq(y_dec)
            out_raw.astype("<i2", copy=False).tofile(fout)
            total_out_iq += y_dec.size

    print(f"Input IQ samples:  {total_in_iq}")
    print(f"Output IQ samples: {total_out_iq}")
    print("Output sample rate: 2,000,000 SPS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decimate interleaved int16 IQ file from 4 MSPS to 2 MSPS with anti-alias FIR."
    )
    parser.add_argument("input", type=Path, help="Input .iq file (int16 interleaved I/Q)")
    parser.add_argument("output", type=Path, help="Output .iq file (int16 interleaved I/Q)")
    parser.add_argument(
        "--num-taps",
        type=int,
        default=127,
        help="Odd FIR length (default: 127)",
    )
    parser.add_argument(
        "--cutoff-norm",
        type=float,
        default=0.45,
        help="LPF cutoff normalized to input Nyquist (0..0.5), default: 0.45",
    )
    parser.add_argument(
        "--chunk-iq",
        type=int,
        default=1_000_000,
        help="Complex samples processed per chunk (default: 1000000)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    decimate_file_by2(
        in_path=args.input,
        out_path=args.output,
        num_taps=args.num_taps,
        cutoff_norm=args.cutoff_norm,
        chunk_iq=args.chunk_iq,
    )


if __name__ == "__main__":
    main()
