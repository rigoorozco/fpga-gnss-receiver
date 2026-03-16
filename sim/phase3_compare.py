#!/usr/bin/env python3
import argparse
import math
import re
import statistics
import sys
from typing import List, Optional, Tuple


LLH = Tuple[float, float, float]


def parse_expected(path: str) -> List[LLH]:
    # Supports both "Height= 100.795" and "Height = 100.795"
    pat = re.compile(
        r"Lat\s*=\s*([-+]?\d+(?:\.\d+)?)\s*\[deg\],\s*"
        r"Long\s*=\s*([-+]?\d+(?:\.\d+)?)\s*\[deg\],\s*"
        r"Height\s*=\s*([-+]?\d+(?:\.\d+)?)\s*\[m\]"
    )
    out: List[LLH] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = pat.search(line)
            if m:
                out.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return out


def parse_sim(path: str) -> Tuple[List[LLH], List[int]]:
    pat = re.compile(
        r"Position at .*?using\s+(\d+)\s+observations\s+is\s+"
        r"Lat\s*=\s*([-+]?\d+(?:\.\d+)?)\s*\[deg\],\s*"
        r"Long\s*=\s*([-+]?\d+(?:\.\d+)?)\s*\[deg\],\s*"
        r"Height\s*=\s*([-+]?\d+(?:\.\d+)?)\s*\[m\]"
    )
    out: List[LLH] = []
    obs_used: List[int] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = pat.search(line)
            if m:
                obs_used.append(int(m.group(1)))
                out.append((float(m.group(2)), float(m.group(3)), float(m.group(4))))
    return out, obs_used


def horiz_distance_m(a: LLH, b: LLH) -> float:
    lat1, lon1, _ = a
    lat2, lon2, _ = b
    # Equirectangular approximation for small separations.
    r = 6371000.0
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlat = lat2r - lat1r
    dlon = math.radians(lon2 - lon1)
    x = dlon * math.cos((lat1r + lat2r) * 0.5)
    y = dlat
    return r * math.sqrt(x * x + y * y)


def rms(vals: List[float]) -> float:
    if not vals:
        return float("nan")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def p95(vals: List[float]) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    idx = int(math.ceil(0.95 * len(s))) - 1
    idx = max(0, min(idx, len(s) - 1))
    return s[idx]


def phase3_metrics(sim_llh: List[LLH], exp_llh: List[LLH]):
    n = min(len(sim_llh), len(exp_llh))
    if n == 0:
        return None

    sim = sim_llh[:n]
    exp = exp_llh[:n]

    h_err = [horiz_distance_m(s, e) for s, e in zip(sim, exp)]
    v_err = [abs(s[2] - e[2]) for s, e in zip(sim, exp)]
    e3d = [math.sqrt(h * h + v * v) for h, v in zip(h_err, v_err)]

    mean_lat = statistics.fmean([p[0] for p in sim])
    mean_lon = statistics.fmean([p[1] for p in sim])
    mean_h = statistics.fmean([p[2] for p in sim])
    mean_pt: LLH = (mean_lat, mean_lon, mean_h)
    jitter_h = [horiz_distance_m(p, mean_pt) for p in sim]

    return {
        "paired_epochs": n,
        "horizontal_rms_m": rms(h_err),
        "vertical_rms_m": rms(v_err),
        "error_3d_p95_m": p95(e3d),
        "static_jitter_rms_m": rms(jitter_h),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 3 GNSS-SDR comparison")
    ap.add_argument("--sim-log", required=True)
    ap.add_argument("--expected", required=True)
    ap.add_argument("--strict", action="store_true", help="Exit non-zero when any primary metric fails")
    args = ap.parse_args()

    expected = parse_expected(args.expected)
    sim, obs_used = parse_sim(args.sim_log)

    fixes_ok = len(expected) > 0 and len(sim) > 0
    metrics = phase3_metrics(sim, expected)

    rows = []

    def add_metric(name: str, value: Optional[float], target: str, passed: bool):
        if value is None:
            value_s = "N/A"
        else:
            value_s = f"{value:.3f}"
        rows.append((name, value_s, target, "PASS" if passed else "FAIL"))

    add_metric("First fix presence", float(1.0 if fixes_ok else 0.0), "both have >=1 fix", fixes_ok)

    hard_fail = not fixes_ok

    if metrics is None:
        add_metric("Horizontal RMS [m]", None, "<= 25", False)
        add_metric("Vertical RMS [m]", None, "<= 40", False)
        add_metric("3D p95 [m]", None, "<= 75", False)
        add_metric("Static jitter RMS [m]", None, "<= 15", False)
        hard_fail = True
    else:
        h_ok = metrics["horizontal_rms_m"] <= 25.0
        v_ok = metrics["vertical_rms_m"] <= 40.0
        p95_ok = metrics["error_3d_p95_m"] <= 75.0
        j_ok = metrics["static_jitter_rms_m"] <= 15.0

        add_metric("Horizontal RMS [m]", metrics["horizontal_rms_m"], "<= 25", h_ok)
        add_metric("Vertical RMS [m]", metrics["vertical_rms_m"], "<= 40", v_ok)
        add_metric("3D p95 [m]", metrics["error_3d_p95_m"], "<= 75", p95_ok)
        add_metric("Static jitter RMS [m]", metrics["static_jitter_rms_m"], "<= 15", j_ok)

        hard_fail = hard_fail or (not h_ok) or (not v_ok) or (not p95_ok) or (not j_ok)

    min_obs = min(obs_used) if obs_used else None
    obs_ok = (min_obs is not None) and (min_obs >= 4)
    add_metric("Min observations used", None if min_obs is None else float(min_obs), ">= 4", obs_ok)
    hard_fail = hard_fail or (not obs_ok)

    paired = metrics["paired_epochs"] if metrics is not None else 0

    print("Phase 3 GNSS-SDR Comparison")
    print(f"- Expected fixes parsed: {len(expected)}")
    print(f"- Simulation fixes parsed: {len(sim)}")
    print(f"- Paired epochs used: {paired}")
    print("")
    print("| Metric | Value | Target | Result |")
    print("| --- | ---: | ---: | :---: |")
    for r in rows:
        print(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |")

    if args.strict and hard_fail:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
