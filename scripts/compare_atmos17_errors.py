#!/usr/bin/env python3
"""
compare_atmos17_errors.py
-------------------------
Scenario 17 (two-stage rocket to orbit) validation errors against the NASA
TM-2015-218675 reference runs, for the table in papers/eucass/main.tex.

Reports, for each quantity, the peak absolute difference between an Aetherion
run and each reference sim, plus the peak spread *among* the reference sims
themselves.  The spread matters: where the reference implementations disagree
with each other by more than Aetherion differs from any one of them, quoting a
single error against a single nominated run overstates the precision of the
comparison.  This mirrors the methodology already used for the ballistic and
F-16 scenarios (Section 6.2 of the paper).

Aetherion writes SI (m, m/s, rad); the NASA files are imperial (ft, ft/s, deg).
Mixing the two silently is the most likely cause of the disagreement between
earlier ad-hoc comparison scripts, so every conversion here is explicit and
uses the same constants as scripts/plot_atmos17_scenario17.py.

Usage:
    python scripts/compare_atmos17_errors.py <aetherion.csv> [--latex]

Dependencies: numpy, pandas
"""

import argparse
import math
import os
import sys

import numpy as np
import pandas as pd

# Same constants as scripts/plot_atmos17_scenario17.py
FT2M    = 0.3048
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
R_EARTH_M = 6378137.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF_DIR   = os.path.join(REPO_ROOT, "data", "Atmos_17_TwoStageRocketToOrbit")
REF_SIMS  = ("04", "05", "06")


def downrange_m(lon_rad, lat_rad, alt_m):
    """Ground range from the launch point along the surface, in metres.

    Great-circle (haversine) distance on a sphere of radius R_EARTH_M.  The
    projection is deliberately onto the surface rather than onto the vehicle's
    own radius: carrying (R + h) with h up to \\SI{234}{km} inflates the range
    by \\SI{3.7}{\\percent}, which is comparable to the quantity being measured.
    The two constructions agree to better than \\SI{0.1}{\\percent} here, but
    only the surface projection is what "down-range" conventionally denotes.
    """
    del alt_m  # surface projection: altitude deliberately not carried
    lat0, lon0 = lat_rad[0], lon_rad[0]
    dlat, dlon = lat_rad - lat0, lon_rad - lon0
    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat0) * np.cos(lat_rad) * np.sin(dlon / 2.0) ** 2
    return 2.0 * R_EARTH_M * np.arcsin(np.sqrt(np.clip(a, 0.0, 1.0)))


def load_reference(tag):
    """Load one NASA reference run and return SI-converted quantities."""
    path = os.path.join(REF_DIR, f"Atmos_17_sim_{tag}.csv")
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]

    out = pd.DataFrame()
    out["time"]        = df["time"].astype(float)
    out["alt_m"]       = df["altitudeMsl_ft"].astype(float) * FT2M
    lon = df["longitude_deg"].astype(float).values * DEG2RAD
    lat = df["latitude_deg"].astype(float).values * DEG2RAD
    out["pitch_deg"]   = df["eulerAngle_deg_Pitch"].astype(float)
    out["pitchrate_dps"] = df["bodyAngularRateWrtEi_deg_s_Pitch"].astype(float)

    v = np.stack([df[f"feVelocity_ft_s_{a}"].astype(float).values for a in "XYZ"])
    out["speed_ms"]    = np.linalg.norm(v, axis=0) * FT2M
    out["downrange_m"] = downrange_m(lon, lat, out["alt_m"].values)
    return out


def load_aetherion(path):
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]

    out = pd.DataFrame()
    out["time"]        = df["time"].astype(float)
    out["alt_m"]       = df["altitudeMsl_m"].astype(float)
    lon = df["longitude_rad"].astype(float).values
    lat = df["latitude_rad"].astype(float).values
    out["pitch_deg"]   = df["eulerAngle_rad_Pitch"].astype(float) * RAD2DEG
    out["pitchrate_dps"] = df["bodyAngularRateWrtEi_rad_s_Pitch"].astype(float) * RAD2DEG

    v = np.stack([df[f"feVelocity_m_s_{a}"].astype(float).values for a in "XYZ"])
    out["speed_ms"]    = np.linalg.norm(v, axis=0)
    out["downrange_m"] = downrange_m(lon, lat, out["alt_m"].values)
    return out


QUANTITIES = (
    # key,            label,                 unit
    ("alt_m",         "Altitude MSL",        "m"),
    ("downrange_m",   "Down-range position", "m"),
    ("pitch_deg",     "Pitch angle",         "deg"),
    ("pitchrate_dps", "Body pitch rate",     "deg/s"),
    ("speed_ms",      "Vehicle speed",       "m/s"),
)

# The run nominated as the comparison reference in the paper.
NOMINATED = "06"


def resample(src, t):
    """Linear interpolation of every quantity onto the common time grid t."""
    out = {}
    for key, *_ in QUANTITIES:
        out[key] = np.interp(t, src["time"].values, src[key].values)
    return out


def fmt_si(v, unit):
    """Format a magnitude for the LaTeX table, promoting metres to km."""
    if unit == "m" and abs(v) >= 1000.0:
        return rf"\SI{{{v/1000.0:.3g}}}{{\kilo\metre}}"
    if abs(v) >= 1000.0:
        e = int(math.floor(math.log10(abs(v))))
        return rf"${v/10**e:.2f}\times10^{{{e}}}$"
    return f"{v:.3g}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim", help="Aetherion output CSV")
    ap.add_argument("--latex", action="store_true", help="emit LaTeX table rows")
    args = ap.parse_args()

    ae = load_aetherion(args.sim)
    refs = {tag: load_reference(tag) for tag in REF_SIMS}

    # Common grid: the overlap of every series, at the coarsest usable spacing.
    t_end = min([ae["time"].max()] + [r["time"].max() for r in refs.values()])
    t_beg = max([ae["time"].min()] + [r["time"].min() for r in refs.values()])
    n = int(round((t_end - t_beg) / 0.1)) + 1
    t = np.linspace(t_beg, t_end, n)

    print(f"Aetherion run : {args.sim}")
    print(f"               {len(ae)} rows, t = {ae['time'].min():.1f} .. {ae['time'].max():.1f} s")
    for tag, r in refs.items():
        print(f"NASA sim_{tag}   : {len(r)} rows, t = {r['time'].min():.1f} .. {r['time'].max():.1f} s")
    print(f"Comparison window: {t_beg:.1f} .. {t_end:.1f} s  ({n} samples)")
    if t_end < 199.0:
        print(f"\n*** WARNING: Aetherion run stops at {ae['time'].max():.1f} s; "
              f"the reference covers 200 s.  Numbers below are NOT the full scenario.")

    A = resample(ae, t)
    R = {tag: resample(r, t) for tag, r in refs.items()}

    print()
    hdr = (f"{'Quantity':<21}{'unit':<7}{'peak err':>11}{'at t':>7}"
           f"{'final err':>11}{'NASA spread':>13}{'ref range':>24}")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for key, label, unit in QUANTITIES:
        d = np.abs(A[key] - R[NOMINATED][key])
        i = int(np.argmax(d))
        peak, t_peak, final = d[i], t[i], d[-1]

        stack = np.stack([R[tag][key] for tag in REF_SIMS])
        spread = np.max(stack.max(axis=0) - stack.min(axis=0))

        ref = R[NOMINATED][key]
        lo, hi = ref.min(), ref.max()
        rows.append((label, unit, peak, t_peak, final, spread, lo, hi))

        print(f"{label:<21}{unit:<7}{peak:>11.4g}{t_peak:>7.1f}{final:>11.4g}"
              f"{spread:>13.4g}{f'{lo:.4g} .. {hi:.4g}':>24}")

    # How far apart are the reference implementations themselves?
    print("\nPairwise reference disagreement (peak over the window):")
    for a in range(len(REF_SIMS)):
        for b in range(a + 1, len(REF_SIMS)):
            ta, tb = REF_SIMS[a], REF_SIMS[b]
            worst = {lab: np.max(np.abs(R[ta][k] - R[tb][k]))
                     for k, lab, _ in QUANTITIES}
            summary = ", ".join(f"{lab.split()[0]} {v:.3g}" for lab, v in worst.items())
            print(f"  sim_{ta} vs sim_{tb}: {summary}")

    if args.latex:
        print("\n% --- LaTeX rows for tab:scenario17_errors ---")
        for label, unit, peak, t_peak, final, spread, lo, hi in rows:
            rng = rf"{fmt_si(lo, unit)}--{fmt_si(hi, unit)}"
            print(f"    {label} [{unit}]".ljust(32)
                  + f"& {fmt_si(peak, unit):<22} & {t_peak:>5.1f} "
                  + f"& {fmt_si(final, unit):<22} & {fmt_si(spread, unit):<22} "
                  + f"& {rng} \\\\")

    print("\nWhere 'NASA spread' exceeds 'peak err', the reference implementations "
          "disagree\namong themselves by more than Aetherion differs from the "
          "nominated one.")


if __name__ == "__main__":
    main()
