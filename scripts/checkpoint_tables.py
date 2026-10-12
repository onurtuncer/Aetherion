#!/usr/bin/env python3
"""
checkpoint_tables.py
--------------------
Print the per-checkpoint validation tables of doc/examples.rst for one NASA
TM-2015-218675 scenario from a fresh Aetherion run, laid out like the table in
the documentation and against the reference run the documentation names.

The plot scripts (plot_f16_s*_nasa.py, compare_sim_validation.py) write figures
and error_summary.csv files but never print values at the checkpoint times the
tables quote, so re-verifying a table after a change meant reading numbers off
by hand.  This script is that step.  It changes nothing in doc/; it prints, and
the reader compares.

Usage:
    python scripts/checkpoint_tables.py <scenario> <aetherion.csv>

    scenario : 8 9 10 11 12 13.1 13.2 13.3 13.4 15 16 17

Run parameters for <aetherion.csv> are the ones stated in doc/examples.rst for
that scenario (write interval 0.1 s for 8-10 and 17; every step otherwise).

Alignment: checkpoint rows take the sample nearest to the checkpoint time (no
interpolation, so wrapped angles are safe); peak errors use the same alignment
as the plot scripts -- time rounded to 4 decimals, inner join -- so that the
figures the doc quotes reproduce.  Scenario 17 interpolates, since its rows
include S1 burnout (37.4 s) and S2 ignition (131.8 s), which are not samples.

Dependencies: numpy, pandas
"""

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parent.parent
DATA = REPO / "data"

FT = 0.3048
DEG = math.pi / 180.0
KT = 0.5144444444
SLUGFT3 = 515.3788184
RHO0 = 1.225

# Same column mapping as scripts/plot_f16_s11_nasa.py (the subset used here).
NASA_TO_SI = [
    ("feVelocity_ft_s_X", "feVelocity_m_s_X", FT),
    ("feVelocity_ft_s_Y", "feVelocity_m_s_Y", FT),
    ("feVelocity_ft_s_Z", "feVelocity_m_s_Z", FT),
    ("altitudeMsl_ft", "altitudeMsl_m", FT),
    ("longitude_deg", "longitude_rad", DEG),
    ("latitude_deg", "latitude_rad", DEG),
    ("eulerAngle_deg_Yaw", "eulerAngle_rad_Yaw", DEG),
    ("eulerAngle_deg_Pitch", "eulerAngle_rad_Pitch", DEG),
    ("eulerAngle_deg_Roll", "eulerAngle_rad_Roll", DEG),
    ("airDensity_slug_ft3", "airDensity_kg_m3", SLUGFT3),
    ("mach", "mach", 1.0),
    ("trueAirspeed_nmi_h", "trueAirspeed_m_s", KT),
]

DIRS = {
    "8": "Atmos_08_DroppedSphere2DWindShear",
    "9": "Atmos_09_EastwardCannonball",
    "10": "Atmos_10_NorthwardCannonball",
    "11": "Atmos_11_TrimCheckSubsonicF16",
    "12": "Atmos_12_TrimCheckSupersonicF16",
    "13.1": "Atmos_13p1_SubsonicAltitudeChangeF16",
    "13.2": "Atmos_13p2_SubsonicAirspeedChangeF16",
    "13.3": "Atmos_13p3_SubsonicHeadingChangeF16",
    "13.4": "Atmos_13p4_SubsonicLateralSideStepF16",
    "15": "Atmos_15_CircleNorthPole",
    "16": "Atmos_16_CircleEquatorDateLine",
    "17": "Atmos_17_TwoStageRocketToOrbit",
}


def load(path):
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    df["time"] = df["time"].astype(float)
    return df


def _stem(scn):
    parts = DIRS[scn].split("_")
    return parts[0] + "_" + parts[1]


def nasa_si(scn, tag):
    """One imperial NASA reference run, converted to Aetherion's SI columns."""
    raw = load(DATA / DIRS[scn] / f"{_stem(scn)}_sim_{tag}.csv")
    si = pd.DataFrame({"time": raw["time"].values})
    for nasa_col, si_col, scale in NASA_TO_SI:
        if nasa_col in raw.columns:
            si[si_col] = raw[nasa_col].values * scale
    return si


def nasa_si_file(scn, tag):
    """The pre-converted *_si_units.csv file (scenarios 1-10)."""
    return load(DATA / DIRS[scn] / f"{_stem(scn)}_sim_{tag}_si_units.csv")


def at(df, t, col, tol=0.051):
    """Value of col at the sample nearest to t; NaN if none within tol."""
    i = int(np.argmin(np.abs(df["time"].values - t)))
    if abs(df["time"].values[i] - t) > tol:
        return float("nan")
    return float(df[col].values[i])


def interp(df, t, col):
    return float(np.interp(t, df["time"].values, df[col].values))


def joined(sim, ref):
    """Inner join on time rounded to 4 decimals, as the plot scripts do."""
    a, b = sim.copy(), ref.copy()
    a["time"] = a["time"].round(4)
    b["time"] = b["time"].round(4)
    return pd.merge(a, b, on="time", suffixes=("_s", "_r"))


def peak(m, col, scale=1.0, wrap=False):
    e = (m[col + "_s"] - m[col + "_r"]).values
    if wrap:
        e = (e + math.pi) % (2 * math.pi) - math.pi
    i = int(np.argmax(np.abs(e)))
    return abs(e[i]) * scale, m["time"].values[i]


# ---------------------------------------------------------------------------
# Scenarios 8-10: altitude / TAS every 5 s against sim_01, closest run, spread
# ---------------------------------------------------------------------------
def sphere(scn, sim):
    ref = nasa_si_file(scn, "01")
    print(f"Scenario {scn} vs Atmos_{scn.zfill(2)}_sim_01 (SI)")
    print("  t | Alt_ref [m] | Alt_sim [m] |  dAlt  | TAS_ref | TAS_sim |  dTAS")
    for t in (0, 5, 10, 15, 20, 25, 30):
        ar, as_ = at(ref, t, "altitudeMsl_m"), at(sim, t, "altitudeMsl_m")
        vr, vs = at(ref, t, "trueAirspeed_m_s"), at(sim, t, "trueAirspeed_m_s")
        print(f"{t:3d} | {ar:11.3f} | {as_:11.3f} | {as_ - ar:+6.3f} | {vr:7.3f} | {vs:7.3f} | {vs - vr:+6.3f}")

    refs = {}
    for tag in ("01", "02", "03", "04", "05", "06"):
        p = DATA / DIRS[scn] / f"{_stem(scn)}_sim_{tag}_si_units.csv"
        if p.exists():
            refs[tag] = load(p)
    for tag, r in refs.items():
        m = joined(sim, r)
        print(f"  max|dAlt| vs sim_{tag}: {peak(m, 'altitudeMsl_m')[0]:.3g} m")
    t = refs["01"]["time"].values
    stack = np.stack([np.interp(t, r["time"].values, r["altitudeMsl_m"].values) for r in refs.values()])
    print(f"  spread among the {len(refs)} NASA runs: {np.max(stack.max(0) - stack.min(0)):.3g} m")


# ---------------------------------------------------------------------------
# Scenarios 11-13.4: one end-time snapshot against the named NASA run
# ---------------------------------------------------------------------------
def f16_end(scn, sim, tag, t_end):
    ref = nasa_si(scn, tag)
    print(f"Scenario {scn}: t = {t_end} s, Aetherion vs NASA sim {tag}")
    rows = [
        ("altitudeMsl_m", "Altitude [ft]", 1 / FT),
        ("altitudeMsl_m", "Altitude [m]", 1),
        ("trueAirspeed_m_s", "TAS [m/s]", 1),
        ("trueAirspeed_m_s", "TAS [kt]", 1 / KT),
        ("mach", "Mach", 1),
        ("eulerAngle_rad_Pitch", "Pitch [deg]", 1 / DEG),
        ("eulerAngle_rad_Roll", "Roll [deg]", 1 / DEG),
        ("eulerAngle_rad_Yaw", "Yaw [deg]", 1 / DEG),
    ]
    for col, label, k in rows:
        if col in sim.columns and col in ref.columns:
            s, r = at(sim, t_end, col) * k, at(ref, t_end, col) * k
            print(f"  {label:14s} Aetherion {s:14.6f}   NASA {r:14.6f}   diff {s - r:+.6f}")
    if "airDensity_kg_m3" in sim.columns and "airDensity_kg_m3" in ref.columns:
        ks = at(sim, t_end, "trueAirspeed_m_s") * math.sqrt(at(sim, t_end, "airDensity_kg_m3") / RHO0) / KT
        kr = at(ref, t_end, "trueAirspeed_m_s") * math.sqrt(at(ref, t_end, "airDensity_kg_m3") / RHO0) / KT
        print(f"  {'KEAS [kt]':14s} Aetherion {ks:14.6f}   NASA {kr:14.6f}   diff {ks - kr:+.6f}")

    h0 = sim["altitudeMsl_m"].values[0]
    print(f"  max|h - h0| over run: {np.max(np.abs(sim['altitudeMsl_m'].values - h0)) / FT:.3f} ft"
          f"   (h0 = {h0 / FT:.2f} ft, h(end) - h0 = {(at(sim, t_end, 'altitudeMsl_m') - h0) / FT:+.3f} ft)")
    m = joined(sim, ref)
    print(f"  peak errors vs sim {tag} ({len(m)} joined samples):")
    for col, label, k in (("altitudeMsl_m", "alt [ft]", 1 / FT), ("eulerAngle_rad_Pitch", "pitch [deg]", 1 / DEG),
                          ("eulerAngle_rad_Roll", "roll [deg]", 1 / DEG), ("eulerAngle_rad_Yaw", "yaw [deg]", 1 / DEG)):
        if col + "_r" in m.columns:
            v, tp = peak(m, col, k)
            print(f"    {label:12s} {v:.4g} at t = {tp:.2f} s")


# ---------------------------------------------------------------------------
# Scenarios 15-16: the two 30 s tables against sim 05, then the summary errors
# ---------------------------------------------------------------------------
def circle(scn, sim):
    ref = nasa_si(scn, "05")
    print(f"Scenario {scn}:   t | Alt [ft] |   Lat    |    Lon    |   Yaw   | Pitch |   Roll   | Mach")
    for name, df in (("NASA sim 05", ref), ("Aetherion", sim)):
        print(f" -- {name}")
        for t in range(0, 181, 30):
            print(f"{t:4d} | {at(df, t, 'altitudeMsl_m') / FT:8.2f} | {at(df, t, 'latitude_rad') / DEG:+8.4f} | "
                  f"{at(df, t, 'longitude_rad') / DEG:+9.3f} | {at(df, t, 'eulerAngle_rad_Yaw') / DEG:+7.2f} | "
                  f"{at(df, t, 'eulerAngle_rad_Pitch') / DEG:5.3f} | {at(df, t, 'eulerAngle_rad_Roll') / DEG:+8.3f} | "
                  f"{at(df, t, 'mach'):.4f}")
    m = joined(sim, ref)
    lat, _ = peak(m, "latitude_rad", 1 / DEG)
    lon, _ = peak(m, "longitude_rad", 1 / DEG, wrap=True)
    alt, _ = peak(m, "altitudeMsl_m", 1 / FT)
    roll, _ = peak(m, "eulerAngle_rad_Roll", 1 / DEG)
    print(f"  max|lat err| {lat:.6f} deg ({lat * DEG * 6378137:.1f} m);  max|lon err| {lon:.4f} deg;"
          f"  max|alt err| {alt:.2f} ft;  roll peak {roll:.2f} deg")
    late = m[m["time"] >= 60]
    print(f"  max|roll err| for t >= 60 s: {peak(late, 'eulerAngle_rad_Roll', 1 / DEG)[0]:.3f} deg")
    h = sim["altitudeMsl_m"].values / FT
    k = sim["time"].values >= 30
    print(f"  max|h - 10000 ft| for t >= 30 s: {np.max(np.abs(h[k] - 10000)):.2f} ft")


# ---------------------------------------------------------------------------
# Scenario 17: the phase-event table against sim 06, then the phase summaries
# ---------------------------------------------------------------------------
def rocket(sim):
    ref = nasa_si("17", "06")
    print("Scenario 17 vs NASA sim 06")
    print("     t |   Alt_A [m] |  Alt_06 [m] |  Err%   | Pitch_A | Pitch_06 |  TAS_A  |  TAS_06")
    for t in (0, 5, 10, 20, 30, 37.4, 50, 100, 131.8, 150, 175, 200):
        a, r = interp(sim, t, "altitudeMsl_m"), interp(ref, t, "altitudeMsl_m")
        pa, pr = interp(sim, t, "eulerAngle_rad_Pitch") / DEG, interp(ref, t, "eulerAngle_rad_Pitch") / DEG
        va, vr = interp(sim, t, "trueAirspeed_m_s"), interp(ref, t, "trueAirspeed_m_s")
        err = (a - r) / r * 100 if r else 0.0
        print(f"{t:6.1f} | {a:11.1f} | {r:11.1f} | {err:+6.2f}% | {pa:7.3f} | {pr:8.3f} | {va:7.2f} | {vr:8.2f}")

    t_end = min(sim["time"].max(), ref["time"].max())
    a, r = interp(sim, t_end, "altitudeMsl_m"), interp(ref, t_end, "altitudeMsl_m")
    va, vr = interp(sim, t_end, "trueAirspeed_m_s"), interp(ref, t_end, "trueAirspeed_m_s")
    pa, pr = interp(sim, t_end, "eulerAngle_rad_Pitch") / DEG, interp(ref, t_end, "eulerAngle_rad_Pitch") / DEG
    print(f"  final (t = {t_end:.0f} s): altitude {a - r:+.1f} m ({(a - r) / r * 100:+.2f} %), "
          f"TAS {va - vr:+.2f} m/s ({(va - vr) / vr * 100:+.2f} %), pitch {pa - pr:+.3f} deg")

    m = joined(sim, ref)
    t = m["time"].values
    e = (m["altitudeMsl_m_s"] - m["altitudeMsl_m_r"]).values
    href = m["altitudeMsl_m_r"].values
    rel = np.where(href > 0, e / np.where(href > 0, href, 1.0) * 100, 0.0)
    v = (m["trueAirspeed_m_s_s"] - m["trueAirspeed_m_s_r"]).values
    for lo, hi, label in ((0, 37.4, "S1 burn"), (37.4, 131.8, "coast"), (131.8, 200, "S2 + free")):
        k = (t > lo) & (t <= hi)
        print(f"  {label:10s} max|alt err| {np.max(np.abs(e[k])):7.1f} m   max|rel| {np.max(np.abs(rel[k])):.3f} %"
              f"   max|TAS err| {np.max(np.abs(v[k])):.2f} m/s")
    k = (t > 0) & (t <= 37.4) & (np.abs(rel) > 0.16)
    if k.any():
        print(f"  S1 burn |rel alt err| > 0.16 % only for t <= {t[k].max():.1f} s "
              f"(|alt err| there <= {np.max(np.abs(e[k])):.2f} m)")


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in DIRS:
        print(__doc__)
        sys.exit(2)
    scn, path = sys.argv[1], Path(sys.argv[2])
    sim = load(path)
    print(f"[{path}]  {len(sim)} rows, t = {sim['time'].min():.1f} .. {sim['time'].max():.1f} s")
    if scn in ("8", "9", "10"):
        sphere(scn, sim)
    elif scn == "11":
        f16_end(scn, sim, "05", 180)
    elif scn == "12":
        f16_end(scn, sim, "02", 200)
        f16_end(scn, sim, "05", 180)
    elif scn == "13.1":
        f16_end(scn, sim, "05", 20)
    elif scn == "13.2":
        f16_end(scn, sim, "02", 20)
    elif scn == "13.3":
        f16_end(scn, sim, "05", 30)
    elif scn == "13.4":
        f16_end(scn, sim, "05", 60)
    elif scn in ("15", "16"):
        circle(scn, sim)
    elif scn == "17":
        rocket(sim)


if __name__ == "__main__":
    main()
