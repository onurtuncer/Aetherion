#!/usr/bin/env python3
"""
plot_scenario17.py
------------------
Scenario 17 (two-stage rocket to orbit) altitude and speed profiles against the
NASA TM-2015-218675 reference runs, for the EUCASS paper.

Three reference runs are published.  Sims 04 and 05 agree with each other to
6.6 m in altitude over the full 200 s and are plotted as a single band; sim 06
is the independent second implementation.  The shaded band between them is the
reference envelope -- the spread the reference implementations themselves span
-- which is the quantity Aetherion should be judged against, rather than the
error to any one nominated run.

Lower panels show the signed difference of Aetherion from sim 06, with the
burn events marked, so that the concentration of error at S1 burn and S2
burnout is visible rather than merely asserted.

Usage (from the repo root):
    python papers/eucass/scripts/plot_scenario17.py <aetherion.csv>

Output: papers/eucass/figures/scenario17.pdf / .png

Dependencies: numpy, pandas, matplotlib
"""

import argparse
import math
import os
import sys

import numpy as np
import pandas as pd

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
REF_DIR     = os.path.join(REPO_ROOT, "data", "Atmos_17_TwoStageRocketToOrbit")
FIGURES_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "figures")

FT2M      = 0.3048
R_EARTH_M = 6378137.0

# (time [s], label) -- from the staging logic in TwoStageRocket.cpp
EVENTS = ((37.4, "S1 burnout"), (131.8, "S2 ignition"), (193.0, "S2 burnout"))

AE_COLOR  = "#2563EB"
REF_COLOR = "#DC2626"
BAND      = "#DC2626"


def ground_range_m(lon, lat):
    dlat, dlon = lat - lat[0], lon - lon[0]
    a = np.sin(dlat / 2) ** 2 + np.cos(lat[0]) * np.cos(lat) * np.sin(dlon / 2) ** 2
    return 2.0 * R_EARTH_M * np.arcsin(np.sqrt(np.clip(a, 0.0, 1.0)))


def load_ref(tag):
    df = pd.read_csv(os.path.join(REF_DIR, f"Atmos_17_sim_{tag}.csv"))
    df.columns = [c.strip() for c in df.columns]
    t = df["time"].astype(float).values
    alt = df["altitudeMsl_ft"].astype(float).values * FT2M
    v = np.stack([df[f"feVelocity_ft_s_{a}"].astype(float).values for a in "XYZ"])
    return t, alt, np.linalg.norm(v, axis=0) * FT2M


def load_ae(path):
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    t = df["time"].astype(float).values
    alt = df["altitudeMsl_m"].astype(float).values
    v = np.stack([df[f"feVelocity_m_s_{a}"].astype(float).values for a in "XYZ"])
    return t, alt, np.linalg.norm(v, axis=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim", help="Aetherion output CSV")
    ap.add_argument("--outdir", default=FIGURES_DIR,
                    help="directory to write scenario17.pdf/.png into "
                         "(default: papers/eucass/figures)")
    args = ap.parse_args()
    outdir = args.outdir

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- pip install matplotlib")
        sys.exit(1)

    t_ae, alt_ae, spd_ae = load_ae(args.sim)
    refs = {tag: load_ref(tag) for tag in ("04", "05", "06")}

    tg = np.linspace(0.0, min(200.0, t_ae.max()), 2001)
    A_alt = np.interp(tg, t_ae, alt_ae) / 1000.0
    A_spd = np.interp(tg, t_ae, spd_ae)

    R_alt = {k: np.interp(tg, v[0], v[1]) / 1000.0 for k, v in refs.items()}
    R_spd = {k: np.interp(tg, v[0], v[2]) for k, v in refs.items()}

    alt_lo = np.min(np.stack(list(R_alt.values())), axis=0)
    alt_hi = np.max(np.stack(list(R_alt.values())), axis=0)
    spd_lo = np.min(np.stack(list(R_spd.values())), axis=0)
    spd_hi = np.max(np.stack(list(R_spd.values())), axis=0)

    TEXTWIDTH_PT = 430.0
    FIG_W = TEXTWIDTH_PT / 72.27
    fig, axes = plt.subplots(2, 2, figsize=(FIG_W, FIG_W * 0.62),
                             gridspec_kw=dict(height_ratios=[2.4, 1.0]),
                             sharex=True)

    panels = (
        (0, A_alt, R_alt, alt_lo, alt_hi, "Altitude MSL [km]", r"$\Delta h$ [km]"),
        (1, A_spd, R_spd, spd_lo, spd_hi, "Speed [m/s]",       r"$\Delta v$ [m/s]"),
    )

    for col, A, R, lo, hi, ylab, dylab in panels:
        ax, axd = axes[0][col], axes[1][col]

        ax.fill_between(tg, lo, hi, color=BAND, alpha=0.18, lw=0,
                        label="NASA reference envelope")
        ax.plot(tg, R["06"], "-", color=REF_COLOR, lw=1.2, label="NASA sim 06")
        ax.plot(tg, A, "-", color=AE_COLOR, lw=1.5, label="Aetherion")
        ax.set_ylabel(ylab, fontsize=9)
        ax.grid(True, ls=":", lw=0.5, alpha=0.6)
        ax.tick_params(labelsize=8)

        d = A - R["06"]
        axd.axhline(0.0, color="#666666", lw=0.7)
        axd.fill_between(tg, lo - R["06"], hi - R["06"], color=BAND, alpha=0.18, lw=0)
        axd.plot(tg, d, "-", color=AE_COLOR, lw=1.2)
        axd.set_ylabel(dylab, fontsize=9)
        axd.set_xlabel(r"Time $t$ [s]", fontsize=9)
        axd.grid(True, ls=":", lw=0.5, alpha=0.6)
        axd.tick_params(labelsize=8)

        for a in (ax, axd):
            for te, _ in EVENTS:
                a.axvline(te, color="#444444", ls="--", lw=0.7, alpha=0.7)

        for te, lab in EVENTS:
            ax.annotate(lab, xy=(te, ax.get_ylim()[1]), xytext=(2, -2),
                        textcoords="offset points", rotation=90,
                        va="top", ha="left", fontsize=6.5, color="#444444")

    axes[0][0].legend(fontsize=7.5, loc="lower right", framealpha=0.92)
    fig.suptitle("Scenario 17: two-stage rocket to orbit", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    os.makedirs(outdir, exist_ok=True)
    for ext, dpi in (("pdf", 300), ("png", 200)):
        p = os.path.join(outdir, f"scenario17.{ext}")
        fig.savefig(p, format=ext, dpi=dpi, bbox_inches="tight")
        print(f"saved {p}")
    plt.close(fig)

    print(f"\nfinal altitude: Aetherion {A_alt[-1]:.2f} km, sim06 {R_alt['06'][-1]:.2f} km")
    print(f"final speed   : Aetherion {A_spd[-1]:.1f} m/s, sim06 {R_spd['06'][-1]:.1f} m/s")


if __name__ == "__main__":
    main()
