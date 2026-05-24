"""
plot_f16_s16_nasa02.py
Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University

Documentation-quality comparison of the Aetherion F-16 Scenario 16
closed-loop simulation against the NASA TM-2015-218675 Atmos_16_sim_02
reference trajectory (F-16 CCW circumnavigation of the equator/international
date line intersection, 3-nmi radius, 180 s).

Usage
-----
Source-tree:
    python scripts/plot_f16_s16_nasa02.py [sim_csv]

Build directory (after CMake copies this script next to the executable):
    python plot_f16_s16_nasa02.py [sim_csv]

    sim_csv  optional path to the simulation CSV (default: atmos_16_output.csv).

NOTE: the closed-loop LQR is stiff — use timeStep <= 0.02 s for accurate
results.  The default sim CSV is produced by:

    CircleEquatorDateLine --endTime 180 --timeStep 0.02 --outputFileName atmos_16_output.csv

Outputs (doc/_static/f16_s16/ or build-dir figures/f16_s16/):
    fig_circle.png            orbit radius [nmi] from IDL/equator and longitude
    fig_flight_envelope.png   altitude, TAS, Mach
    fig_attitude.png          pitch, roll, yaw
    fig_body_rates.png        p, q, r
    fig_position.png          latitude, longitude, altitude
    fig_ned_velocity.png      v_N, v_E, v_D
    fig_aero_forces.png       body Fx, Fy, Fz
    fig_aero_moments.png      L, M, N
    fig_atmosphere.png        a, rho, p, T
    fig_overview.png          all shared channels dashboard
    error_summary.csv         per-channel error statistics
"""

from __future__ import annotations
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd

# ── Paths ─────────────────────────────────────────────────────────────────────

_HERE = Path(__file__).resolve().parent
_BUILD_NASA = _HERE / "Atmos_16_sim_02.csv"
_IN_BUILD_DIR = _BUILD_NASA.exists()

if _IN_BUILD_DIR:
    _SIM_DEFAULT = _HERE / "atmos_16_output.csv"
    NASA_CSV = _BUILD_NASA
    OUT_DIR  = _HERE / "figures" / "f16_s16"
else:
    _REPO    = _HERE.parent
    _SIM_DEFAULT = _REPO / "atmos_16_output.csv"
    NASA_CSV = _REPO / "data" / "Atmos_16_CircleEquatorDateLine" / "Atmos_16_sim_02.csv"
    OUT_DIR  = _REPO / "doc" / "_static" / "f16_s16"

SIM_CSV = Path(sys.argv[1]) if len(sys.argv) > 1 else _SIM_DEFAULT
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Unit-conversion constants ─────────────────────────────────────────────────

kFt  = 0.3048
kDeg = np.pi / 180.0
kFtMin = kFt / 60.0
kSlugFt3 = 515.3788184
kLbfFt2  = 47.88025898
kDgR_K   = 5.0 / 9.0
kLbf     = 4.448221615260751
kFtLbf   = 1.355817948329279
kNmiH    = 0.5144444444

NASA_TO_SI = [
    ("feVelocity_ft_s_X",              "feVelocity_m_s_X",             kFt),
    ("feVelocity_ft_s_Y",              "feVelocity_m_s_Y",             kFt),
    ("feVelocity_ft_s_Z",              "feVelocity_m_s_Z",             kFt),
    ("altitudeMsl_ft",                 "altitudeMsl_m",                kFt),
    ("longitude_deg",                  "longitude_rad",                kDeg),
    ("latitude_deg",                   "latitude_rad",                 kDeg),
    ("localGravity_ft_s2",             "localGravity_m_s2",            kFt),
    ("eulerAngle_deg_Yaw",             "eulerAngle_rad_Yaw",           kDeg),
    ("eulerAngle_deg_Pitch",           "eulerAngle_rad_Pitch",         kDeg),
    ("eulerAngle_deg_Roll",            "eulerAngle_rad_Roll",          kDeg),
    ("bodyAngularRateWrtEi_deg_s_Roll",  "bodyAngularRateWrtEi_rad_s_Roll",  kDeg),
    ("bodyAngularRateWrtEi_deg_s_Pitch", "bodyAngularRateWrtEi_rad_s_Pitch", kDeg),
    ("bodyAngularRateWrtEi_deg_s_Yaw",   "bodyAngularRateWrtEi_rad_s_Yaw",   kDeg),
    ("altitudeRateWrtMsl_ft_min",      "altitudeRateWrtMsl_m_s",       kFtMin),
    ("speedOfSound_ft_s",              "speedOfSound_m_s",             kFt),
    ("airDensity_slug_ft3",            "airDensity_kg_m3",             kSlugFt3),
    ("ambientPressure_lbf_ft2",        "ambientPressure_Pa",           kLbfFt2),
    ("ambientTemperature_dgR",         "ambientTemperature_K",         kDgR_K),
    ("aero_bodyForce_lbf_X",           "aero_bodyForce_N_X",           kLbf),
    ("aero_bodyForce_lbf_Y",           "aero_bodyForce_N_Y",           kLbf),
    ("aero_bodyForce_lbf_Z",           "aero_bodyForce_N_Z",           kLbf),
    ("aero_bodyMoment_ftlbf_L",        "aero_bodyMoment_Nm_L",         kFtLbf),
    ("aero_bodyMoment_ftlbf_M",        "aero_bodyMoment_Nm_M",         kFtLbf),
    ("aero_bodyMoment_ftlbf_N",        "aero_bodyMoment_Nm_N",         kFtLbf),
    ("mach",                           "mach",                         1.0),
    ("trueAirspeed_nmi_h",             "trueAirspeed_m_s",             kNmiH),
]

# ── Plot style ────────────────────────────────────────────────────────────────

SIM_COLOR = "#1D4ED8"; REF_COLOR = "#DC2626"; ERR_COLOR = "#16A34A"
LW_MAIN = 1.6; LW_ERR = 1.0; FILL_A = 0.12; DPI = 200

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white",
    "axes.edgecolor": "#374151", "axes.grid": True,
    "grid.color": "#E5E7EB", "grid.linestyle": "--", "grid.linewidth": 0.5,
    "font.family": "sans-serif", "font.size": 9,
    "axes.titlesize": 9, "axes.labelsize": 9,
    "xtick.labelsize": 8, "ytick.labelsize": 8,
    "legend.framealpha": 0.92, "legend.edgecolor": "#D1D5DB", "legend.fontsize": 8,
})

# ── Data loading ──────────────────────────────────────────────────────────────

def load_sim(path):
    df = pd.read_csv(path); df.columns = [c.strip() for c in df.columns]; return df

def load_nasa_si(path):
    raw = pd.read_csv(path); raw.columns = [c.strip() for c in raw.columns]
    si = pd.DataFrame({"time": raw["time"].values.astype(float)})
    for nc, sc, scale in NASA_TO_SI:
        if nc in raw.columns:
            si[sc] = raw[nc].values * scale
    return si

def align(sim, ref):
    sim = sim.copy(); ref = ref.copy()
    sim["time"] = sim["time"].astype(float).round(4)
    ref["time"] = ref["time"].astype(float).round(4)
    merged = pd.merge(sim, ref, on="time", suffixes=("_s", "_r"))
    shared = [c for c in sim.columns if c != "time" and c in ref.columns]
    s = merged[["time"] + [f"{c}_s" for c in shared]].rename(columns={f"{c}_s": c for c in shared})
    r = merged[["time"] + [f"{c}_r" for c in shared]].rename(columns={f"{c}_r": c for c in shared})
    return s, r

# ── Plot helpers ──────────────────────────────────────────────────────────────

def _signal_error_row(ax_sig, ax_err, t, sim, ref, ylabel, unit="", scale=1.0):
    s = sim * scale; r = ref * scale; e = s - r
    ax_sig.plot(t, r, color=REF_COLOR, lw=LW_MAIN, label="NASA ref", zorder=3)
    ax_sig.plot(t, s, color=SIM_COLOR, lw=LW_MAIN, ls="--", label="Aetherion", zorder=4)
    ax_sig.fill_between(t, r, s, alpha=FILL_A, color=SIM_COLOR)
    label = f"{ylabel} [{unit}]" if unit else ylabel
    ax_sig.set_ylabel(label, labelpad=4)
    ax_sig.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.4g"))
    ax_sig.legend(loc="best")
    ax_err.plot(t, e, color=ERR_COLOR, lw=LW_ERR)
    ax_err.axhline(0, color="#9CA3AF", lw=0.7, ls=":")
    ax_err.fill_between(t, e, alpha=FILL_A, color=ERR_COLOR)
    eu = f"[{unit}]" if unit else ""
    ax_err.set_ylabel(f"Aetherion-NASA {eu}", labelpad=4, color=ERR_COLOR, fontsize=7)
    ax_err.tick_params(axis="y", labelcolor=ERR_COLOR, labelsize=7)
    ax_err.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.3g"))

def multi_panel(t, rows, sim_df, ref_df, title, fname, h_per_row=2.4):
    n = len(rows)
    fig = plt.figure(figsize=(13, max(n * h_per_row, 3.5)))
    fig.suptitle(title, fontsize=11, fontweight="bold", y=1.005)
    gs = gridspec.GridSpec(n, 2, figure=fig, hspace=0.55, wspace=0.38,
                           left=0.07, right=0.97, top=0.96, bottom=0.07)
    for i, (col, ylabel, unit, scale) in enumerate(rows):
        ax_sig = fig.add_subplot(gs[i, 0])
        ax_err = fig.add_subplot(gs[i, 1])
        if col not in sim_df.columns or col not in ref_df.columns:
            ax_sig.text(0.5, 0.5, f"{col}\n(n/a)", ha="center", va="center",
                        transform=ax_sig.transAxes, color="#9CA3AF")
            ax_err.set_visible(False)
            continue
        _signal_error_row(ax_sig, ax_err, t,
                          sim_df[col].values.astype(float),
                          ref_df[col].values.astype(float),
                          ylabel, unit, scale)
        if i < n - 1:
            ax_sig.set_xticklabels([]); ax_err.set_xticklabels([])
        else:
            ax_sig.set_xlabel("Time [s]"); ax_err.set_xlabel("Time [s]")
        if i == 0:
            ax_sig.set_title("Signal", fontsize=8, color="#6B7280", pad=3)
            ax_err.set_title("Error  (Aetherion - NASA)", fontsize=8, color="#6B7280", pad=3)
    out = OUT_DIR / fname
    fig.savefig(out, dpi=DPI, bbox_inches="tight"); plt.close(fig)
    return out

def overview(t, sim_df, ref_df):
    cols = [c for c in sim_df.columns if c != "time" and c in ref_df.columns
            and pd.api.types.is_numeric_dtype(sim_df[c])]
    nc, nr = 4, int(np.ceil(len(cols) / 4))
    fig, axes = plt.subplots(nr, nc, figsize=(nc * 3.8, nr * 2.6), squeeze=False)
    fig.suptitle("Aetherion F-16 Scenario 16 — Full Channel Comparison vs NASA Atmos_16_sim_02",
                 fontsize=12, fontweight="bold", y=1.01)
    for idx, col in enumerate(cols):
        r, c = divmod(idx, nc)
        ax = axes[r][c]
        ax.plot(t, ref_df[col].values.astype(float), color=REF_COLOR, lw=1.0, label="NASA")
        ax.plot(t, sim_df[col].values.astype(float), color=SIM_COLOR, lw=1.0, ls="--", label="Sim")
        ax.set_title(col.replace("_", " "), fontsize=6.5, pad=2)
        ax.set_xlabel("t [s]", fontsize=6); ax.tick_params(labelsize=6)
        ax.legend(fontsize=5, loc="best")
    for idx in range(len(cols), nr * nc):
        r, c = divmod(idx, nc); axes[r][c].set_visible(False)
    fig.tight_layout()
    out = OUT_DIR / "fig_overview.png"
    fig.savefig(out, dpi=DPI, bbox_inches="tight"); plt.close(fig)
    return out

def save_error_summary(t, sim_df, ref_df):
    cols = [c for c in sim_df.columns if c != "time" and c in ref_df.columns
            and pd.api.types.is_numeric_dtype(sim_df[c])]
    rows = []
    for col in cols:
        s = sim_df[col].values.astype(float); r = ref_df[col].values.astype(float)
        e = s - r
        with np.errstate(divide="ignore", invalid="ignore"):
            rel = np.where(np.abs(r) > 1e-12, e / np.abs(r) * 100.0, np.nan)
        ca = e[np.isfinite(e)]; cr = rel[np.isfinite(rel)]
        rows.append({"column": col,
                     "abs_mean": float(np.mean(ca)) if len(ca) else np.nan,
                     "abs_std":  float(np.std(ca))  if len(ca) else np.nan,
                     "abs_max":  float(np.max(np.abs(ca))) if len(ca) else np.nan,
                     "rel_mean_%": float(np.nanmean(rel)),
                     "rel_max_%":  float(np.nanmax(np.abs(cr))) if len(cr) else np.nan})
    out = OUT_DIR / "error_summary.csv"
    pd.DataFrame(rows).to_csv(out, index=False, float_format="%.6g")
    return out

# ── Orbit radius from equator/IDL crossing ────────────────────────────────────

def orbit_radius_nmi(lat_deg, lon_deg):
    """
    Distance from the equator/IDL crossing in nmi.
    The DML computes:
        degEFromTgt = lon+180 if lon<0 else lon-180
        ownshipN_ft = lat_deg * deg_to_ft
        ownshipE_ft = degEFromTgt * deg_to_ft * cos(lat_rad)
        r = sqrt(N^2 + E^2)
    """
    kDegToFt = 60.0 * 6076.12
    kFtToNmi = 1.0 / 6076.12
    deg_e_from_tgt = np.where(lon_deg < 0, lon_deg + 180.0, lon_deg - 180.0)
    n_ft = lat_deg * kDegToFt
    e_ft = deg_e_from_tgt * kDegToFt * np.cos(np.radians(lat_deg))
    return np.sqrt(n_ft**2 + e_ft**2) * kFtToNmi

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"Loading sim  : {SIM_CSV}")
    print(f"Loading NASA : {NASA_CSV}")
    if not SIM_CSV.exists():  sys.exit(f"ERROR: {SIM_CSV} not found")
    if not NASA_CSV.exists(): sys.exit(f"ERROR: {NASA_CSV} not found")

    sim_raw = load_sim(SIM_CSV)
    ref_si  = load_nasa_si(NASA_CSV)
    sim_df, ref_df = align(sim_raw, ref_si)
    t = sim_df["time"].values.astype(float)
    print(f"Aligned {len(t)} steps  ({t[0]:.2f} - {t[-1]:.2f} s)")
    print(f"Output -> {OUT_DIR}\n")

    r2d = 180.0 / np.pi

    # ── Circle orbit (primary channel for Scenario 16) ────────────────────────
    lat_s_deg = sim_df["latitude_rad"].values.astype(float) * r2d
    lat_r_deg = ref_df["latitude_rad"].values.astype(float) * r2d
    lon_s_deg = sim_df["longitude_rad"].values.astype(float) * r2d
    lon_r_deg = ref_df["longitude_rad"].values.astype(float) * r2d

    radius_s_nmi = orbit_radius_nmi(lat_s_deg, lon_s_deg)
    radius_r_nmi = orbit_radius_nmi(lat_r_deg, lon_r_deg)

    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    fig.suptitle("Scenario 16 — Equator/Date-Line Circumnavigation  (cmd: 3 nmi radius circle)",
                 fontsize=11, fontweight="bold")

    ax1, ax2, ax3 = axes
    ax1.plot(t, radius_r_nmi, color=REF_COLOR, lw=LW_MAIN, label="NASA ref")
    ax1.plot(t, radius_s_nmi, color=SIM_COLOR, lw=LW_MAIN, ls="--", label="Aetherion")
    ax1.axhline(3.0, color="#F59E0B", lw=1.0, ls=":", label="cmd 3 nmi")
    ax1.set_ylabel("Circle radius [nmi]"); ax1.legend(loc="best")
    ax1.set_title("Orbit radius from equator/IDL crossing", fontsize=8, color="#6B7280", pad=3)

    ax2.plot(t, radius_s_nmi - radius_r_nmi, color=ERR_COLOR, lw=LW_ERR)
    ax2.axhline(0, color="#9CA3AF", lw=0.7, ls=":")
    ax2.fill_between(t, radius_s_nmi - radius_r_nmi, alpha=FILL_A, color=ERR_COLOR)
    ax2.set_ylabel("Radius error [nmi]", color=ERR_COLOR)
    ax2.tick_params(axis="y", labelcolor=ERR_COLOR)

    ax3.plot(t, lat_r_deg, color=REF_COLOR, lw=LW_MAIN, label="NASA ref")
    ax3.plot(t, lat_s_deg, color=SIM_COLOR, lw=LW_MAIN, ls="--", label="Aetherion")
    ax3.set_ylabel("Latitude [deg]"); ax3.set_xlabel("Time [s]"); ax3.legend(loc="best")
    ax3.set_title("Latitude oscillation (orbit around equator)", fontsize=8, color="#6B7280", pad=3)

    fig.tight_layout()
    out = OUT_DIR / "fig_circle.png"
    fig.savefig(out, dpi=DPI, bbox_inches="tight"); plt.close(fig)
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("altitudeMsl_m",    "Altitude MSL",  "m",   1.0),
        ("trueAirspeed_m_s", "True airspeed", "m/s", 1.0),
        ("mach",             "Mach",          "-",   1.0),
    ], sim_df, ref_df, "Scenario 16 — Flight Envelope", "fig_flight_envelope.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("eulerAngle_rad_Pitch", "Pitch theta", "deg", r2d),
        ("eulerAngle_rad_Roll",  "Roll phi",    "deg", r2d),
        ("eulerAngle_rad_Yaw",   "Yaw psi",     "deg", r2d),
    ], sim_df, ref_df, "Scenario 16 — Euler Attitude Angles", "fig_attitude.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("bodyAngularRateWrtEi_rad_s_Roll",  "Roll rate p",  "deg/s", r2d),
        ("bodyAngularRateWrtEi_rad_s_Pitch", "Pitch rate q", "deg/s", r2d),
        ("bodyAngularRateWrtEi_rad_s_Yaw",   "Yaw rate r",   "deg/s", r2d),
    ], sim_df, ref_df, "Scenario 16 — Body Angular Rates", "fig_body_rates.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("latitude_rad",  "Latitude",  "deg", r2d),
        ("longitude_rad", "Longitude", "deg", r2d),
        ("altitudeMsl_m", "Altitude",  "m",   1.0),
    ], sim_df, ref_df, "Scenario 16 — Geographic Position", "fig_position.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("feVelocity_m_s_X", "v_N", "m/s", 1.0),
        ("feVelocity_m_s_Y", "v_E", "m/s", 1.0),
        ("feVelocity_m_s_Z", "v_D", "m/s", 1.0),
    ], sim_df, ref_df, "Scenario 16 — NED Velocity", "fig_ned_velocity.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("aero_bodyForce_N_X", "Aero Fx", "N", 1.0),
        ("aero_bodyForce_N_Y", "Aero Fy", "N", 1.0),
        ("aero_bodyForce_N_Z", "Aero Fz", "N", 1.0),
    ], sim_df, ref_df, "Scenario 16 — Aerodynamic Body Forces", "fig_aero_forces.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("aero_bodyMoment_Nm_L", "Roll moment L",  "N*m", 1.0),
        ("aero_bodyMoment_Nm_M", "Pitch moment M", "N*m", 1.0),
        ("aero_bodyMoment_Nm_N", "Yaw moment N",   "N*m", 1.0),
    ], sim_df, ref_df, "Scenario 16 — Aerodynamic Body Moments", "fig_aero_moments.png")
    print(f"  {out.name}")

    out = multi_panel(t, [
        ("speedOfSound_m_s",    "Speed of sound", "m/s",   1.0),
        ("airDensity_kg_m3",    "Air density",    "kg/m3", 1.0),
        ("ambientPressure_Pa",  "Static pressure","Pa",    1.0),
        ("ambientTemperature_K","Temperature",    "K",     1.0),
    ], sim_df, ref_df, "Scenario 16 — Atmospheric State", "fig_atmosphere.png",
    h_per_row=2.1)
    print(f"  {out.name}")

    out = overview(t, sim_df, ref_df)
    print(f"  {out.name}")

    out = save_error_summary(t, sim_df, ref_df)
    print(f"  {out.name}")
    print("\nDone.")

if __name__ == "__main__":
    main()
