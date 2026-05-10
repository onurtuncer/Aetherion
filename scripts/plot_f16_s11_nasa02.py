"""
plot_f16_s11_nasa02.py
Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University

Documentation-quality comparison of the Aetherion F-16 Case-11 simulation
against the NASA TM-2015-218675 Atmos_11_sim_02 reference trajectory.

Usage
-----
Source-tree (repo root as working directory):
    python scripts/plot_f16_s11_nasa02.py

Build directory (after CMake copies this script next to the executable):
    python plot_f16_s11_nasa02.py [sim_csv]

    sim_csv  optional path to the simulation output CSV (default: f16_s11_200s.csv
             in the same directory as this script).

The script auto-detects its location:
  - If Atmos_11_sim_02.csv is a sibling (build-dir layout), all paths resolve
    relative to the script's own directory.
  - Otherwise it falls back to the source-tree layout (scripts/ -> repo root).

Outputs (figures/ sub-directory next to this script):
    fig_flight_envelope.png   altitude, TAS, Mach
    fig_attitude.png          pitch, roll, yaw
    fig_body_rates.png        roll, pitch, yaw rate
    fig_position.png          latitude, longitude, altitude
    fig_aero_forces.png       aero Fx, Fy, Fz
    fig_aero_moments.png      aero L, M, N
    fig_atmosphere.png        speed-of-sound, density, pressure, temperature
    fig_ned_velocity.png      NED velocity components
    fig_overview.png          24-channel overview dashboard
    error_summary.csv         per-channel absolute-error statistics
"""

from __future__ import annotations

import os
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

# Build-dir layout: NASA CSV is a sibling of this script (copied by CMake).
# Source-tree layout: this script lives in scripts/, repo root is one level up.
_BUILD_NASA = _HERE / "Atmos_11_sim_02.csv"
_IN_BUILD_DIR = _BUILD_NASA.exists()

if _IN_BUILD_DIR:
    # Running from the CMake build directory
    _SIM_DEFAULT = _HERE / "f16_s11_200s.csv"
    NASA_CSV = _BUILD_NASA
    OUT_DIR  = _HERE / "figures" / "f16_s11"
else:
    # Running from the source tree (scripts/ directory)
    _REPO    = _HERE.parent
    _SIM_DEFAULT = _REPO / "f16_s11_200s.csv"
    NASA_CSV = _REPO / "data" / "Atmos_11_TrimCheckSubsonicF16" / "Atmos_11_sim_02.csv"
    OUT_DIR  = _REPO / "doc" / "figures" / "f16_s11"

# Optional positional argument overrides the sim CSV path.
SIM_CSV = Path(sys.argv[1]) if len(sys.argv) > 1 else _SIM_DEFAULT

OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Unit-conversion constants ─────────────────────────────────────────────────

kFt_m        = 0.3048
kDeg_rad     = np.pi / 180.0
kFtMin_ms    = kFt_m / 60.0
kSlug_kg     = 14.59390294
kSlugFt3_kgm3= kSlug_kg / kFt_m**3   # 515.3788184
kLbfFt2_Pa   = 47.88025898            # 1 lbf/ft² in Pa
kDgR_K       = 5.0 / 9.0             # °Rankine → Kelvin
kLbf_N       = 4.448221615260751
kFtLbf_Nm    = 1.355817948329279
kNmiH_ms     = 0.5144444444          # knots → m/s

# ── NASA imperial → SI column mapping ────────────────────────────────────────
# (nasa_col, si_col, scale_factor)

NASA_TO_SI: list[tuple[str, str, float]] = [
    ("feVelocity_ft_s_X",              "feVelocity_m_s_X",             kFt_m),
    ("feVelocity_ft_s_Y",              "feVelocity_m_s_Y",             kFt_m),
    ("feVelocity_ft_s_Z",              "feVelocity_m_s_Z",             kFt_m),
    ("altitudeMsl_ft",                 "altitudeMsl_m",                kFt_m),
    ("longitude_deg",                  "longitude_rad",                kDeg_rad),
    ("latitude_deg",                   "latitude_rad",                 kDeg_rad),
    ("localGravity_ft_s2",             "localGravity_m_s2",            kFt_m),
    ("eulerAngle_deg_Yaw",             "eulerAngle_rad_Yaw",           kDeg_rad),
    ("eulerAngle_deg_Pitch",           "eulerAngle_rad_Pitch",         kDeg_rad),
    ("eulerAngle_deg_Roll",            "eulerAngle_rad_Roll",          kDeg_rad),
    ("bodyAngularRateWrtEi_deg_s_Roll",  "bodyAngularRateWrtEi_rad_s_Roll",  kDeg_rad),
    ("bodyAngularRateWrtEi_deg_s_Pitch", "bodyAngularRateWrtEi_rad_s_Pitch", kDeg_rad),
    ("bodyAngularRateWrtEi_deg_s_Yaw",   "bodyAngularRateWrtEi_rad_s_Yaw",   kDeg_rad),
    ("altitudeRateWrtMsl_ft_min",      "altitudeRateWrtMsl_m_s",       kFtMin_ms),
    ("speedOfSound_ft_s",              "speedOfSound_m_s",             kFt_m),
    ("airDensity_slug_ft3",            "airDensity_kg_m3",             kSlugFt3_kgm3),
    ("ambientPressure_lbf_ft2",        "ambientPressure_Pa",           kLbfFt2_Pa),
    ("ambientTemperature_dgR",         "ambientTemperature_K",         kDgR_K),
    ("aero_bodyForce_lbf_X",           "aero_bodyForce_N_X",           kLbf_N),
    ("aero_bodyForce_lbf_Y",           "aero_bodyForce_N_Y",           kLbf_N),
    ("aero_bodyForce_lbf_Z",           "aero_bodyForce_N_Z",           kLbf_N),
    ("aero_bodyMoment_ftlbf_L",        "aero_bodyMoment_Nm_L",         kFtLbf_Nm),
    ("aero_bodyMoment_ftlbf_M",        "aero_bodyMoment_Nm_M",         kFtLbf_Nm),
    ("aero_bodyMoment_ftlbf_N",        "aero_bodyMoment_Nm_N",         kFtLbf_Nm),
    ("mach",                           "mach",                         1.0),
    ("trueAirspeed_nmi_h",             "trueAirspeed_m_s",             kNmiH_ms),
]

# ── Plot style ────────────────────────────────────────────────────────────────

SIM_COLOR = "#1D4ED8"   # deep blue
REF_COLOR = "#DC2626"   # red
ERR_COLOR = "#16A34A"   # green
LW_MAIN   = 1.6
LW_ERR    = 1.0
FILL_A    = 0.12
DPI       = 200

plt.rcParams.update({
    "figure.facecolor":  "white",
    "axes.facecolor":    "white",
    "axes.edgecolor":    "#374151",
    "axes.labelcolor":   "#111827",
    "axes.grid":         True,
    "grid.color":        "#E5E7EB",
    "grid.linestyle":    "--",
    "grid.linewidth":    0.5,
    "xtick.color":       "#374151",
    "ytick.color":       "#374151",
    "xtick.labelsize":   8,
    "ytick.labelsize":   8,
    "text.color":        "#111827",
    "legend.framealpha": 0.92,
    "legend.edgecolor":  "#D1D5DB",
    "legend.fontsize":   8,
    "font.family":       "sans-serif",
    "font.size":         9,
    "axes.titlesize":    9,
    "axes.labelsize":    9,
})

# ── Data loading & conversion ─────────────────────────────────────────────────

def load_sim(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    return df


def load_nasa_si(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(path)
    raw.columns = [c.strip() for c in raw.columns]
    si = pd.DataFrame({"time": raw["time"].values})
    for nasa_col, si_col, scale in NASA_TO_SI:
        if nasa_col in raw.columns:
            si[si_col] = raw[nasa_col].values * scale
    return si


def align(sim: pd.DataFrame, ref: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Align on time: round both to 4 decimal places then inner-join."""
    sim = sim.copy()
    ref = ref.copy()
    sim["time"] = sim["time"].astype(float).round(4)
    ref["time"] = ref["time"].astype(float).round(4)
    merged = pd.merge(sim, ref, on="time", suffixes=("_s", "_r"))
    shared = [c for c in sim.columns if c != "time" and c in ref.columns]
    s = merged[["time"] + [f"{c}_s" for c in shared]].rename(
        columns={f"{c}_s": c for c in shared})
    r = merged[["time"] + [f"{c}_r" for c in shared]].rename(
        columns={f"{c}_r": c for c in shared})
    return s, r


# ── Low-level plot helpers ────────────────────────────────────────────────────

def _fmt_ax(ax: plt.Axes, ylabel: str, unit: str = "") -> None:
    label = f"{ylabel} [{unit}]" if unit else ylabel
    ax.set_ylabel(label, labelpad=4)
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.4g"))


def _signal_error_row(
    ax_sig: plt.Axes,
    ax_err: plt.Axes,
    t: np.ndarray,
    sim: np.ndarray,
    ref: np.ndarray,
    ylabel: str,
    unit: str = "",
    display_scale: float = 1.0,
) -> None:
    s = sim * display_scale
    r = ref * display_scale
    e = s - r

    ax_sig.plot(t, r, color=REF_COLOR, lw=LW_MAIN, label="NASA ref",    zorder=3)
    ax_sig.plot(t, s, color=SIM_COLOR, lw=LW_MAIN, ls="--", label="Aetherion", zorder=4)
    ax_sig.fill_between(t, r, s, alpha=FILL_A, color=SIM_COLOR)
    _fmt_ax(ax_sig, ylabel, unit)
    ax_sig.legend(loc="best")

    ax_err.plot(t, e, color=ERR_COLOR, lw=LW_ERR)
    ax_err.axhline(0, color="#9CA3AF", lw=0.7, ls=":")
    ax_err.fill_between(t, e, alpha=FILL_A, color=ERR_COLOR)
    err_unit = unit if unit else ""
    ax_err.set_ylabel(f"Δ [{err_unit}]" if err_unit else "Δ", labelpad=4, color=ERR_COLOR)
    ax_err.tick_params(axis="y", labelcolor=ERR_COLOR, labelsize=7)
    ax_err.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.3g"))


def multi_panel_figure(
    t: np.ndarray,
    rows: list[tuple[str, str, str, float]],  # (si_col, ylabel, unit, display_scale)
    sim_df: pd.DataFrame,
    ref_df: pd.DataFrame,
    title: str,
    out_name: str,
    fig_height_per_row: float = 2.4,
) -> Path:
    """
    Two-column layout: left = signal, right = error.
    Each channel is one row.
    """
    n = len(rows)
    fig_h = max(n * fig_height_per_row, 3.5)
    fig = plt.figure(figsize=(13, fig_h))
    fig.suptitle(title, fontsize=11, fontweight="bold", y=1.005)

    gs = gridspec.GridSpec(
        n, 2, figure=fig,
        hspace=0.55, wspace=0.38,
        left=0.07, right=0.97, top=0.96, bottom=0.07,
    )

    for row_idx, (col, ylabel, unit, scale) in enumerate(rows):
        ax_sig = fig.add_subplot(gs[row_idx, 0])
        ax_err = fig.add_subplot(gs[row_idx, 1])

        if col not in sim_df.columns or col not in ref_df.columns:
            ax_sig.text(0.5, 0.5, f"{col}\n(not available)",
                        ha="center", va="center", transform=ax_sig.transAxes,
                        color="#9CA3AF")
            ax_err.set_visible(False)
            continue

        _signal_error_row(
            ax_sig, ax_err,
            t,
            sim_df[col].values.astype(float),
            ref_df[col].values.astype(float),
            ylabel, unit, scale,
        )

        # Only label x-axis on bottom row
        if row_idx < n - 1:
            ax_sig.set_xticklabels([])
            ax_err.set_xticklabels([])
        else:
            ax_sig.set_xlabel("Time [s]")
            ax_err.set_xlabel("Time [s]")

        # Column headers (top row only)
        if row_idx == 0:
            ax_sig.set_title("Signal", fontsize=8, color="#6B7280", pad=3)
            ax_err.set_title("Error  (Aetherion − NASA)", fontsize=8, color="#6B7280", pad=3)

    out_path = OUT_DIR / out_name
    fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    return out_path


# ── Overview dashboard ────────────────────────────────────────────────────────

def overview_dashboard(
    t: np.ndarray,
    sim_df: pd.DataFrame,
    ref_df: pd.DataFrame,
    out_name: str = "fig_overview.png",
) -> Path:
    cols = [c for c in sim_df.columns if c != "time" and c in ref_df.columns
            and pd.api.types.is_numeric_dtype(sim_df[c])]

    ncols = 4
    nrows = int(np.ceil(len(cols) / ncols))
    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(ncols * 3.8, nrows * 2.6),
                             squeeze=False)
    fig.suptitle(
        "Aetherion F-16 Case 11 — Full Channel Comparison (sim vs. NASA Atmos_11_sim_02)",
        fontsize=12, fontweight="bold", y=1.01,
    )

    for idx, col in enumerate(cols):
        r, c = divmod(idx, ncols)
        ax = axes[r][c]
        s_v = sim_df[col].values.astype(float)
        r_v = ref_df[col].values.astype(float)
        ax.plot(t, r_v, color=REF_COLOR, lw=1.0, label="NASA")
        ax.plot(t, s_v, color=SIM_COLOR, lw=1.0, ls="--", label="Sim")
        ax.set_title(col.replace("_", " "), fontsize=6.5, pad=2)
        ax.set_xlabel("t [s]", fontsize=6)
        ax.tick_params(labelsize=6)
        ax.legend(fontsize=5, loc="best")

    for idx in range(len(cols), nrows * ncols):
        r, c = divmod(idx, ncols)
        axes[r][c].set_visible(False)

    fig.tight_layout()
    out_path = OUT_DIR / out_name
    fig.savefig(out_path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    return out_path


# ── Error summary CSV ─────────────────────────────────────────────────────────

def save_error_summary(
    t: np.ndarray,
    sim_df: pd.DataFrame,
    ref_df: pd.DataFrame,
) -> Path:
    cols = [c for c in sim_df.columns if c != "time" and c in ref_df.columns
            and pd.api.types.is_numeric_dtype(sim_df[c])]
    rows = []
    for col in cols:
        s = sim_df[col].values.astype(float)
        r = ref_df[col].values.astype(float)
        e = s - r
        with np.errstate(divide="ignore", invalid="ignore"):
            rel = np.where(np.abs(r) > 1e-12, e / np.abs(r) * 100.0, np.nan)
        rows.append({
            "column":       col,
            "abs_mean":     float(np.mean(e)),
            "abs_std":      float(np.std(e)),
            "abs_max":      float(np.max(np.abs(e))),
            "rel_mean_%":   float(np.nanmean(rel)),
            "rel_max_%":    float(np.nanmax(np.abs(rel[np.isfinite(rel)]))
                                  if np.any(np.isfinite(rel)) else np.nan),
        })
    out = OUT_DIR / "error_summary.csv"
    pd.DataFrame(rows).to_csv(out, index=False, float_format="%.6g")
    return out


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    print(f"Loading sim  : {SIM_CSV}")
    print(f"Loading NASA : {NASA_CSV}")

    if not SIM_CSV.exists():
        sys.exit(f"ERROR: simulation CSV not found: {SIM_CSV}")
    if not NASA_CSV.exists():
        sys.exit(f"ERROR: NASA reference CSV not found: {NASA_CSV}")

    sim_raw = load_sim(SIM_CSV)
    ref_si  = load_nasa_si(NASA_CSV)

    sim_df, ref_df = align(sim_raw, ref_si)
    t = sim_df["time"].values.astype(float)

    print(f"Aligned time steps: {len(t)}  ({t[0]:.1f} - {t[-1]:.1f} s)")
    print(f"Output -> {OUT_DIR}\n")

    # ── 1. Flight envelope ────────────────────────────────────────────────────
    out = multi_panel_figure(t, [
        ("altitudeMsl_m",    "Altitude MSL",        "m",   1.0),
        ("trueAirspeed_m_s", "True airspeed",        "m/s", 1.0),
        ("mach",             "Mach number",          "—",   1.0),
    ], sim_df, ref_df,
    "Case 11 — Flight Envelope", "fig_flight_envelope.png")
    print(f"  {out.name}")

    # ── 2. Attitude (displayed in degrees) ────────────────────────────────────
    r2d = 180.0 / np.pi
    out = multi_panel_figure(t, [
        ("eulerAngle_rad_Pitch", "Pitch angle θ", "deg", r2d),
        ("eulerAngle_rad_Roll",  "Roll angle φ",  "deg", r2d),
        ("eulerAngle_rad_Yaw",   "Yaw angle ψ",   "deg", r2d),
    ], sim_df, ref_df,
    "Case 11 — Euler Attitude Angles", "fig_attitude.png")
    print(f"  {out.name}")

    # ── 3. Body angular rates (deg/s) ────────────────────────────────────────
    out = multi_panel_figure(t, [
        ("bodyAngularRateWrtEi_rad_s_Roll",  "Roll rate  p", "deg/s", r2d),
        ("bodyAngularRateWrtEi_rad_s_Pitch", "Pitch rate q", "deg/s", r2d),
        ("bodyAngularRateWrtEi_rad_s_Yaw",   "Yaw rate  r",  "deg/s", r2d),
    ], sim_df, ref_df,
    "Case 11 — Body Angular Rates", "fig_body_rates.png")
    print(f"  {out.name}")

    # ── 4. Position (lat/lon in deg, altitude in m) ───────────────────────────
    out = multi_panel_figure(t, [
        ("latitude_rad",  "Latitude",  "deg", r2d),
        ("longitude_rad", "Longitude", "deg", r2d),
        ("altitudeMsl_m", "Altitude",  "m",   1.0),
    ], sim_df, ref_df,
    "Case 11 — Geographic Position", "fig_position.png")
    print(f"  {out.name}")

    # ── 5. NED velocity components ────────────────────────────────────────────
    out = multi_panel_figure(t, [
        ("feVelocity_m_s_X", "v_N (North)", "m/s", 1.0),
        ("feVelocity_m_s_Y", "v_E (East)",  "m/s", 1.0),
        ("feVelocity_m_s_Z", "v_D (Down)",  "m/s", 1.0),
    ], sim_df, ref_df,
    "Case 11 — Earth-Relative Velocity (NED)", "fig_ned_velocity.png")
    print(f"  {out.name}")

    # ── 6. Aerodynamic forces ─────────────────────────────────────────────────
    out = multi_panel_figure(t, [
        ("aero_bodyForce_N_X", "Aero force X (axial)",  "N", 1.0),
        ("aero_bodyForce_N_Y", "Aero force Y (lateral)", "N", 1.0),
        ("aero_bodyForce_N_Z", "Aero force Z (normal)",  "N", 1.0),
    ], sim_df, ref_df,
    "Case 11 — Aerodynamic Body Forces", "fig_aero_forces.png")
    print(f"  {out.name}")

    # ── 7. Aerodynamic moments ────────────────────────────────────────────────
    out = multi_panel_figure(t, [
        ("aero_bodyMoment_Nm_L", "Roll moment  L", "N·m", 1.0),
        ("aero_bodyMoment_Nm_M", "Pitch moment M", "N·m", 1.0),
        ("aero_bodyMoment_Nm_N", "Yaw moment  N",  "N·m", 1.0),
    ], sim_df, ref_df,
    "Case 11 — Aerodynamic Body Moments", "fig_aero_moments.png")
    print(f"  {out.name}")

    # ── 8. Atmosphere ─────────────────────────────────────────────────────────
    out = multi_panel_figure(t, [
        ("speedOfSound_m_s",    "Speed of sound a", "m/s", 1.0),
        ("airDensity_kg_m3",    "Air density ρ",     "kg/m³", 1.0),
        ("ambientPressure_Pa",  "Static pressure p", "Pa",  1.0),
        ("ambientTemperature_K","Temperature T",     "K",   1.0),
    ], sim_df, ref_df,
    "Case 11 — Atmospheric State", "fig_atmosphere.png",
    fig_height_per_row=2.1)
    print(f"  {out.name}")

    # ── 9. Overview dashboard ─────────────────────────────────────────────────
    out = overview_dashboard(t, sim_df, ref_df)
    print(f"  {out.name}")

    # ── 10. Error summary CSV ─────────────────────────────────────────────────
    out = save_error_summary(t, sim_df, ref_df)
    print(f"  {out.name}")

    print("\nDone.")


if __name__ == "__main__":
    main()
