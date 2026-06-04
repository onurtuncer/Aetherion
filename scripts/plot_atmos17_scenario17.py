"""
plot_atmos17_scenario17.py
Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University

Generate comparison plots for NASA TM-2015-218675 Scenario 17
(Two-Stage Rocket to Orbit) — Aetherion vs. NASA Sim 06.

The NASA reference CSV uses US-customary units; this script converts
everything to SI before plotting.

Usage:
    python plot_atmos17_scenario17.py \\
        --sim    path/to/twostage_sim_output.csv \\
        --ref    path/to/Atmos_17_sim_06.csv     \\
        --output doc/_static/atmos17
"""

import argparse, os, math, warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec

warnings.filterwarnings("ignore")

# ── Unit conversion constants ────────────────────────────────────────────────
FT2M        = 0.3048          # ft → m
DEG2RAD     = math.pi / 180.0 # deg → rad
KNOT2MS     = 0.514444        # nmi/h → m/s
SLUG_FT3_TO_KG_M3 = 515.3788 # slug/ft³ → kg/m³
LBF_FT2_TO_PA     = 47.88026 # lbf/ft² → Pa
LBF_TO_N          = 4.448222 # lbf → N
FTLBF_TO_NM       = 1.355818 # ft·lbf → N·m
RANKINE_TO_K      = 5.0 / 9.0
FTMIN_TO_MS       = FT2M / 60.0  # ft/min → m/s

# ── Style (matches compare_sim_validation.py) ────────────────────────────────
SIM_COLOR  = "#2563EB"
VAL_COLOR  = "#DC2626"
ERR_COLOR  = "#16A34A"
LW         = 1.6
GRID_ALPHA = 0.25

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white",
    "axes.edgecolor": "#374151", "axes.labelcolor": "#111827",
    "axes.grid": True, "grid.color": "#D1D5DB",
    "grid.linestyle": "--", "grid.linewidth": 0.6,
    "xtick.color": "#374151", "ytick.color": "#374151",
    "text.color": "#111827", "legend.framealpha": 0.9,
    "legend.edgecolor": "#D1D5DB", "font.family": "sans-serif",
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9,
})


# ── Unit conversion: build SI version of NASA Sim 06 ────────────────────────

def convert_sim06_to_si(df: pd.DataFrame) -> pd.DataFrame:
    """Return a new DataFrame with NASA Sim 06 columns in SI units,
    renamed to match Aetherion Snapshot2 column names."""
    si = pd.DataFrame()
    si["time"] = df["time"]

    si["altitudeMsl_m"]       = df["altitudeMsl_ft"]        * FT2M
    si["longitude_rad"]       = df["longitude_deg"]         * DEG2RAD
    si["latitude_rad"]        = df["latitude_deg"]          * DEG2RAD
    si["localGravity_m_s2"]   = df["localGravity_ft_s2"]    * FT2M
    si["eulerAngle_rad_Yaw"]  = df["eulerAngle_deg_Yaw"]    * DEG2RAD
    si["eulerAngle_rad_Pitch"]= df["eulerAngle_deg_Pitch"]  * DEG2RAD
    si["eulerAngle_rad_Roll"] = df["eulerAngle_deg_Roll"]   * DEG2RAD
    si["bodyAngularRateWrtEi_rad_s_Roll"]  = df["bodyAngularRateWrtEi_deg_s_Roll"]  * DEG2RAD
    si["bodyAngularRateWrtEi_rad_s_Pitch"] = df["bodyAngularRateWrtEi_deg_s_Pitch"] * DEG2RAD
    si["bodyAngularRateWrtEi_rad_s_Yaw"]   = df["bodyAngularRateWrtEi_deg_s_Yaw"]   * DEG2RAD
    si["altitudeRateWrtMsl_m_s"] = df["altitudeRateWrtMsl_ft_min"] * FTMIN_TO_MS
    si["speedOfSound_m_s"]    = df["speedOfSound_ft_s"]     * FT2M
    si["airDensity_kg_m3"]    = df["airDensity_slug_ft3"]   * SLUG_FT3_TO_KG_M3
    si["ambientPressure_Pa"]  = df["ambientPressure_lbf_ft2"] * LBF_FT2_TO_PA
    si["ambientTemperature_K"]= df["ambientTemperature_dgR"] * RANKINE_TO_K
    si["aero_bodyForce_N_X"]  = df["aero_bodyForce_lbf_X"]  * LBF_TO_N
    si["aero_bodyForce_N_Y"]  = df["aero_bodyForce_lbf_Y"]  * LBF_TO_N
    si["aero_bodyForce_N_Z"]  = df["aero_bodyForce_lbf_Z"]  * LBF_TO_N
    si["aero_bodyMoment_Nm_L"]= df["aero_bodyMoment_ftlbf_L"] * FTLBF_TO_NM
    si["aero_bodyMoment_Nm_M"]= df["aero_bodyMoment_ftlbf_M"] * FTLBF_TO_NM
    si["aero_bodyMoment_Nm_N"]= df["aero_bodyMoment_ftlbf_N"] * FTLBF_TO_NM
    si["mach"]                = df["mach"]
    si["dynamicPressure_Pa"]  = df["dynamicPressure_lbf_ft2"] * LBF_FT2_TO_PA
    si["trueAirspeed_m_s"]    = df["trueAirspeed_nmi_h"]    * KNOT2MS
    # feVelocity: NASA Sim 06 stores NED-equivalent components
    si["feVelocity_m_s_X"]    = df["feVelocity_ft_s_X"]     * FT2M
    si["feVelocity_m_s_Y"]    = df["feVelocity_ft_s_Y"]     * FT2M
    si["feVelocity_m_s_Z"]    = df["feVelocity_ft_s_Z"]     * FT2M
    return si


# ── Axis labels ──────────────────────────────────────────────────────────────

LABELS = {
    "altitudeMsl_m":              "Altitude MSL [m]",
    "trueAirspeed_m_s":           "True Airspeed [m/s]",
    "mach":                       "Mach Number [−]",
    "dynamicPressure_Pa":         "Dynamic Pressure [Pa]",
    "eulerAngle_rad_Pitch":       "Euler Pitch [rad]",
    "eulerAngle_rad_Yaw":         "Euler Yaw [rad]",
    "eulerAngle_rad_Roll":        "Euler Roll [rad]",
    "feVelocity_m_s_X":           "FE Velocity N [m/s]",
    "feVelocity_m_s_Y":           "FE Velocity E [m/s]",
    "feVelocity_m_s_Z":           "FE Velocity D [m/s]",
    "bodyAngularRateWrtEi_rad_s_Pitch": "Body Pitch Rate (EI) [rad/s]",
    "altitudeRateWrtMsl_m_s":     "Altitude Rate [m/s]",
    "speedOfSound_m_s":           "Speed of Sound [m/s]",
    "airDensity_kg_m3":           "Air Density [kg/m³]",
    "ambientPressure_Pa":         "Ambient Pressure [Pa]",
    "ambientTemperature_K":       "Ambient Temperature [K]",
    "aero_bodyForce_N_X":         "Aero Body Force X [N]",
    "aero_bodyForce_N_Y":         "Aero Body Force Y [N]",
    "aero_bodyForce_N_Z":         "Aero Body Force Z [N]",
    "aero_bodyMoment_Nm_M":       "Aero Pitch Moment [N·m]",
    "localGravity_m_s2":          "Local Gravity [m/s²]",
    "latitude_rad":               "Latitude [rad]",
    "longitude_rad":              "Longitude [rad]",
}


# ── Single-column plot ────────────────────────────────────────────────────────

def plot_column(t, sim, ref, col, out_dir, dpi, fmt, show):
    ylabel = LABELS.get(col, col)
    abs_err = sim - ref

    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()

    l1, = ax1.plot(t, ref, color=VAL_COLOR, lw=LW,   label="NASA Sim 06")
    l2, = ax1.plot(t, sim, color=SIM_COLOR, lw=LW, ls="--", label="Aetherion")
    l3, = ax2.plot(t, abs_err, color=ERR_COLOR, lw=0.9, alpha=0.85,
                   label="Aetherion − Sim 06")
    ax2.axhline(0, color=ERR_COLOR, lw=0.5, ls=":", alpha=0.5)

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel(ylabel)
    ax2.set_ylabel("Absolute error (Aetherion − Sim 06)", color=ERR_COLOR)
    ax2.tick_params(axis="y", labelcolor=ERR_COLOR)

    # stats
    clean = abs_err[np.isfinite(abs_err)]
    stats = (f"max|e| = {np.max(np.abs(clean)):.3g}   "
             f"μ = {np.mean(clean):.3g}   "
             f"σ = {np.std(clean):.3g}") if len(clean) else "N/A"
    fig.suptitle(ylabel, fontsize=11, fontweight="bold", y=1.01)
    ax1.set_title(f"Stats → {stats}", fontsize=8, color="#6B7280", pad=4)

    ax1.legend([l1, l2, l3], [l.get_label() for l in [l1, l2, l3]],
               loc="upper left", fontsize=8)
    fig.tight_layout()

    fname = os.path.join(out_dir, f"{col}.{fmt}")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
    return fname


# ── Overview dashboard ────────────────────────────────────────────────────────

DASHBOARD_COLS = [
    "altitudeMsl_m",
    "trueAirspeed_m_s",
    "mach",
    "dynamicPressure_Pa",
    "eulerAngle_rad_Pitch",
    "feVelocity_m_s_Y",
    "bodyAngularRateWrtEi_rad_s_Pitch",
    "altitudeRateWrtMsl_m_s",
    "speedOfSound_m_s",
    "airDensity_kg_m3",
    "ambientTemperature_K",
    "aero_bodyMoment_Nm_M",
]

def plot_dashboard(t, sim_df, ref_df, cols, out_dir, dpi, fmt, show):
    n = len(cols)
    ncols_grid = 3
    nrows_grid = int(math.ceil(n / ncols_grid))
    fig, axes = plt.subplots(nrows_grid, ncols_grid,
                             figsize=(ncols_grid * 5, nrows_grid * 3.2),
                             squeeze=False)
    fig.suptitle("Aetherion vs. NASA Sim 06 — Scenario 17 Overview",
                 fontsize=13, fontweight="bold", y=1.01)

    for idx, col in enumerate(cols):
        r, c = divmod(idx, ncols_grid)
        ax1 = axes[r][c]
        ax2 = ax1.twinx()

        s = sim_df[col].values.astype(float)
        v = ref_df[col].values.astype(float)
        e = s - v

        ax1.plot(t, v, color=VAL_COLOR, lw=1.2, label="Sim 06")
        ax1.plot(t, s, color=SIM_COLOR, lw=1.2, ls="--", label="AE")
        ax2.plot(t, e, color=ERR_COLOR, lw=0.8, alpha=0.7)
        ax2.axhline(0, color=ERR_COLOR, lw=0.4, ls=":")
        ax2.tick_params(axis="y", labelsize=6, labelcolor=ERR_COLOR)
        ax2.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2g"))

        ax1.set_title(LABELS.get(col, col), fontsize=8, pad=3)
        ax1.set_xlabel("t [s]", fontsize=7)
        ax1.tick_params(labelsize=7)
        ax1.legend(fontsize=6, loc="upper left")

    for idx in range(n, nrows_grid * ncols_grid):
        r, c = divmod(idx, ncols_grid)
        axes[r][c].set_visible(False)

    fig.tight_layout()
    fname = os.path.join(out_dir, f"overview_dashboard.{fmt}")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
    return fname


# ── Mission phases marker helper ──────────────────────────────────────────────

def add_phase_lines(ax):
    """Add vertical markers for S1 burnout and S2 ignition."""
    for t_phase, label, color in [
        (37.4,  "S1 burnout", "#92400E"),
        (131.8, "S2 ignition", "#5B21B6"),
    ]:
        ax.axvline(t_phase, color=color, lw=0.9, ls=":", alpha=0.7)
        ax.text(t_phase + 1, ax.get_ylim()[0], label,
                fontsize=7, color=color, rotation=90, va="bottom", alpha=0.8)


# ── Key-variable focused plots ────────────────────────────────────────────────

def plot_altitude_focused(t, sim_df, ref_df, out_dir, dpi, fmt, show):
    """Altitude comparison with mission phase markers."""
    s = sim_df["altitudeMsl_m"].values / 1000.0   # km
    v = ref_df["altitudeMsl_m"].values / 1000.0
    e = (sim_df["altitudeMsl_m"].values - ref_df["altitudeMsl_m"].values) / 1000.0

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})
    fig.suptitle("Altitude MSL — Aetherion vs. NASA Sim 06", fontsize=12,
                 fontweight="bold")

    ax = axes[0]
    ax.plot(t, v, color=VAL_COLOR, lw=LW,       label="NASA Sim 06")
    ax.plot(t, s, color=SIM_COLOR, lw=LW, ls="--", label="Aetherion (dt=0.001 s)")
    ax.set_ylabel("Altitude MSL [km]")
    ax.legend(fontsize=9)

    # phase lines
    for t_ph, lbl, col in [(37.4, "S1 burnout", "#92400E"),
                            (131.8, "S2 ignition", "#5B21B6")]:
        ax.axvline(t_ph, color=col, lw=1, ls=":", alpha=0.8)
        ax.text(t_ph + 1, ax.get_ylim()[1] * 0.98, lbl,
                fontsize=7, color=col, va="top", alpha=0.9)

    ax2 = axes[1]
    ax2.plot(t, e, color=ERR_COLOR, lw=1.2)
    ax2.axhline(0, color="#374151", lw=0.6)
    ax2.set_ylabel("Error [km]")
    ax2.set_xlabel("Time [s]")
    ax2.fill_between(t, e, 0, alpha=0.15, color=ERR_COLOR)

    clean = e[np.isfinite(e)]
    ax.set_title(f"max|err| = {np.max(np.abs(clean))*1000:.0f} m   "
                 f"final err = {e[-1]*1000:.0f} m   "
                 f"(final = {s[-1]:.1f} km vs {v[-1]:.1f} km)",
                 fontsize=8, color="#6B7280", pad=3)

    fig.tight_layout()
    fname = os.path.join(out_dir, "altitudeMsl_m.png")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show: plt.show()
    plt.close(fig)
    print(f"  saved: {fname}")


def plot_pitch_focused(t, sim_df, ref_df, out_dir, dpi, fmt, show):
    """Pitch angle comparison with phase markers."""
    s_deg = np.degrees(sim_df["eulerAngle_rad_Pitch"].values)
    v_deg = np.degrees(ref_df["eulerAngle_rad_Pitch"].values)
    e_deg = s_deg - v_deg

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})
    fig.suptitle("Euler Pitch Angle — Aetherion vs. NASA Sim 06",
                 fontsize=12, fontweight="bold")

    ax = axes[0]
    ax.plot(t, v_deg, color=VAL_COLOR, lw=LW,       label="NASA Sim 06")
    ax.plot(t, s_deg, color=SIM_COLOR, lw=LW, ls="--", label="Aetherion (dt=0.001 s)")
    ax.set_ylabel("Euler Pitch [°]")
    ax.legend(fontsize=9)

    for t_ph, lbl, col in [(37.4, "S1 burnout", "#92400E"),
                            (131.8, "S2 ignition", "#5B21B6")]:
        ax.axvline(t_ph, color=col, lw=1, ls=":", alpha=0.8)
        ax.text(t_ph + 1, ax.get_ylim()[1] * 0.98, lbl,
                fontsize=7, color=col, va="top", alpha=0.9)

    ax2 = axes[1]
    ax2.plot(t, e_deg, color=ERR_COLOR, lw=1.2)
    ax2.axhline(0, color="#374151", lw=0.6)
    ax2.set_ylabel("Error [°]")
    ax2.set_xlabel("Time [s]")
    ax2.fill_between(t, e_deg, 0, alpha=0.15, color=ERR_COLOR)

    clean = e_deg[np.isfinite(e_deg)]
    ax.set_title(f"final err = {e_deg[-1]:.2f}°   "
                 f"(final = {s_deg[-1]:.2f}° vs {v_deg[-1]:.2f}°)",
                 fontsize=8, color="#6B7280", pad=3)

    fig.tight_layout()
    fname = os.path.join(out_dir, "eulerAngle_rad_Pitch.png")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show: plt.show()
    plt.close(fig)
    print(f"  saved: {fname}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Scenario 17 comparison plots: Aetherion vs. NASA Sim 06.")
    parser.add_argument("--sim",    default="twostage_sim_output.csv",
                        help="Aetherion output CSV (Snapshot2, SI units)")
    parser.add_argument("--ref",    default="Atmos_17_sim_06.csv",
                        help="NASA Sim 06 reference CSV")
    parser.add_argument("--output", default="comparison_output",
                        help="Output directory for plots")
    parser.add_argument("--dpi",    type=int, default=150)
    parser.add_argument("--fmt",    default="png",
                        choices=["png", "pdf", "svg"])
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    show = not args.no_show
    os.makedirs(args.output, exist_ok=True)

    print(f"Loading Aetherion: {args.sim}")
    sim_df = pd.read_csv(args.sim)

    print(f"Loading NASA Sim06: {args.ref}")
    ref_raw = pd.read_csv(args.ref)
    ref_df  = convert_sim06_to_si(ref_raw)

    # Align on common time (inner join, both at 0.1 s cadence)
    t_sim = sim_df["time"].values
    t_ref = ref_df["time"].values

    # Interpolate reference to sim time grid to handle any rounding
    ref_interp = pd.DataFrame({"time": t_sim})
    for col in ref_df.columns:
        if col == "time": continue
        ref_interp[col] = np.interp(t_sim, t_ref,
                                    ref_df[col].values,
                                    left=np.nan, right=np.nan)

    # Common columns
    common = [c for c in sim_df.columns
              if c != "time" and c in ref_interp.columns]
    print(f"  {len(common)} shared columns over {len(t_sim)} time steps\n")

    # ── Focused high-quality plots ────────────────────────────────────────────
    plot_altitude_focused(t_sim, sim_df, ref_interp, args.output,
                          args.dpi, args.fmt, show)
    plot_pitch_focused(t_sim, sim_df, ref_interp, args.output,
                       args.dpi, args.fmt, show)

    # ── Standard per-column plots ─────────────────────────────────────────────
    all_plot_cols = [
        "altitudeMsl_m",
        "trueAirspeed_m_s",
        "mach",
        "dynamicPressure_Pa",
        "eulerAngle_rad_Yaw",
        "eulerAngle_rad_Roll",
        "feVelocity_m_s_X",
        "feVelocity_m_s_Y",
        "feVelocity_m_s_Z",
        "bodyAngularRateWrtEi_rad_s_Roll",
        "bodyAngularRateWrtEi_rad_s_Pitch",
        "bodyAngularRateWrtEi_rad_s_Yaw",
        "altitudeRateWrtMsl_m_s",
        "speedOfSound_m_s",
        "airDensity_kg_m3",
        "ambientPressure_Pa",
        "ambientTemperature_K",
        "aero_bodyForce_N_X",
        "aero_bodyForce_N_Y",
        "aero_bodyForce_N_Z",
        "aero_bodyMoment_Nm_L",
        "aero_bodyMoment_Nm_M",
        "aero_bodyMoment_Nm_N",
        "localGravity_m_s2",
        "latitude_rad",
        "longitude_rad",
    ]
    # Only plot columns present in both
    plot_cols = [c for c in all_plot_cols if c in common]

    for col in plot_cols:
        if col in ("altitudeMsl_m", "eulerAngle_rad_Pitch"):
            continue  # already done with focused version
        s = sim_df[col].values.astype(float)
        v = ref_interp[col].values.astype(float)
        fname = plot_column(t_sim, s, v, col,
                            args.output, args.dpi, args.fmt, show)
        print(f"  saved: {fname}")

    # ── Overview dashboard ────────────────────────────────────────────────────
    dash_cols = [c for c in DASHBOARD_COLS if c in common]
    fname = plot_dashboard(t_sim, sim_df, ref_interp, dash_cols,
                           args.output, args.dpi, args.fmt, show)
    print(f"  saved: {fname}")

    print("\nDone.")


if __name__ == "__main__":
    main()
