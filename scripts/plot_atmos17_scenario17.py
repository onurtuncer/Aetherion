"""
plot_atmos17_scenario17.py
Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University

Generate comparison plots for NASA TM-2015-218675 Scenario 17
(Two-Stage Rocket to Orbit) — Aetherion vs. NASA Sim 06.

Usage:
    python plot_atmos17_scenario17.py \\
        --sim    path/to/twostage_sim_output.csv \\
        --ref    path/to/Atmos_17_sim_06.csv     \\
        --output doc/_static/atmos17
"""

import argparse, os, math, warnings
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.cm as cm
import matplotlib.colors as mcolors

warnings.filterwarnings("ignore")

# ── Unit conversions ─────────────────────────────────────────────────────────
FT2M             = 0.3048
DEG2RAD          = math.pi / 180.0
KNOT2MS          = 0.514444
SLUG_FT3_TO_KG_M3 = 515.3788
LBF_FT2_TO_PA    = 47.88026
LBF_TO_N         = 4.448222
FTLBF_TO_NM      = 1.355818
RANKINE_TO_K     = 5.0 / 9.0
FTMIN_TO_MS      = FT2M / 60.0
R_EARTH_KM       = 6378.137

# ── Mission events ────────────────────────────────────────────────────────────
EVENTS = [
    (37.4,  "S1 burnout",   "#92400E"),   # amber
    (131.8, "S2 ignition",  "#5B21B6"),   # violet
    (193.0, "S2 burnout",   "#065F46"),   # teal
]

# ── Colours & style ───────────────────────────────────────────────────────────
SIM_COLOR = "#2563EB"
VAL_COLOR = "#DC2626"
ERR_COLOR = "#16A34A"
LW        = 1.6

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

# ── Axis labels ───────────────────────────────────────────────────────────────
LABELS = {
    "altitudeMsl_m":                      "Altitude MSL [m]",
    "trueAirspeed_m_s":                   "True Airspeed [m/s]",
    "mach":                               "Mach Number [−]",
    "dynamicPressure_Pa":                 "Dynamic Pressure [Pa]",
    "eulerAngle_rad_Pitch":               "Euler Pitch [rad]",
    "eulerAngle_rad_Yaw":                 "Euler Yaw [rad]",
    "eulerAngle_rad_Roll":                "Euler Roll [rad]",
    "feVelocity_m_s_X":                   "FE Velocity N [m/s]",
    "feVelocity_m_s_Y":                   "FE Velocity E [m/s]",
    "feVelocity_m_s_Z":                   "FE Velocity D [m/s]",
    "bodyAngularRateWrtEi_rad_s_Pitch":   "Body Pitch Rate (EI) [rad/s]",
    "bodyAngularRateWrtEi_rad_s_Roll":    "Body Roll Rate (EI) [rad/s]",
    "bodyAngularRateWrtEi_rad_s_Yaw":     "Body Yaw Rate (EI) [rad/s]",
    "altitudeRateWrtMsl_m_s":             "Altitude Rate [m/s]",
    "speedOfSound_m_s":                   "Speed of Sound [m/s]",
    "airDensity_kg_m3":                   "Air Density [kg/m³]",
    "ambientPressure_Pa":                 "Ambient Pressure [Pa]",
    "ambientTemperature_K":               "Ambient Temperature [K]",
    "aero_bodyForce_N_X":                 "Aero Body Force X [N]",
    "aero_bodyForce_N_Y":                 "Aero Body Force Y [N]",
    "aero_bodyForce_N_Z":                 "Aero Body Force Z [N]",
    "aero_bodyMoment_Nm_L":               "Aero Roll Moment [N·m]",
    "aero_bodyMoment_Nm_M":               "Aero Pitch Moment [N·m]",
    "aero_bodyMoment_Nm_N":               "Aero Yaw Moment [N·m]",
    "localGravity_m_s2":                  "Local Gravity [m/s²]",
    "latitude_rad":                       "Latitude [rad]",
    "longitude_rad":                      "Longitude [rad]",
}


# ── Unit conversion: NASA Sim 06 → SI ────────────────────────────────────────
def convert_sim06_to_si(df: pd.DataFrame) -> pd.DataFrame:
    si = pd.DataFrame()
    si["time"]                              = df["time"]
    si["altitudeMsl_m"]                     = df["altitudeMsl_ft"]          * FT2M
    si["longitude_rad"]                     = df["longitude_deg"]           * DEG2RAD
    si["latitude_rad"]                      = df["latitude_deg"]            * DEG2RAD
    si["localGravity_m_s2"]                 = df["localGravity_ft_s2"]      * FT2M
    si["eulerAngle_rad_Yaw"]                = df["eulerAngle_deg_Yaw"]      * DEG2RAD
    si["eulerAngle_rad_Pitch"]              = df["eulerAngle_deg_Pitch"]    * DEG2RAD
    si["eulerAngle_rad_Roll"]               = df["eulerAngle_deg_Roll"]     * DEG2RAD
    si["bodyAngularRateWrtEi_rad_s_Roll"]   = df["bodyAngularRateWrtEi_deg_s_Roll"]  * DEG2RAD
    si["bodyAngularRateWrtEi_rad_s_Pitch"]  = df["bodyAngularRateWrtEi_deg_s_Pitch"] * DEG2RAD
    si["bodyAngularRateWrtEi_rad_s_Yaw"]    = df["bodyAngularRateWrtEi_deg_s_Yaw"]   * DEG2RAD
    si["altitudeRateWrtMsl_m_s"]            = df["altitudeRateWrtMsl_ft_min"] * FTMIN_TO_MS
    si["speedOfSound_m_s"]                  = df["speedOfSound_ft_s"]       * FT2M
    si["airDensity_kg_m3"]                  = df["airDensity_slug_ft3"]     * SLUG_FT3_TO_KG_M3
    si["ambientPressure_Pa"]                = df["ambientPressure_lbf_ft2"] * LBF_FT2_TO_PA
    si["ambientTemperature_K"]              = df["ambientTemperature_dgR"]  * RANKINE_TO_K
    si["aero_bodyForce_N_X"]                = df["aero_bodyForce_lbf_X"]   * LBF_TO_N
    si["aero_bodyForce_N_Y"]                = df["aero_bodyForce_lbf_Y"]   * LBF_TO_N
    si["aero_bodyForce_N_Z"]                = df["aero_bodyForce_lbf_Z"]   * LBF_TO_N
    si["aero_bodyMoment_Nm_L"]              = df["aero_bodyMoment_ftlbf_L"] * FTLBF_TO_NM
    si["aero_bodyMoment_Nm_M"]              = df["aero_bodyMoment_ftlbf_M"] * FTLBF_TO_NM
    si["aero_bodyMoment_Nm_N"]              = df["aero_bodyMoment_ftlbf_N"] * FTLBF_TO_NM
    si["mach"]                              = df["mach"]
    si["dynamicPressure_Pa"]                = df["dynamicPressure_lbf_ft2"] * LBF_FT2_TO_PA
    si["trueAirspeed_m_s"]                  = df["trueAirspeed_nmi_h"]      * KNOT2MS
    si["feVelocity_m_s_X"]                  = df["feVelocity_ft_s_X"]       * FT2M
    si["feVelocity_m_s_Y"]                  = df["feVelocity_ft_s_Y"]       * FT2M
    si["feVelocity_m_s_Z"]                  = df["feVelocity_ft_s_Z"]       * FT2M
    return si


# ── Event-line helper ─────────────────────────────────────────────────────────
def draw_event_lines(ax, label_side="top"):
    """Draw vertical dotted lines and labels for mission events.
    Uses axes-fraction coordinates for the text y-position so it works
    regardless of whether the axis limits have been finalised yet.
    """
    # blended transform: x in data coords, y in axes fraction (0–1)
    trans = ax.get_xaxis_transform()
    y_frac = 0.97 if label_side == "top" else 0.03
    va     = "top"    if label_side == "top" else "bottom"
    for t_ev, lbl, col in EVENTS:
        ax.axvline(t_ev, color=col, lw=1.1, ls=":", alpha=0.85, zorder=5)
        ax.text(t_ev + 1.5, y_frac, lbl,
                transform=trans,
                fontsize=7, color=col, rotation=90, va=va,
                alpha=0.9, zorder=6)


# ── Standard per-column comparison plot ──────────────────────────────────────
def plot_column(t, sim, ref, col, out_dir, dpi, fmt, show):
    ylabel  = LABELS.get(col, col)
    abs_err = sim - ref

    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()

    l1, = ax1.plot(t, ref, color=VAL_COLOR, lw=LW,         label="NASA Sim 06")
    l2, = ax1.plot(t, sim, color=SIM_COLOR, lw=LW, ls="--", label="Aetherion")
    l3, = ax2.plot(t, abs_err, color=ERR_COLOR, lw=0.9, alpha=0.85,
                   label="Aetherion − Sim 06")
    ax2.axhline(0, color=ERR_COLOR, lw=0.5, ls=":", alpha=0.5)

    draw_event_lines(ax1)

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel(ylabel)
    ax2.set_ylabel("Absolute error (Aetherion − Sim 06)", color=ERR_COLOR)
    ax2.tick_params(axis="y", labelcolor=ERR_COLOR)

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
    if show: plt.show()
    plt.close(fig)
    return fname


# ── Overview dashboard ────────────────────────────────────────────────────────
DASHBOARD_COLS = [
    "altitudeMsl_m", "trueAirspeed_m_s", "mach", "dynamicPressure_Pa",
    "eulerAngle_rad_Pitch", "feVelocity_m_s_Y",
    "bodyAngularRateWrtEi_rad_s_Pitch", "altitudeRateWrtMsl_m_s",
    "speedOfSound_m_s", "airDensity_kg_m3", "ambientTemperature_K",
    "aero_bodyMoment_Nm_M",
]

def plot_dashboard(t, sim_df, ref_df, cols, out_dir, dpi, fmt, show):
    n = len(cols)
    nc, nr = 3, int(math.ceil(n / 3))
    fig, axes = plt.subplots(nr, nc, figsize=(nc * 5, nr * 3.2), squeeze=False)
    fig.suptitle("Aetherion vs. NASA Sim 06 — Scenario 17 Overview",
                 fontsize=13, fontweight="bold", y=1.01)

    for idx, col in enumerate(cols):
        r, c = divmod(idx, nc)
        ax1 = axes[r][c]; ax2 = ax1.twinx()
        s = sim_df[col].values.astype(float)
        v = ref_df[col].values.astype(float)
        e = s - v
        ax1.plot(t, v, color=VAL_COLOR, lw=1.2, label="Sim 06")
        ax1.plot(t, s, color=SIM_COLOR, lw=1.2, ls="--", label="AE")
        ax2.plot(t, e, color=ERR_COLOR, lw=0.8, alpha=0.7)
        ax2.axhline(0, color=ERR_COLOR, lw=0.4, ls=":")
        ax2.tick_params(axis="y", labelsize=6, labelcolor=ERR_COLOR)
        ax2.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2g"))
        draw_event_lines(ax1)
        ax1.set_title(LABELS.get(col, col), fontsize=8, pad=3)
        ax1.set_xlabel("t [s]", fontsize=7)
        ax1.tick_params(labelsize=7)
        ax1.legend(fontsize=6, loc="upper left")

    for idx in range(n, nr * nc):
        r, c = divmod(idx, nc)
        axes[r][c].set_visible(False)

    fig.tight_layout()
    fname = os.path.join(out_dir, f"overview_dashboard.{fmt}")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show: plt.show()
    plt.close(fig)
    return fname


# ── Focused altitude plot ─────────────────────────────────────────────────────
def plot_altitude_focused(t, sim_df, ref_df, out_dir, dpi, fmt, show):
    s = sim_df["altitudeMsl_m"].values / 1000.0
    v = ref_df["altitudeMsl_m"].values / 1000.0
    e = s - v

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})
    fig.suptitle("Altitude MSL — Aetherion vs. NASA Sim 06",
                 fontsize=12, fontweight="bold")

    ax = axes[0]
    ax.plot(t, v, color=VAL_COLOR, lw=LW,         label="NASA Sim 06")
    ax.plot(t, s, color=SIM_COLOR, lw=LW, ls="--", label="Aetherion (dt=0.001 s)")
    ax.set_ylabel("Altitude MSL [km]")
    ax.legend(fontsize=9)
    draw_event_lines(ax)

    ax2 = axes[1]
    ax2.plot(t, e, color=ERR_COLOR, lw=1.2)
    ax2.axhline(0, color="#374151", lw=0.6)
    ax2.fill_between(t, e, 0, alpha=0.15, color=ERR_COLOR)
    ax2.set_ylabel("Error [km]")
    ax2.set_xlabel("Time [s]")
    draw_event_lines(ax2, label_side="bottom")

    clean = e[np.isfinite(e)]
    ax.set_title(f"max|err| = {np.max(np.abs(clean))*1000:.0f} m   "
                 f"final err = {e[-1]*1000:.0f} m   "
                 f"(final {s[-1]:.1f} km vs {v[-1]:.1f} km)",
                 fontsize=8, color="#6B7280", pad=3)

    fig.tight_layout()
    fname = os.path.join(out_dir, "altitudeMsl_m.png")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show: plt.show()
    plt.close(fig)
    print(f"  saved: {fname}")


# ── Focused pitch plot ────────────────────────────────────────────────────────
def plot_pitch_focused(t, sim_df, ref_df, out_dir, dpi, fmt, show):
    s_deg = np.degrees(sim_df["eulerAngle_rad_Pitch"].values)
    v_deg = np.degrees(ref_df["eulerAngle_rad_Pitch"].values)
    e_deg = s_deg - v_deg

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})
    fig.suptitle("Euler Pitch Angle — Aetherion vs. NASA Sim 06",
                 fontsize=12, fontweight="bold")

    ax = axes[0]
    ax.plot(t, v_deg, color=VAL_COLOR, lw=LW,         label="NASA Sim 06")
    ax.plot(t, s_deg, color=SIM_COLOR, lw=LW, ls="--", label="Aetherion (dt=0.001 s)")
    ax.set_ylabel("Euler Pitch [°]")
    ax.legend(fontsize=9)
    draw_event_lines(ax)

    ax2 = axes[1]
    ax2.plot(t, e_deg, color=ERR_COLOR, lw=1.2)
    ax2.axhline(0, color="#374151", lw=0.6)
    ax2.fill_between(t, e_deg, 0, alpha=0.15, color=ERR_COLOR)
    ax2.set_ylabel("Error [°]")
    ax2.set_xlabel("Time [s]")
    draw_event_lines(ax2, label_side="bottom")

    ax.set_title(f"final err = {e_deg[-1]:.2f}°   "
                 f"(final {s_deg[-1]:.2f}° vs {v_deg[-1]:.2f}°)",
                 fontsize=8, color="#6B7280", pad=3)

    fig.tight_layout()
    fname = os.path.join(out_dir, "eulerAngle_rad_Pitch.png")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show: plt.show()
    plt.close(fig)
    print(f"  saved: {fname}")


# ── 3D Trajectory plot ────────────────────────────────────────────────────────
# Phase colours for the trajectory
PHASE_COLORS = {
    "S1 burn":  "#DC2626",   # red
    "Coast":    "#2563EB",   # blue
    "S2 burn":  "#7C3AED",   # purple
    "Coast 2":  "#059669",   # green
}

def _traj_to_enu(lon_rad, lat_rad, alt_m):
    """Convert geodetic to local East-North-Up (km) relative to launch site."""
    Re = R_EARTH_KM * 1000.0   # m
    lon0 = lon_rad[0]
    lat0 = lat_rad[0]
    dlat = lat_rad - lat0
    dlon = lon_rad - lon0
    E = (Re + alt_m) * np.cos(lat0) * dlon / 1000.0   # km East
    N = (Re + alt_m) * dlat / 1000.0                   # km North
    U = alt_m / 1000.0                                  # km Up
    return E, N, U


def _phase_mask(t):
    """Return array of phase indices: 0=S1 burn, 1=coast, 2=S2 burn, 3=coast2."""
    p = np.zeros(len(t), dtype=int)
    p[t > EVENTS[0][0]]  = 1   # after S1 burnout → coast
    p[t > EVENTS[1][0]]  = 2   # after S2 ignition → S2 burn
    p[t > EVENTS[2][0]]  = 3   # after S2 burnout → final coast
    return p


def plot_trajectory_3d(t, sim_df, ref_df, out_dir, dpi, fmt, show):
    """3D trajectory: East (km) vs Up/Altitude (km), with North ~0."""

    # Aetherion geodetic coords
    lon_ae  = sim_df["longitude_rad"].values
    lat_ae  = sim_df["latitude_rad"].values
    alt_ae  = sim_df["altitudeMsl_m"].values
    E_ae, N_ae, U_ae = _traj_to_enu(lon_ae, lat_ae, alt_ae)

    # NASA Sim 06 geodetic coords
    lon_ref = ref_df["longitude_rad"].values
    lat_ref = ref_df["latitude_rad"].values
    alt_ref = ref_df["altitudeMsl_m"].values
    E_ref, N_ref, U_ref = _traj_to_enu(lon_ref, lat_ref, alt_ref)

    phase = _phase_mask(t)
    phase_names = ["S1 burn (0–37.4 s)", "Coast (37.4–131.8 s)",
                   "S2 burn (131.8–193 s)", "Final coast (193–200 s)"]
    phase_cols  = ["#DC2626", "#2563EB", "#7C3AED", "#059669"]

    fig = plt.figure(figsize=(14, 9))
    ax  = fig.add_subplot(111, projection="3d")

    # ── Draw Earth surface arc ─────────────────────────────────────────────
    # Flat ground plane at U=0, coloured light green/blue
    E_max = max(E_ae.max(), E_ref.max()) * 1.05
    E_gr  = np.linspace(0, E_max, 40)
    N_gr  = np.linspace(-5, 5, 10)
    EG, NG = np.meshgrid(E_gr, N_gr)
    UG = np.zeros_like(EG)
    ax.plot_surface(EG, NG, UG, alpha=0.12, color="#3B82F6", linewidth=0)

    # ── Aetherion trajectory — phase-coloured segments ─────────────────────
    for ph in range(4):
        mask = phase == ph
        if mask.sum() < 2: continue
        ax.plot(E_ae[mask], N_ae[mask], U_ae[mask],
                color=phase_cols[ph], lw=2.2, label=phase_names[ph], zorder=5)

    # ── NASA Sim 06 trajectory (single dashed grey line) ───────────────────
    ax.plot(E_ref, N_ref, U_ref,
            color="#6B7280", lw=1.4, ls="--", alpha=0.8,
            label="NASA Sim 06", zorder=4)

    # ── Event markers ──────────────────────────────────────────────────────
    marker_styles = {0: "S1 burnout", 1: "S2 ignition", 2: "S2 burnout"}
    ev_cols = ["#92400E", "#5B21B6", "#065F46"]
    for idx, (t_ev, lbl, col) in enumerate(EVENTS):
        i = np.searchsorted(t, t_ev)
        if i >= len(E_ae): i = len(E_ae) - 1
        # Aetherion marker
        ax.scatter(E_ae[i], N_ae[i], U_ae[i],
                   color=col, s=60, zorder=8, depthshade=False)
        ax.text(E_ae[i] + 15, N_ae[i], U_ae[i] + 8, lbl,
                fontsize=8, color=col, fontweight="bold")
        # vertical drop line to ground
        ax.plot([E_ae[i], E_ae[i]], [N_ae[i], N_ae[i]], [0, U_ae[i]],
                color=col, lw=0.8, ls=":", alpha=0.5)

    # ── Launch marker ──────────────────────────────────────────────────────
    ax.scatter([0], [0], [0], color="#1E293B", s=80, marker="^",
               zorder=9, depthshade=False, label="Launch site (0°N, 0°E)")
    ax.text(5, 0, 2, "Launch", fontsize=8, color="#1E293B", fontweight="bold")

    # ── Final position markers ─────────────────────────────────────────────
    ax.scatter([E_ae[-1]], [N_ae[-1]], [U_ae[-1]],
               color=SIM_COLOR, s=80, marker="*",
               zorder=9, depthshade=False)
    ax.scatter([E_ref[-1]], [N_ref[-1]], [U_ref[-1]],
               color=VAL_COLOR, s=60, marker="*",
               zorder=9, depthshade=False)

    # ── Axes & labels ──────────────────────────────────────────────────────
    ax.set_xlabel("East distance [km]",   labelpad=10)
    ax.set_ylabel("North offset [km]",    labelpad=10)
    ax.set_zlabel("Altitude [km]",        labelpad=10)
    ax.set_title("Two-Stage Rocket Trajectory — Aetherion vs. NASA Sim 06\n"
                 "NASA TM-2015-218675 Atmospheric Scenario 17",
                 fontsize=12, fontweight="bold", pad=15)

    ax.legend(loc="upper left", fontsize=8, framealpha=0.85)
    ax.view_init(elev=22, azim=-60)

    # ── Annotation box ─────────────────────────────────────────────────────
    note = (f"Aetherion: alt={alt_ae[-1]/1000:.1f} km, TAS={sim_df['trueAirspeed_m_s'].values[-1]:.0f} m/s\n"
            f"NASA Sim 06: alt={alt_ref[-1]/1000:.1f} km, TAS={ref_df['trueAirspeed_m_s'].values[-1]:.0f} m/s\n"
            f"Error: {(alt_ae[-1]-alt_ref[-1])/alt_ref[-1]*100:.2f}%  (dt = 0.001 s, Radau IIA RKMK)")
    fig.text(0.02, 0.02, note, fontsize=8, color="#374151",
             bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                       edgecolor="#D1D5DB", alpha=0.9))

    fig.tight_layout()
    fname = os.path.join(out_dir, "trajectory_3d.png")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show: plt.show()
    plt.close(fig)
    print(f"  saved: {fname}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Scenario 17 comparison plots: Aetherion vs. NASA Sim 06.")
    parser.add_argument("--sim",     default="twostage_sim_output.csv")
    parser.add_argument("--ref",     default="Atmos_17_sim_06.csv")
    parser.add_argument("--output",  default="comparison_output")
    parser.add_argument("--dpi",     type=int, default=150)
    parser.add_argument("--fmt",     default="png",
                        choices=["png", "pdf", "svg"])
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    show = not args.no_show
    os.makedirs(args.output, exist_ok=True)

    print(f"Loading Aetherion : {args.sim}")
    sim_df  = pd.read_csv(args.sim)
    print(f"Loading NASA Sim06: {args.ref}")
    ref_raw = pd.read_csv(args.ref)
    ref_si  = convert_sim06_to_si(ref_raw)

    # Interpolate reference to sim time grid
    t_sim = sim_df["time"].values
    t_ref = ref_si["time"].values
    ref_df = pd.DataFrame({"time": t_sim})
    for col in ref_si.columns:
        if col == "time": continue
        ref_df[col] = np.interp(t_sim, t_ref, ref_si[col].values,
                                left=np.nan, right=np.nan)

    common = [c for c in sim_df.columns
              if c != "time" and c in ref_df.columns]
    print(f"  {len(common)} shared columns over {len(t_sim)} time steps\n")

    # ── Focused plots (altitude + pitch) ────────────────────────────────────
    plot_altitude_focused(t_sim, sim_df, ref_df, args.output,
                          args.dpi, args.fmt, show)
    plot_pitch_focused(t_sim, sim_df, ref_df, args.output,
                       args.dpi, args.fmt, show)

    # ── Standard per-column plots ────────────────────────────────────────────
    skip = {"altitudeMsl_m", "eulerAngle_rad_Pitch"}   # already done focused
    all_cols = [
        "altitudeMsl_m", "trueAirspeed_m_s", "mach", "dynamicPressure_Pa",
        "eulerAngle_rad_Yaw", "eulerAngle_rad_Roll",
        "feVelocity_m_s_X", "feVelocity_m_s_Y", "feVelocity_m_s_Z",
        "bodyAngularRateWrtEi_rad_s_Roll", "bodyAngularRateWrtEi_rad_s_Pitch",
        "bodyAngularRateWrtEi_rad_s_Yaw",
        "altitudeRateWrtMsl_m_s", "speedOfSound_m_s",
        "airDensity_kg_m3", "ambientPressure_Pa", "ambientTemperature_K",
        "aero_bodyForce_N_X", "aero_bodyForce_N_Y", "aero_bodyForce_N_Z",
        "aero_bodyMoment_Nm_L", "aero_bodyMoment_Nm_M", "aero_bodyMoment_Nm_N",
        "localGravity_m_s2", "latitude_rad", "longitude_rad",
    ]
    for col in all_cols:
        if col in skip or col not in common: continue
        s = sim_df[col].values.astype(float)
        v = ref_df[col].values.astype(float)
        fname = plot_column(t_sim, s, v, col,
                            args.output, args.dpi, args.fmt, show)
        print(f"  saved: {fname}")

    # ── Overview dashboard ───────────────────────────────────────────────────
    dash_cols = [c for c in DASHBOARD_COLS if c in common]
    fname = plot_dashboard(t_sim, sim_df, ref_df, dash_cols,
                           args.output, args.dpi, args.fmt, show)
    print(f"  saved: {fname}")

    # ── 3D trajectory ────────────────────────────────────────────────────────
    print("\nGenerating 3D trajectory plot…")
    plot_trajectory_3d(t_sim, sim_df, ref_si, args.output,
                       args.dpi, args.fmt, show)

    print("\nDone.")


if __name__ == "__main__":
    main()
