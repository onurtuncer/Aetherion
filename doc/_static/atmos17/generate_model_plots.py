"""
Generate model-data plots for Scenario 17 documentation.
Run from the doc/_static/atmos17/ directory (or adjust OUTPUT_DIR).
No simulation required — all data comes directly from the DML tables.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

STYLE = {
    "figure.dpi": 150,
    "axes.grid": True,
    "grid.linestyle": "--",
    "grid.alpha": 0.4,
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "legend.fontsize": 10,
    "lines.linewidth": 2.0,
}
plt.rcParams.update(STYLE)

BLUE  = "#1f77b4"
RED   = "#d62728"
GREEN = "#2ca02c"
ORNG  = "#ff7f0e"
PURP  = "#9467bd"

# ── Aerodynamic coefficient tables (from twostage_aero.dml) ──────────────────

alpha_bp = np.array([-10., -8., -6., -4., -2., 0., 2., 4., 6., 8., 10.])
beta_bp  = np.array([-10., -8., -6., -4., -2., 0., 2., 4., 6., 8., 10.])

CL_data  = np.array([-1.6, -1.0, -0.73, -0.49, -0.24, 0., 0.24, 0.49, 0.73, 1.0, 1.6])
CD_data  = np.array([0.48, 0.38, 0.31, 0.25, 0.23, 0.21, 0.23, 0.25, 0.31, 0.38, 0.48])
CY_data  = np.array([1.6, 1.0, 0.73, 0.49, 0.24, 0., -0.24, -0.49, -0.73, -1.0, -1.6])

alpha_m_bp = np.array([-20., 0., 20.])
beta_m_bp  = np.array([-20., 0., 20.])
Cm_data    = np.array([0.6, 0., -0.6])
Cn_data    = np.array([-0.6, 0., 0.6])

alpha_fine = np.linspace(-10, 10, 200)
beta_fine  = np.linspace(-10, 10, 200)
CL_fine    = np.interp(alpha_fine, alpha_bp, CL_data)
CD_fine    = np.interp(alpha_fine, alpha_bp, CD_data)
CY_fine    = np.interp(beta_fine,  beta_bp,  CY_data)

alpha_m_fine = np.linspace(-20, 20, 200)
beta_m_fine  = np.linspace(-20, 20, 200)
Cm_fine = np.interp(alpha_m_fine, alpha_m_bp, Cm_data)
Cn_fine = np.interp(beta_m_fine,  beta_m_bp,  Cn_data)

# ── Figure 1: Force coefficients (CL, CD, CY) ───────────────────────────────

fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
fig.suptitle("Aerodynamic Force Coefficients — twostage_aero.dml", fontweight="bold")

ax = axes[0]
ax.plot(alpha_fine, CL_fine, color=BLUE)
ax.plot(alpha_bp, CL_data, "o", color=BLUE, ms=5, label="Table breakpoints")
ax.set_xlabel("Angle of attack α [deg]")
ax.set_ylabel(r"$C_L$")
ax.set_title("Lift Coefficient")
ax.legend()

ax = axes[1]
ax.plot(alpha_fine, CD_fine, color=RED)
ax.plot(alpha_bp, CD_data, "o", color=RED, ms=5, label="Table breakpoints")
ax.set_xlabel("Total angle of attack α_T [deg]")
ax.set_ylabel(r"$C_D$")
ax.set_title("Drag Coefficient (vs. total α)")
ax.legend()

ax = axes[2]
ax.plot(beta_fine, CY_fine, color=GREEN)
ax.plot(beta_bp, CY_data, "o", color=GREEN, ms=5, label="Table breakpoints")
ax.set_xlabel("Sideslip angle β [deg]")
ax.set_ylabel(r"$C_Y$")
ax.set_title("Sideforce Coefficient")
ax.legend()

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "aero_force_coefficients.png"), bbox_inches="tight")
plt.close(fig)
print("Saved aero_force_coefficients.png")

# ── Figure 2: Moment coefficients (Cm, Cn) ──────────────────────────────────

fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
fig.suptitle("Aerodynamic Moment Coefficients — twostage_aero.dml", fontweight="bold")

ax = axes[0]
ax.plot(alpha_m_fine, Cm_fine, color=BLUE)
ax.plot(alpha_m_bp, Cm_data, "o", color=BLUE, ms=6, label="Table breakpoints")
ax.set_xlabel("Angle of attack α [deg]")
ax.set_ylabel(r"$C_m$")
ax.set_title("Pitching Moment Coefficient")
ax.legend()

ax = axes[1]
ax.plot(beta_m_fine, Cn_fine, color=RED)
ax.plot(beta_m_bp, Cn_data, "o", color=RED, ms=6, label="Table breakpoints")
ax.set_xlabel("Sideslip angle β [deg]")
ax.set_ylabel(r"$C_n$")
ax.set_title("Yawing Moment Coefficient")
ax.legend()

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "aero_moment_coefficients.png"), bbox_inches="tight")
plt.close(fig)
print("Saved aero_moment_coefficients.png")

# ── Figure 3: Stage 1 mass properties vs. fuel consumed ─────────────────────

stg1_fuel_cap = 180000.0  # kg
stg1_fuel = np.linspace(0, stg1_fuel_cap, 500)
stg1_frac  = (stg1_fuel_cap - stg1_fuel) / stg1_fuel_cap  # 1 = full, 0 = empty

mass_liftoff   = 314000.0;  mass_s1burnout  = 134000.0
xcg_liftoff    = 16.91879;  xcg_s1burnout   = 9.421642
ixx_liftoff    = 353250.0;  ixx_s1burnout   = 150750.0
iyy_liftoff    = 33501637.473461; iyy_s1burnout = 10886636.572139

mass_s1 = mass_s1burnout + stg1_fuel_cap * stg1_frac
xcg_s1  = xcg_s1burnout  + (xcg_liftoff - xcg_s1burnout)  * stg1_frac
ixx_s1  = ixx_s1burnout  + (ixx_liftoff - ixx_s1burnout)  * stg1_frac
iyy_s1  = iyy_s1burnout  + (iyy_liftoff - iyy_s1burnout)  * stg1_frac

fig, axes = plt.subplots(2, 2, figsize=(12, 9))
fig.suptitle("Stage 1 Mass Properties vs. Fuel Consumed — twostage_inertia.dml",
             fontweight="bold")

ax = axes[0, 0]
ax.plot(stg1_fuel / 1e3, mass_s1 / 1e3, color=BLUE)
ax.set_xlabel("Stage-1 fuel consumed [×10³ kg]")
ax.set_ylabel("Total mass [×10³ kg]")
ax.set_title("Vehicle Mass (Stage 1 active)")

ax = axes[0, 1]
ax.plot(stg1_fuel / 1e3, xcg_s1, color=RED)
ax.set_xlabel("Stage-1 fuel consumed [×10³ kg]")
ax.set_ylabel(r"$x_{CG}$ [m aft of nose]")
ax.set_title("Centre-of-Gravity (Stage 1 active)")

ax = axes[1, 0]
ax.plot(stg1_fuel / 1e3, ixx_s1 / 1e3, color=GREEN)
ax.set_xlabel("Stage-1 fuel consumed [×10³ kg]")
ax.set_ylabel(r"$I_{xx}$ [×10³ kg·m²]")
ax.set_title("Roll Inertia (Stage 1 active)")

ax = axes[1, 1]
ax.plot(stg1_fuel / 1e3, iyy_s1 / 1e6, color=ORNG)
ax.set_xlabel("Stage-1 fuel consumed [×10³ kg]")
ax.set_ylabel(r"$I_{yy} = I_{zz}$ [×10⁶ kg·m²]")
ax.set_title("Pitch/Yaw Inertia (Stage 1 active)")

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "mass_properties_stage1.png"), bbox_inches="tight")
plt.close(fig)
print("Saved mass_properties_stage1.png")

# ── Figure 4: Stage 2 mass properties vs. fuel consumed ─────────────────────

stg2_fuel_cap = 80000.0  # kg
stg2_fuel = np.linspace(0, stg2_fuel_cap, 500)
stg2_frac  = (stg2_fuel_cap - stg2_fuel) / stg2_fuel_cap

mass_s2ign  = 99000.0;   mass_s2burnout  = 19000.0
xcg_s2ign   = 4.79798;   xcg_s2burnout   = 3.947368
ixx_s2ign   = 111375.0;  ixx_s2burnout   = 21375.0
iyy_s2ign   = 941063.762626; iyy_s2burnout = 212384.868421

mass_s2 = mass_s2burnout + stg2_fuel_cap * stg2_frac
xcg_s2  = xcg_s2burnout  + (xcg_s2ign - xcg_s2burnout)  * stg2_frac
ixx_s2  = ixx_s2burnout  + (ixx_s2ign - ixx_s2burnout)  * stg2_frac
iyy_s2  = iyy_s2burnout  + (iyy_s2ign - iyy_s2burnout)  * stg2_frac

fig, axes = plt.subplots(2, 2, figsize=(12, 9))
fig.suptitle("Stage 2 Mass Properties vs. Fuel Consumed — twostage_inertia.dml",
             fontweight="bold")

ax = axes[0, 0]
ax.plot(stg2_fuel / 1e3, mass_s2 / 1e3, color=BLUE)
ax.set_xlabel("Stage-2 fuel consumed [×10³ kg]")
ax.set_ylabel("Vehicle mass [×10³ kg]")
ax.set_title("Vehicle Mass (Stage 2 active)")

ax = axes[0, 1]
ax.plot(stg2_fuel / 1e3, xcg_s2, color=RED)
ax.set_xlabel("Stage-2 fuel consumed [×10³ kg]")
ax.set_ylabel(r"$x_{CG}$ [m aft of nose]")
ax.set_title("Centre-of-Gravity (Stage 2 active)")

ax = axes[1, 0]
ax.plot(stg2_fuel / 1e3, ixx_s2 / 1e3, color=GREEN)
ax.set_xlabel("Stage-2 fuel consumed [×10³ kg]")
ax.set_ylabel(r"$I_{xx}$ [×10³ kg·m²]")
ax.set_title("Roll Inertia (Stage 2 active)")

ax = axes[1, 1]
ax.plot(stg2_fuel / 1e3, iyy_s2 / 1e3, color=ORNG)
ax.set_xlabel("Stage-2 fuel consumed [×10³ kg]")
ax.set_ylabel(r"$I_{yy} = I_{zz}$ [×10³ kg·m²]")
ax.set_title("Pitch/Yaw Inertia (Stage 2 active)")

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "mass_properties_stage2.png"), bbox_inches="tight")
plt.close(fig)
print("Saved mass_properties_stage2.png")

# ── Figure 5: Propulsion — thrust and mass flow rate per stage ───────────────

g0 = 9.8066  # m/s²
stages = {
    "Stage 1": {"T": 17e6, "Isp": 360.0, "color": BLUE},
    "Stage 2": {"T":  5e6, "Isp": 390.0, "color": RED},
}

fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
fig.suptitle("Propulsion Model Constants — twostage_prop.dml", fontweight="bold")

labels = list(stages.keys())
colors = [s["color"] for s in stages.values()]
T_vals = [s["T"] / 1e6 for s in stages.values()]
mdot_vals = [s["T"] / (s["Isp"] * g0) for s in stages.values()]

ax = axes[0]
bars = ax.bar(labels, T_vals, color=colors, width=0.4)
ax.set_ylabel("Thrust [MN]")
ax.set_title("Maximum Thrust per Stage")
for bar, v in zip(bars, T_vals):
    ax.text(bar.get_x() + bar.get_width() / 2, v + 0.1, f"{v:.0f} MN",
            ha="center", va="bottom", fontsize=11)
ax.set_ylim(0, max(T_vals) * 1.25)

ax = axes[1]
bars = ax.bar(labels, mdot_vals, color=colors, width=0.4)
ax.set_ylabel(r"$\dot{m}$ [kg/s]")
ax.set_title("Propellant Mass Flow Rate")
for bar, v in zip(bars, mdot_vals):
    ax.text(bar.get_x() + bar.get_width() / 2, v + 20, f"{v:.1f} kg/s",
            ha="center", va="bottom", fontsize=11)
ax.set_ylim(0, max(mdot_vals) * 1.3)

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "propulsion_constants.png"), bbox_inches="tight")
plt.close(fig)
print("Saved propulsion_constants.png")

# ── Figure 6: Wind-to-body force transform diagram (schematic) ───────────────

fig, ax = plt.subplots(figsize=(8, 5))
ax.set_xlim(-0.2, 1.2)
ax.set_ylim(-0.7, 1.05)
ax.set_aspect("equal")
ax.axis("off")
fig.suptitle("Wind-to-Body Force Transform — Aerodynamic Angle Convention",
             fontweight="bold", fontsize=12)

# Draw body axis (rocket nose to right = +x_body)
ax.annotate("", xy=(1.1, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", lw=2, color=BLUE))
ax.text(1.12, 0, r"$x_B$ (nose / thrust)", fontsize=10, color=BLUE, va="center")

# Draw wind velocity vector (pointing right and slightly down)
alpha_deg = 8.0
alpha_rad = np.deg2rad(alpha_deg)
vt = 0.9
vx = vt * np.cos(alpha_rad)
vz = vt * np.sin(alpha_rad)  # positive = down in body frame
ax.annotate("", xy=(vx, -vz), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", lw=2, color=RED))
ax.text(vx + 0.02, -vz - 0.05, r"$\mathbf{v}_{rel}$", fontsize=11, color=RED)

# Draw body z axis (down)
ax.annotate("", xy=(0, -0.6), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", lw=2, color=ORNG))
ax.text(0.02, -0.63, r"$z_B$ (down)", fontsize=10, color=ORNG)

# Draw body y axis (into page symbol)
ax.plot(0, 0, "x", ms=12, mew=2, color=GREEN)
ax.text(0.04, 0.05, r"$y_B$ (into page)", fontsize=10, color=GREEN)

# Arc for alpha
theta_arc = np.linspace(0, -alpha_rad, 50)
r_arc = 0.35
ax.plot(r_arc * np.cos(theta_arc), r_arc * np.sin(theta_arc), color="gray", lw=1.5)
ax.text(r_arc * np.cos(-alpha_rad / 2) + 0.04,
        r_arc * np.sin(-alpha_rad / 2) + 0.02,
        r"$\alpha$", fontsize=12, color="gray")

# Drag and lift vectors (from velocity direction)
Fx_drag = -vx * 0.4
Fz_drag = vz * 0.4
ax.annotate("", xy=(vx / 2 + Fx_drag, -vz / 2 + Fz_drag), xytext=(vx / 2, -vz / 2),
            arrowprops=dict(arrowstyle="->", lw=2, color=PURP))
ax.text(vx / 2 + Fx_drag - 0.08, -vz / 2 + Fz_drag + 0.04, r"$D = C_D\,\bar{q}S$",
        fontsize=10, color=PURP)

lift_perp_x = np.sin(alpha_rad) * 0.4
lift_perp_z = np.cos(alpha_rad) * 0.4
ax.annotate("", xy=(vx / 2 + lift_perp_x, -vz / 2 - lift_perp_z),
            xytext=(vx / 2, -vz / 2),
            arrowprops=dict(arrowstyle="->", lw=2, color="darkcyan"))
ax.text(vx / 2 + lift_perp_x + 0.02, -vz / 2 - lift_perp_z - 0.06,
        r"$L = C_L\,\bar{q}S$", fontsize=10, color="darkcyan")

ax.text(0.5, 0.95, r"$\mathbf{F}_{body} = R_{BW}\,\mathbf{F}_{wind}$,"
        r"  $R_{BW} = R_y(\alpha)\,R_z(\beta)$",
        fontsize=11, ha="center", va="top",
        bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow", ec="gray"))

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "wind_body_transform.png"), bbox_inches="tight")
plt.close(fig)
print("Saved wind_body_transform.png")

# ── Figure 7: CG-MRC offset diagram vs. fuel state ──────────────────────────

fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
fig.suptitle(r"CG Position and DXCG = MRC $-$ CG Offset", fontweight="bold")

ax = axes[0]
ax.plot(stg1_fuel / 1e3, xcg_s1, color=BLUE, label=r"$x_{CG}$ (Stage 1 active)")
ax.axhline(xcg_liftoff, ls="--", color=BLUE, lw=1, alpha=0.5, label=r"Liftoff $x_{CG}$")
MRC_full = 16.918790
ax.axhline(MRC_full, ls=":", color="gray", lw=1.5, label=r"MRC$_1$ = 16.919 m")
ax.fill_between(stg1_fuel / 1e3, xcg_s1, MRC_full, alpha=0.15, color=RED,
                label=r"DXCG = MRC $-$ $x_{CG}$")
ax.set_xlabel("Stage-1 fuel consumed [×10³ kg]")
ax.set_ylabel("Position aft of nose [m]")
ax.set_title("Stage 1: CG and MRC positions")
ax.legend(fontsize=9)

ax = axes[1]
ax.plot(stg2_fuel / 1e3, xcg_s2, color=RED, label=r"$x_{CG}$ (Stage 2 active)")
MRC_stg2 = 4.797980
ax.axhline(MRC_stg2, ls=":", color="gray", lw=1.5, label=r"MRC$_2$ = 4.798 m")
ax.fill_between(stg2_fuel / 1e3, xcg_s2, MRC_stg2, alpha=0.15, color=BLUE,
                label=r"DXCG = MRC $-$ $x_{CG}$")
ax.set_xlabel("Stage-2 fuel consumed [×10³ kg]")
ax.set_ylabel("Position aft of nose [m]")
ax.set_title("Stage 2: CG and MRC positions")
ax.legend(fontsize=9)

fig.tight_layout()
fig.savefig(os.path.join(OUTPUT_DIR, "cg_mrc_offset.png"), bbox_inches="tight")
plt.close(fig)
print("Saved cg_mrc_offset.png")

print("\nAll plots generated successfully.")
