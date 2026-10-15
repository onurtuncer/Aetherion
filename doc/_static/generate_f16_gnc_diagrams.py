#!/usr/bin/env python3
"""
generate_f16_gnc_diagrams.py
Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University

Block diagrams of the F-16 GNC model (F16_control.dml) for doc/examples.rst:

    f16_gnc_overview.png      two-loop architecture (autopilot outer loop,
                              LQR stability-augmentation inner loop)
    f16_lqr_longitudinal.png  longitudinal LQR signal flow
    f16_lqr_lateral.png       lateral-directional LQR signal flow

They draw the structure the section's captions describe; the gain values
themselves are read from the DML at run time and are not reproduced here.

Usage:
    python doc/_static/generate_f16_gnc_diagrams.py [--output doc/_static]
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle

INK = "#1F2937"
BOX = "#EEF2FF"
BOX_EDGE = "#4F46E5"
GAIN = "#FEF3C7"
GAIN_EDGE = "#B45309"
SUM = "#F3F4F6"
DASH = "#6B7280"


def box(ax, x, y, w, h, text, fc=BOX, ec=BOX_EDGE, fs=9, weight="normal"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=fc, ec=ec, lw=1.3, zorder=3))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            color=INK, fontweight=weight, zorder=4)


def arrow(ax, p0, p1, text=None, ls="-", color=INK, dy=0.12, fs=8, ha="center"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=12,
                                 lw=1.2, ls=ls, color=color, zorder=2,
                                 shrinkA=0, shrinkB=0))
    if text:
        ax.text((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2 + dy, text, ha=ha,
                va="bottom", fontsize=fs, color=color, zorder=4)


def summing(ax, x, y, r=0.16, signs=("+", "+")):
    ax.add_patch(Circle((x, y), r, fc=SUM, ec=INK, lw=1.2, zorder=3))
    ax.plot([x - r, x + r], [y, y], color=INK, lw=0.8, zorder=4)
    ax.plot([x, x], [y - r, y + r], color=INK, lw=0.8, zorder=4)
    ax.text(x - r * 0.55, y + r * 0.5, signs[0], ha="center", va="center", fontsize=8, zorder=5)
    ax.text(x + r * 0.55, y + r * 0.5, signs[1], ha="center", va="center", fontsize=8, zorder=5)


def new_axes(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def overview(out):
    fig, ax = new_axes(12, 5.2)
    box(ax, 0.3, 1.0, 1.6, 3.0, "Pilot /\nscenario\ncommands\n\n$h_{cmd}$\n$V_{eq,cmd}$\n$\\chi_{cmd}$\nlateral offset", fc="#F9FAFB", ec=DASH)
    box(ax, 2.6, 2.5, 2.6, 1.5, "Autopilot outer loop\n(altitude, airspeed,\nheading hold)\nmode flag apOn", weight="bold")
    box(ax, 5.9, 2.5, 2.6, 1.5, "LQR stability\naugmentation\n(inner loop)\nmode flag sasOn", weight="bold")
    box(ax, 9.2, 2.5, 2.4, 1.5, "F-16 plant\n(DAVE-ML aero,\npropulsion, 6-DoF)")
    box(ax, 5.9, 0.4, 2.6, 1.0, "Trim feed-forwards\n$\\ell_{trim}$, $t_{trim}$", fc=GAIN, ec=GAIN_EDGE)

    arrow(ax, (1.9, 3.25), (2.6, 3.25))
    arrow(ax, (5.2, 3.25), (5.9, 3.25), "commanded states")
    arrow(ax, (8.5, 3.25), (9.2, 3.25), "$\\delta_e, \\delta_a, \\delta_r, N$")
    arrow(ax, (7.2, 1.4), (7.2, 2.5))
    # feedback path from the plant
    ax.plot([11.6, 11.85, 11.85, 4.0, 4.0], [3.25, 3.25, 4.75, 4.75, 4.0], color=INK, lw=1.2, zorder=2)
    ax.plot([11.85, 7.2, 7.2], [4.75, 4.75, 4.0], color=INK, lw=1.2, zorder=2)
    arrow(ax, (4.0, 4.1), (4.0, 4.0))
    arrow(ax, (7.2, 4.1), (7.2, 4.0))
    ax.text(7.9, 4.85, "sensor feedbacks: $h, V_{eq}, \\alpha, \\beta, \\phi, \\theta, \\psi, p, q, r$",
            ha="center", va="bottom", fontsize=8.5, color=INK)
    ax.text(6.0, 0.12, "Gains and trim values are read from F16_control.dml at run time.",
            ha="center", va="bottom", fontsize=8, color=DASH, style="italic")
    fig.savefig(os.path.join(out, "f16_gnc_overview.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def longitudinal(out):
    fig, ax = new_axes(12, 5.6)
    errs = ["$\\Delta V$", "$\\Delta\\alpha$", "$q$", "$\\Delta\\theta$"]
    ys = [4.6, 3.6, 2.6, 1.6]
    for e, y in zip(errs, ys):
        box(ax, 0.3, y - 0.3, 1.3, 0.6, e, fc="#F9FAFB", ec=DASH)
        arrow(ax, (1.6, y), (2.4, y))
        arrow(ax, (1.6, y), (2.4, y - 0.0), ls="--", color=DASH)  # overlay to hint reuse
    box(ax, 2.4, 1.2, 1.9, 3.8, "$\\mathbf{K}_{long}$\n(1 × 4)", fc=GAIN, ec=GAIN_EDGE, weight="bold")
    box(ax, 2.4, 0.2, 1.9, 0.75, "$\\mathbf{K}_{throt}$ (1 × 4)", fc=GAIN, ec=GAIN_EDGE)
    # dashed: throttle reuses the same errors
    for y in ys:
        ax.plot([2.0, 2.0], [y, 0.575], color=DASH, lw=0.8, ls="--", zorder=1)
    ax.plot([2.0, 2.4], [0.575, 0.575], color=DASH, lw=0.8, ls="--", zorder=1)
    ax.text(1.95, 0.75, "same errors\n(dashed)", fontsize=7, color=DASH, ha="right", va="center")

    # elevator channel
    summing(ax, 5.3, 3.1, signs=("−", "+"))
    arrow(ax, (4.3, 3.1), (5.14, 3.1), "$\\sum$", dy=0.08)
    box(ax, 4.6, 4.3, 1.5, 0.6, "$\\ell_{trim}$", fc=GAIN, ec=GAIN_EDGE)
    arrow(ax, (5.3, 4.3), (5.3, 3.26))
    box(ax, 6.2, 2.75, 1.7, 0.7, "saturate\n[$\\delta_{e,min}$, $\\delta_{e,max}$]")
    arrow(ax, (5.46, 3.1), (6.2, 3.1))
    box(ax, 8.5, 2.75, 1.5, 0.7, "scale\nrad → deg")
    arrow(ax, (7.9, 3.1), (8.5, 3.1))
    arrow(ax, (10.0, 3.1), (11.0, 3.1), "$\\delta_e$ [deg, TED+]", dy=0.1)

    # throttle channel
    summing(ax, 5.3, 0.575, signs=("−", "+"))
    arrow(ax, (4.3, 0.575), (5.14, 0.575), "$\\sum$", dy=0.08)
    box(ax, 4.6, 1.3, 1.5, 0.55, "$t_{trim}$", fc=GAIN, ec=GAIN_EDGE)
    arrow(ax, (5.3, 1.3), (5.3, 0.735))
    box(ax, 6.2, 0.25, 1.7, 0.65, "saturate\n[0, 1]")
    arrow(ax, (5.46, 0.575), (6.2, 0.575))
    box(ax, 8.5, 0.25, 1.5, 0.65, "scale\n× 100")
    arrow(ax, (7.9, 0.575), (8.5, 0.575))
    arrow(ax, (10.0, 0.575), (11.0, 0.575), "$N$ [0–100 %]", dy=0.1)

    ax.text(6.0, 5.35, "Longitudinal LQR: state errors × gains, summed and negated, plus trim feed-forward",
            ha="center", va="bottom", fontsize=9.5, color=INK, fontweight="bold")
    fig.savefig(os.path.join(out, "f16_lqr_longitudinal.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def lateral(out):
    fig, ax = new_axes(12, 5.2)
    errs = ["$\\Delta\\phi$", "$\\beta$", "$p$", "$r$"]
    ys = [4.3, 3.4, 2.5, 1.6]
    for e, y in zip(errs, ys):
        box(ax, 0.3, y - 0.3, 1.3, 0.6, e, fc="#F9FAFB", ec=DASH)
        arrow(ax, (1.6, y), (2.4, y))
    box(ax, 2.4, 3.0, 1.9, 1.8, "$\\mathbf{K}_{ail}$\n(1 × 4)", fc=GAIN, ec=GAIN_EDGE, weight="bold")
    box(ax, 2.4, 0.9, 1.9, 1.8, "$\\mathbf{K}_{rdr}$\n(1 × 4)", fc=GAIN, ec=GAIN_EDGE, weight="bold")
    for y in ys:
        ax.plot([2.0, 2.0], [y, 1.8], color=DASH, lw=0.8, ls="--", zorder=1)
    ax.plot([2.0, 2.4], [1.8, 1.8], color=DASH, lw=0.8, ls="--", zorder=1)

    # aileron
    summing(ax, 5.3, 3.9, signs=("−", " "))
    arrow(ax, (4.3, 3.9), (5.14, 3.9), "$\\sum$", dy=0.08)
    box(ax, 6.2, 3.55, 1.7, 0.7, "saturate,\nscale rad → deg")
    arrow(ax, (5.46, 3.9), (6.2, 3.9))
    arrow(ax, (7.9, 3.9), (11.0, 3.9), "$\\delta_a$ [deg, LWD+]", dy=0.1)

    # rudder with aileron cross-feed
    summing(ax, 5.3, 1.8, signs=("−", "+"))
    arrow(ax, (4.3, 1.8), (5.14, 1.8), "$\\sum$", dy=0.08)
    box(ax, 6.2, 1.45, 1.7, 0.7, "saturate,\nscale rad → deg")
    arrow(ax, (5.46, 1.8), (6.2, 1.8))
    summing(ax, 8.6, 1.8, signs=("+", "+"))
    arrow(ax, (7.9, 1.8), (8.44, 1.8))
    arrow(ax, (8.76, 1.8), (11.0, 1.8), "$\\delta_r$ [deg, TEL+]", dy=0.1)
    # cross-coupling: delta_r += 0.008 * delta_a (dashed)
    box(ax, 8.0, 2.55, 1.2, 0.5, "× 0.008", fc=GAIN, ec=GAIN_EDGE, fs=8)
    ax.plot([9.6, 9.6], [3.9, 3.05], color=DASH, lw=1.0, ls="--", zorder=2)
    ax.plot([9.6, 9.2], [3.05, 3.05], color=DASH, lw=1.0, ls="--", zorder=2)
    ax.plot([8.6, 8.6], [2.55, 2.0], color=DASH, lw=1.0, ls="--", zorder=2)
    ax.add_patch(FancyArrowPatch((8.6, 2.2), (8.6, 1.96), arrowstyle="-|>", mutation_scale=12,
                                 lw=1.0, ls="--", color=DASH, zorder=2))
    ax.text(9.65, 2.6, "aileron-to-rudder\ncross-feed (dashed)", fontsize=7.5, color=DASH, ha="left", va="center")

    ax.text(6.0, 4.95, "Lateral-directional LQR: state errors × gains, summed and negated; $\\delta_r \\leftarrow \\delta_r + 0.008\\,\\delta_a$",
            ha="center", va="bottom", fontsize=9.5, color=INK, fontweight="bold")
    fig.savefig(os.path.join(out, "f16_lqr_lateral.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", default=os.path.dirname(os.path.abspath(__file__)))
    args = ap.parse_args()
    os.makedirs(args.output, exist_ok=True)
    overview(args.output)
    longitudinal(args.output)
    lateral(args.output)
    print("wrote f16_gnc_overview.png, f16_lqr_longitudinal.png, f16_lqr_lateral.png to", args.output)


if __name__ == "__main__":
    main()
