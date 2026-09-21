#!/usr/bin/env python3
"""
plot_constraint_drift.py
------------------------
Reads papers/eucass/data/manifold_factorial_trace.csv (written by the
[factorial][csv] Catch2 test) and produces the attitude-constraint figure for
the EUCASS paper.

Six traces over the same torque-free tumbling trajectory, forming a factorial
in (manifold treatment) x (Butcher tableau).  The cell letters match
Table~\\ref{tab:drift} in the paper:

  (a) SE(3) RKMK,  Radau IIA (5)   -- product of exact SO(3) elements
  (d) SE(3) RKMK,  explicit RK4    -- same, with a different tableau
  (c) flat state,  Radau IIA (5)   -- the controlled comparison against (a)
  (e) flat state,  Gauss (6)       -- symplectic, conserves ||q||^2 exactly
  (b) flat state,  explicit RK4    -- the conventional baseline
  (f) flat state,  RK4 + renorm    -- constraint met by projection

The point of the figure is that (a) vs (c) holds the tableau fixed, so the
nine-order separation between them is attributable to the Munthe-Kaas update
and to nothing else.

Output: papers/eucass/figures/constraint_drift.pdf
        papers/eucass/figures/constraint_drift.png

Usage (from the repo root):
    python papers/eucass/scripts/plot_constraint_drift.py

Dependencies: numpy, matplotlib
"""

import sys
import csv
from pathlib import Path

SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent.parent.parent
DATA_FILE   = REPO_ROOT / "papers" / "eucass" / "data" / "manifold_factorial_trace.csv"
FIGURES_DIR = REPO_ROOT / "papers" / "eucass" / "figures"

COLUMNS = ("t", "mk_radau5", "mk_rk4", "flat_radau5",
           "flat_gauss6", "flat_rk4", "flat_rk4_renorm")

# key, style, colour, linewidth, label
SERIES = (
    ("flat_rk4",        "-",  "#d62728", 1.6, "(b) flat, explicit RK4"),
    ("flat_radau5",     "-",  "#ff7f0e", 1.8, "(c) flat, Radau IIA (5)"),
    ("mk_radau5",       "-",  "#1f77b4", 1.8, "(a) SE(3) RKMK, Radau IIA (5)"),
    ("mk_rk4",          "--", "#17becf", 1.4, "(d) SE(3) RKMK, explicit RK4"),
    ("flat_gauss6",     "-",  "#2ca02c", 1.4, "(e) flat, Gauss (6), symplectic"),
    ("flat_rk4_renorm", ":",  "#7f7f7f", 1.4, "(f) flat, RK4 + re-normalisation"),
)


def main():
    if not DATA_FILE.exists():
        print(f"ERROR: {DATA_FILE} not found.")
        print("Build and run the [factorial][csv] Catch2 test first:")
        print("  ctest -R 'Manifold factorial' --output-on-failure")
        sys.exit(1)

    cols = {k: [] for k in COLUMNS}
    with open(DATA_FILE, newline="") as f:
        reader = csv.DictReader(f)
        missing = [k for k in COLUMNS if k not in (reader.fieldnames or [])]
        if missing:
            print(f"ERROR: {DATA_FILE} is missing columns {missing}.")
            sys.exit(1)
        for row in reader:
            for k in COLUMNS:
                cols[k].append(float(row[k]))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib / numpy not available -- install with: pip install matplotlib numpy")
        sys.exit(1)

    TEXTWIDTH_PT  = 430.0
    INCHES_PER_PT = 1.0 / 72.27
    FIG_W = TEXTWIDTH_PT * INCHES_PER_PT
    FIG_H = FIG_W * 0.62

    t = np.array(cols["t"])

    # A zero at t = 0 cannot be drawn on a log axis.  Clip to the smallest
    # positive value present anywhere in the data set, so that every series is
    # clipped identically and the first sample does not vanish silently.
    all_pos = np.concatenate([
        np.array(cols[k])[np.array(cols[k]) > 0.0] for k in COLUMNS[1:]
    ])
    floor = all_pos.min() if all_pos.size else 1e-18

    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))

    for key, style, colour, lw, label in SERIES:
        v = np.maximum(np.array(cols[key]), floor)
        ax.semilogy(t, v, style, color=colour, lw=lw, label=label)

    # Annotate the controlled comparison: same tableau, only the manifold
    # treatment differs.
    i = len(t) - 1
    anchor = (0.80 * t[-1], 3e-10)
    ratio = cols["flat_radau5"][i] / max(cols["mk_radau5"][i], 1e-300)
    ax.annotate(
        f"same tableau,\nMK toggled:\n{ratio:.0e}".replace("e+0", r"$\times10^{") + "}$",
        xy=(0.995 * t[-1], cols["flat_radau5"][i]),
        xytext=anchor,
        fontsize=8,
        ha="center",
        va="center",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#999999", lw=0.6),
        arrowprops=dict(arrowstyle="-|>", lw=0.9, color="#333333",
                        connectionstyle="arc3,rad=0.15"),
    )
    ax.annotate(
        "",
        xy=(0.995 * t[-1], max(cols["mk_radau5"][i], floor)),
        xytext=anchor,
        arrowprops=dict(arrowstyle="-|>", lw=0.9, color="#333333",
                        connectionstyle="arc3,rad=-0.15"),
    )

    ax.set_xlabel(r"Time $t$ [s]", fontsize=9)
    ax.set_ylabel(r"$\|R^\top R - I\|_F$", fontsize=9)
    ax.set_title("Attitude constraint residual, torque-free tumbling body",
                 fontsize=10)
    ax.grid(True, which="both", ls=":", lw=0.5, alpha=0.6)
    ax.tick_params(labelsize=8)
    ax.legend(fontsize=7.5, loc="center left", framealpha=0.92)

    fig.tight_layout()

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    pdf_path = FIGURES_DIR / "constraint_drift.pdf"
    png_path = FIGURES_DIR / "constraint_drift.png"
    fig.savefig(pdf_path, format="pdf", dpi=300, bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("Figures saved:")
    print(f"  {pdf_path}")
    print(f"  {png_path}")
    print(f"\nFinal values at t = {t[-1]:.0f} s:")
    for k in COLUMNS[1:]:
        print(f"  {k:18s} {cols[k][-1]:.3e}")

    ratio = cols["flat_radau5"][-1] / max(cols["mk_radau5"][-1], 1e-300)
    print(f"\nControlled comparison (c)/(a), same Radau IIA tableau: {ratio:.2e}")


if __name__ == "__main__":
    main()
