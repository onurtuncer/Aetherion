#!/usr/bin/env python3
"""
plot_constraint_drift.py
------------------------
Reads papers/eucass/data/constraint_drift.csv (written by the [drift][csv]
Catch2 test) and produces the attitude-constraint figure for the EUCASS paper.

The three traces are, over the same torque-free tumbling trajectory:
  * SE(3) RKMK              -- pose is a product of exact SO(3) elements
  * quaternion RK4, plain   -- 13-component unconstrained state
  * quaternion RK4, renorm. -- same, projected back to the unit sphere each step

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
DATA_FILE   = REPO_ROOT / "papers" / "eucass" / "data" / "constraint_drift.csv"
FIGURES_DIR = REPO_ROOT / "papers" / "eucass" / "figures"

COLUMNS = ("t", "rkmk_se3", "quat_rk4_plain", "quat_rk4_renorm")


def main():
    if not DATA_FILE.exists():
        print(f"ERROR: {DATA_FILE} not found.")
        print("Build and run the [drift][csv] Catch2 test first:")
        print("  ctest -R 'Constraint drift' --output-on-failure")
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
    FIG_H = FIG_W * 0.55

    t = np.array(cols["t"])

    # A zero at t = 0 cannot be drawn on a log axis; clip to the smallest
    # positive value present so the first sample does not vanish silently.
    def clipped(key):
        v = np.array(cols[key])
        pos = v[v > 0.0]
        floor = pos.min() if pos.size else 1e-18
        return np.maximum(v, floor)

    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))

    ax.semilogy(t, clipped("quat_rk4_plain"), "-", color="#d62728", lw=1.6,
                label="Quaternion RK4, no re-normalisation")
    ax.semilogy(t, clipped("rkmk_se3"), "-", color="#1f77b4", lw=1.6,
                label="SE(3) RKMK (this work)")
    ax.semilogy(t, clipped("quat_rk4_renorm"), "--", color="#2ca02c", lw=1.4,
                label="Quaternion RK4, re-normalised each step")

    ax.set_xlabel(r"Time $t$ [s]", fontsize=9)
    ax.set_ylabel(r"$\|R^\top R - I\|_F$", fontsize=9)
    ax.set_title("Attitude constraint residual, torque-free tumbling body",
                 fontsize=10)
    ax.grid(True, which="both", ls=":", lw=0.5, alpha=0.6)
    ax.tick_params(labelsize=8)
    ax.legend(fontsize=8, loc="center right")

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


if __name__ == "__main__":
    main()
