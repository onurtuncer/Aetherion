#!/usr/bin/env python3
"""
plot_convergence.py
-------------------
Reads papers/eucass/data/convergence_order.csv (written by the
[convergence][csv] Catch2 test) and produces a publication-quality
log-log convergence-order figure for the EUCASS paper.

The test problem is a torque-free *asymmetric* rigid body.  A symmetric body
cannot be used: its body twist is constant, the exact flow is a one-parameter
subgroup, and every RKMK method reproduces it exactly at every step size, so
the log-log slope is meaningless noise.  See test_ConvergenceOrder.cpp.

Output: papers/eucass/figures/convergence_order.pdf
        papers/eucass/figures/convergence_order.png  (for previews)

Usage (from the repo root):
    python papers/eucass/scripts/plot_convergence.py

Dependencies: numpy, matplotlib
    pip install numpy matplotlib
"""

import sys
import csv
import math
from pathlib import Path

# ---------------------------------------------------------------------------
# Locate repo root (three levels up from this script)
# ---------------------------------------------------------------------------
SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent.parent.parent   # papers/eucass/scripts -> repo root
DATA_FILE   = REPO_ROOT / "papers" / "eucass" / "data" / "convergence_order.csv"
FIGURES_DIR = REPO_ROOT / "papers" / "eucass" / "figures"


def orders(hs, errs, floor=1e-13):
    """Mean log-log slope over consecutive step-size pairs above the round-off floor."""
    return [
        math.log(errs[i] / errs[i + 1]) / math.log(hs[i] / hs[i + 1])
        for i in range(len(hs) - 1)
        if errs[i] > floor and errs[i + 1] > floor
    ]


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def main():
    # -----------------------------------------------------------------------
    # Read CSV
    # -----------------------------------------------------------------------
    if not DATA_FILE.exists():
        print(f"ERROR: {DATA_FILE} not found.")
        print("Build and run the [convergence][csv] Catch2 test first:")
        print("  ctest -R 'convergence.csv' --output-on-failure")
        sys.exit(1)

    cols = {k: [] for k in ("h", "rot_radau5", "pos_radau5", "rot_rk4", "pos_rk4")}
    with open(DATA_FILE, newline="") as f:
        reader = csv.DictReader(f)
        missing = [k for k in cols if k not in (reader.fieldnames or [])]
        if missing:
            print(f"ERROR: {DATA_FILE} is missing columns {missing}.")
            print(f"       Found: {reader.fieldnames}")
            print("       Re-run the [convergence][csv] test to regenerate it.")
            sys.exit(1)
        for row in reader:
            for k in cols:
                cols[k].append(float(row[k]))

    h = cols["h"]
    if len(h) < 2:
        print("ERROR: CSV has fewer than 2 rows -- re-run the test.")
        sys.exit(1)

    # -----------------------------------------------------------------------
    # Empirical orders
    # -----------------------------------------------------------------------
    summary = {}
    for key, label in (("rot_radau5", "Radau IIA  attitude"),
                       ("pos_radau5", "Radau IIA  position"),
                       ("rot_rk4",    "RK4        attitude"),
                       ("pos_rk4",    "RK4        position")):
        p = orders(h, cols[key])
        summary[key] = mean(p)
        print(f"{label}: orders = {[f'{v:.2f}' for v in p]}   mean = {mean(p):.2f}")

    # -----------------------------------------------------------------------
    # Plot
    # -----------------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib / numpy not available -- install with: pip install matplotlib numpy")
        sys.exit(1)

    TEXTWIDTH_PT  = 430.0                    # EUCASS text width (approx.)
    INCHES_PER_PT = 1.0 / 72.27
    FIG_W = TEXTWIDTH_PT * INCHES_PER_PT     # ~5.95 in
    FIG_H = FIG_W * 0.45

    fig, axes = plt.subplots(1, 2, figsize=(FIG_W, FIG_H))

    h_arr = np.array(h)
    h_ref = np.array([h_arr[0], h_arr[-1]])

    panels = [
        (axes[0], "rot_radau5", "rot_rk4",
         r"$\|R_\mathrm{num}(T) - R_\mathrm{ref}(T)\|_F$", "Attitude"),
        (axes[1], "pos_radau5", "pos_rk4",
         r"$\|p_\mathrm{num}(T) - p_\mathrm{ref}(T)\|_2$  [m]", "Position"),
    ]

    for ax, k_radau, k_rk4, ylabel, title in panels:
        er = np.array(cols[k_radau])
        ek = np.array(cols[k_rk4])

        ax.loglog(h_arr, er, "o-",  color="#1f77b4", lw=1.6, ms=4.5,
                  label=f"Radau IIA RKMK ($p$={summary[k_radau]:.2f})")
        ax.loglog(h_arr, ek, "s--", color="#d62728", lw=1.4, ms=4.5,
                  label=f"Explicit RK4 RKMK ($p$={summary[k_rk4]:.2f})")

        ax.loglog(h_ref, er[0] * (h_ref / h_arr[0]) ** 5,
                  "k:",  lw=0.9, label=r"$\mathcal{O}(h^5)$")
        ax.loglog(h_ref, ek[0] * (h_ref / h_arr[0]) ** 4,
                  "k-.", lw=0.9, label=r"$\mathcal{O}(h^4)$")

        ax.set_xlabel(r"Step size $h$ [s]", fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=9)
        ax.grid(True, which="both", ls=":", lw=0.5, alpha=0.6)
        ax.tick_params(labelsize=8)
        ax.legend(fontsize=7, loc="upper left")

    fig.suptitle(r"Convergence order: torque-free asymmetric body, $T=2\,\mathrm{s}$",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    pdf_path = FIGURES_DIR / "convergence_order.pdf"
    png_path = FIGURES_DIR / "convergence_order.png"
    fig.savefig(pdf_path, format="pdf", dpi=300, bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("\nFigures saved:")
    print(f"  {pdf_path}")
    print(f"  {png_path}")
    print("\nInclude in LaTeX with:")
    print(r"  \includegraphics[width=0.72\linewidth]{figures/convergence_order}")


if __name__ == "__main__":
    main()
