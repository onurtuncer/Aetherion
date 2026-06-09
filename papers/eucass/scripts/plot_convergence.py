#!/usr/bin/env python3
"""
plot_convergence.py
-------------------
Reads papers/eucass/data/convergence_order.csv (written by the
[convergence][csv] Catch2 test) and produces a publication-quality
log-log convergence-order figure for the EUCASS paper.

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
# Locate repo root (two levels up from this script)
# ---------------------------------------------------------------------------
SCRIPT_DIR  = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent.parent.parent   # papers/eucass/scripts -> repo root
DATA_FILE   = REPO_ROOT / "papers" / "eucass" / "data" / "convergence_order.csv"
FIGURES_DIR = REPO_ROOT / "papers" / "eucass" / "figures"

def main():
    # -----------------------------------------------------------------------
    # Read CSV
    # -----------------------------------------------------------------------
    if not DATA_FILE.exists():
        print(f"ERROR: {DATA_FILE} not found.")
        print("Build and run the [convergence][csv] Catch2 test first:")
        print("  ctest -R 'convergence.csv' --output-on-failure")
        sys.exit(1)

    h_vals, err_radau, err_rk4 = [], [], []
    with open(DATA_FILE, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            h_vals.append(float(row["h"]))
            err_radau.append(float(row["error_radau5_rkmk"]))
            err_rk4.append(float(row["error_rk4_rkmk"]))

    if len(h_vals) < 2:
        print("ERROR: CSV has fewer than 2 rows — re-run the test.")
        sys.exit(1)

    # -----------------------------------------------------------------------
    # Compute empirical orders
    # -----------------------------------------------------------------------
    def orders(hs, errs):
        return [
            math.log(errs[i] / errs[i+1]) / math.log(hs[i] / hs[i+1])
            for i in range(len(hs) - 1)
            if errs[i] > 1e-14 and errs[i+1] > 1e-14
        ]

    ord_radau = orders(h_vals, err_radau)
    ord_rk4   = orders(h_vals, err_rk4)

    print(f"Radau IIA RKMK  — empirical orders: {[f'{p:.2f}' for p in ord_radau]}")
    print(f"                  mean = {sum(ord_radau)/len(ord_radau):.2f}")
    print(f"Explicit RK4 RKMK — empirical orders: {[f'{p:.2f}' for p in ord_rk4]}")
    print(f"                    mean = {sum(ord_rk4)/len(ord_rk4):.2f}")

    # -----------------------------------------------------------------------
    # Plot
    # -----------------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib / numpy not available — install with: pip install matplotlib numpy")
        sys.exit(1)

    TEXTWIDTH_PT = 430.0                    # EUCASS column width (approx.)
    INCHES_PER_PT = 1.0 / 72.27
    FIG_W = TEXTWIDTH_PT * INCHES_PER_PT    # ~5.95 in
    FIG_H = FIG_W * 0.75

    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))

    h_arr  = np.array(h_vals)
    er_arr = np.array(err_radau)
    ek_arr = np.array(err_rk4)

    # Data lines
    ax.loglog(h_arr, er_arr, "o-",  color="#1f77b4", lw=1.6, ms=5,
              label=r"Radau IIA RKMK (this work)")
    ax.loglog(h_arr, ek_arr, "s--", color="#d62728", lw=1.4, ms=5,
              label=r"Explicit RK4 RKMK (baseline)")

    # Reference slopes anchored at the coarsest step size
    h_ref = np.array([h_arr[0], h_arr[-1]])

    def ref_line(order, anchor_val, h_anchor):
        return anchor_val * (h_ref / h_anchor) ** order

    ax.loglog(h_ref, ref_line(5, er_arr[0], h_arr[0]),
              "k:",  lw=0.9, label=r"$\mathcal{O}(h^5)$ slope")
    ax.loglog(h_ref, ref_line(4, ek_arr[0], h_arr[0]),
              "k-.", lw=0.9, label=r"$\mathcal{O}(h^4)$ slope")

    ax.set_xlabel(r"Step size $h$ [s]", fontsize=10)
    ax.set_ylabel(r"$\|R_\mathrm{num}(T) - R_\mathrm{exact}(T)\|_F$", fontsize=10)
    ax.set_title(r"Convergence order — free symmetric sphere, $T=1\,\mathrm{s}$",
                 fontsize=10)
    ax.legend(fontsize=9, loc="upper left")
    ax.grid(True, which="both", ls=":", lw=0.5, alpha=0.6)
    ax.tick_params(labelsize=9)

    # Annotate mean orders
    mean_r = sum(ord_radau) / len(ord_radau)
    mean_k = sum(ord_rk4)   / len(ord_rk4)
    ax.annotate(f"mean order = {mean_r:.2f}",
                xy=(h_arr[len(h_arr)//2], er_arr[len(er_arr)//2]),
                xytext=(6, 12), textcoords="offset points",
                fontsize=8, color="#1f77b4")
    ax.annotate(f"mean order = {mean_k:.2f}",
                xy=(h_arr[len(h_arr)//2], ek_arr[len(ek_arr)//2]),
                xytext=(6, -18), textcoords="offset points",
                fontsize=8, color="#d62728")

    fig.tight_layout()

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    pdf_path = FIGURES_DIR / "convergence_order.pdf"
    png_path = FIGURES_DIR / "convergence_order.png"
    fig.savefig(pdf_path, format="pdf", dpi=300, bbox_inches="tight")
    fig.savefig(png_path, format="png", dpi=200, bbox_inches="tight")
    plt.close(fig)

    print(f"\nFigures saved:")
    print(f"  {pdf_path}")
    print(f"  {png_path}")
    print(f"\nInclude in LaTeX with:")
    print(r"  \includegraphics[width=\columnwidth]{figures/convergence_order}")

if __name__ == "__main__":
    main()
