"""
compare_sim_validation.py
Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
─────────────────────────
Compare Aetherion simulation results against validation data.

Usage:
    python compare_sim_validation.py <sim_csv> <validation_csv> [options]

    python compare_sim_validation.py sim.csv validation.csv
    python compare_sim_validation.py sim.csv validation.csv --output my_report --time-col time
    python compare_sim_validation.py sim.csv validation.csv --columns altitudeMsl_m mach trueAirspeed_m_s
    python compare_sim_validation.py sim.csv validation.csv --error-type relative

Arguments:
    sim_csv         Path to Aetherion simulation CSV file
    validation_csv  Path to validation/reference data CSV file

Options:
    --output        Output directory (default: comparison_output)
    --time-col      Name of the time column (default: time)
    --columns       Specific columns to plot (default: all numeric columns)
    --error-type    Error metric: 'absolute' | 'relative' | 'both' (default: absolute)
    --interp        Interpolate validation data to sim time grid if time axes differ
    --no-show       Don't open plot windows, only save to disk
    --dpi           Figure DPI (default: 150)
    --fmt           Output figure format: png | pdf | svg (default: png)
"""

import argparse
import os
import sys
import warnings
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd
from matplotlib.gridspec import GridSpec

warnings.filterwarnings("ignore")

# ─── Style ──────────────────────────────────────────────────────────────────

SIM_COLOR   = "#2563EB"   # blue   – Aetherion sim
VAL_COLOR   = "#DC2626"   # red    – validation / reference
ERR_COLOR   = "#16A34A"   # green  – error
FILL_ALPHA  = 0.12
GRID_ALPHA  = 0.25
LINE_WIDTH  = 1.5

plt.rcParams.update({
    "figure.facecolor":  "white",
    "axes.facecolor":    "white",
    "axes.edgecolor":    "#374151",
    "axes.labelcolor":   "#111827",
    "axes.grid":         True,
    "grid.color":        "#D1D5DB",
    "grid.linestyle":    "--",
    "grid.linewidth":    0.6,
    "xtick.color":       "#374151",
    "ytick.color":       "#374151",
    "text.color":        "#111827",
    "legend.framealpha": 0.9,
    "legend.edgecolor":  "#D1D5DB",
    "font.family":       "sans-serif",
    "font.size":         9,
    "axes.titlesize":    10,
    "axes.labelsize":    9,
})

# ─── Helpers ─────────────────────────────────────────────────────────────────

def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    return df


def align_dataframes(df_sim: pd.DataFrame,
                     df_val: pd.DataFrame,
                     time_col: str,
                     interp: bool) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Align sim and validation on a common time axis."""
    if interp:
        t_sim = df_sim[time_col].values
        df_val_aligned = pd.DataFrame({time_col: t_sim})
        for col in df_val.columns:
            if col == time_col:
                continue
            if col in df_sim.columns:
                df_val_aligned[col] = np.interp(
                    t_sim,
                    df_val[time_col].values,
                    df_val[col].values,
                    left=np.nan,
                    right=np.nan,
                )
        return df_sim, df_val_aligned
    else:
        # Merge on time col (inner join keeps only matching rows)
        merged = pd.merge(
            df_sim, df_val,
            on=time_col, suffixes=("_sim", "_val")
        )
        sim_cols   = {time_col: time_col}
        val_cols   = {time_col: time_col}
        shared = [c for c in df_sim.columns if c != time_col and c in df_val.columns]
        for c in shared:
            sim_cols[f"{c}_sim"] = c
            val_cols[f"{c}_val"] = c
        df_s = merged[[time_col] + [f"{c}_sim" for c in shared]].rename(columns=sim_cols)
        df_v = merged[[time_col] + [f"{c}_val" for c in shared]].rename(columns=val_cols)
        return df_s, df_v


def compute_error(sim: np.ndarray,
                  val: np.ndarray,
                  error_type: str) -> tuple[np.ndarray, str]:
    abs_err = sim - val
    if error_type == "absolute":
        return abs_err, "Absolute error (sim − val)"
    elif error_type == "relative":
        with np.errstate(divide="ignore", invalid="ignore"):
            rel_err = np.where(np.abs(val) > 1e-12, abs_err / np.abs(val) * 100.0, np.nan)
        return rel_err, "Relative error [%]"
    else:  # both → return absolute; caller will handle relative separately
        return abs_err, "Absolute error (sim − val)"


def stats_str(arr: np.ndarray) -> str:
    clean = arr[np.isfinite(arr)]
    if len(clean) == 0:
        return "N/A"
    return (f"max|e|={np.max(np.abs(clean)):.3g}  "
            f"μ={np.mean(clean):.3g}  "
            f"σ={np.std(clean):.3g}")


# ─── Per-column plot ──────────────────────────────────────────────────────────

def plot_column(time: np.ndarray,
                sim_vals: np.ndarray,
                val_vals: np.ndarray,
                col_name: str,
                error_type: str,
                out_path: str,
                dpi: int,
                fmt: str,
                show: bool):

    err, err_label = compute_error(sim_vals, val_vals, error_type)

    fig = plt.figure(figsize=(12, 5))
    gs  = GridSpec(1, 1, figure=fig)
    ax1 = fig.add_subplot(gs[0])
    ax2 = ax1.twinx()

    # ── Primary axis: sim & validation ──────────────────────────────────────
    l1, = ax1.plot(time, val_vals, color=VAL_COLOR, lw=LINE_WIDTH,
                   label="Validation", zorder=3)
    l2, = ax1.plot(time, sim_vals, color=SIM_COLOR,  lw=LINE_WIDTH,
                   ls="--", label="Aetherion Sim", zorder=4)

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel(col_name, color="#111827")
    ax1.tick_params(axis="y", labelcolor="#111827")

    # ── Secondary axis: error ────────────────────────────────────────────────
    l3, = ax2.plot(time, err, color=ERR_COLOR, lw=0.9, alpha=0.85,
                   label=err_label, zorder=2)
    ax2.axhline(0, color=ERR_COLOR, lw=0.6, ls=":", alpha=0.5)
    ax2.fill_between(time, err, 0, color=ERR_COLOR, alpha=FILL_ALPHA)
    ax2.set_ylabel(err_label, color=ERR_COLOR)
    ax2.tick_params(axis="y", labelcolor=ERR_COLOR)

    # ── Titles & legend ──────────────────────────────────────────────────────
    title = f"{col_name}"
    fig.suptitle(title, fontsize=11, fontweight="bold", y=1.01)
    ax1.set_title(f"Stats → {stats_str(err)}", fontsize=8, color="#6B7280", pad=4)

    lines  = [l1, l2, l3]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc="upper left", fontsize=8)

    fig.tight_layout()
    fname = os.path.join(out_path, f"{col_name.replace('/', '_')}.{fmt}")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
    return fname


# ─── Summary dashboard ───────────────────────────────────────────────────────

def plot_summary(time: np.ndarray,
                 columns: list[str],
                 sim_df: pd.DataFrame,
                 val_df: pd.DataFrame,
                 error_type: str,
                 out_path: str,
                 dpi: int,
                 fmt: str,
                 show: bool):
    """Multi-panel overview: up to 12 columns on a single figure."""
    n = min(len(columns), 12)
    cols_to_plot = columns[:n]
    ncols = 3
    nrows = int(np.ceil(n / ncols))

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(ncols * 5, nrows * 3.2),
                             squeeze=False)
    fig.suptitle("Aetherion Sim vs. Validation — Overview", fontsize=13,
                 fontweight="bold", y=1.01)

    for idx, col in enumerate(cols_to_plot):
        r, c = divmod(idx, ncols)
        ax1  = axes[r][c]
        ax2  = ax1.twinx()

        s = sim_df[col].values.astype(float)
        v = val_df[col].values.astype(float)
        e, elabel = compute_error(s, v, error_type)

        ax1.plot(time, v, color=VAL_COLOR, lw=1.2, label="Val")
        ax1.plot(time, s, color=SIM_COLOR, lw=1.2, ls="--", label="Sim")
        ax2.plot(time, e, color=ERR_COLOR, lw=0.8, alpha=0.7)
        ax2.fill_between(time, e, 0, color=ERR_COLOR, alpha=FILL_ALPHA)
        ax2.tick_params(axis="y", labelsize=6, labelcolor=ERR_COLOR)
        ax2.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2g"))

        ax1.set_title(col, fontsize=8, pad=3)
        ax1.set_xlabel("t [s]", fontsize=7)
        ax1.tick_params(labelsize=7)
        ax1.legend(fontsize=6, loc="upper left")

    # hide unused axes
    for idx in range(n, nrows * ncols):
        r, c = divmod(idx, ncols)
        axes[r][c].set_visible(False)

    fig.tight_layout()
    fname = os.path.join(out_path, f"overview_dashboard.{fmt}")
    fig.savefig(fname, dpi=dpi, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
    return fname


# ─── Error summary table ─────────────────────────────────────────────────────

def save_error_table(time: np.ndarray,
                     columns: list[str],
                     sim_df: pd.DataFrame,
                     val_df: pd.DataFrame,
                     out_path: str):
    rows = []
    for col in columns:
        s = sim_df[col].values.astype(float)
        v = val_df[col].values.astype(float)
        abs_e = s - v
        with np.errstate(divide="ignore", invalid="ignore"):
            rel_e = np.where(np.abs(v) > 1e-12, abs_e / np.abs(v) * 100.0, np.nan)
        clean_a = abs_e[np.isfinite(abs_e)]
        clean_r = rel_e[np.isfinite(rel_e)]
        rows.append({
            "column":            col,
            "abs_mean":          np.mean(clean_a)   if len(clean_a) else np.nan,
            "abs_std":           np.std(clean_a)    if len(clean_a) else np.nan,
            "abs_max":           np.max(np.abs(clean_a)) if len(clean_a) else np.nan,
            "rel_mean_%":        np.mean(clean_r)   if len(clean_r) else np.nan,
            "rel_std_%":         np.std(clean_r)    if len(clean_r) else np.nan,
            "rel_max_%":         np.max(np.abs(clean_r)) if len(clean_r) else np.nan,
        })
    df_err = pd.DataFrame(rows)
    csv_path = os.path.join(out_path, "error_summary.csv")
    df_err.to_csv(csv_path, index=False, float_format="%.6g")
    return csv_path


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compare Aetherion simulation vs. validation CSV data.")
    parser.add_argument("sim_csv",        help="Simulation CSV file path")
    parser.add_argument("validation_csv", help="Validation CSV file path")
    parser.add_argument("--output",       default="comparison_output",
                        help="Output directory (default: comparison_output)")
    parser.add_argument("--time-col",     default="time",
                        help="Name of the time column (default: time)")
    parser.add_argument("--columns",      nargs="+", default=None,
                        help="Specific columns to compare (default: all numeric)")
    parser.add_argument("--error-type",   default="absolute",
                        choices=["absolute", "relative", "both"],
                        help="Error metric type (default: absolute)")
    parser.add_argument("--interp",       action="store_true",
                        help="Interpolate validation to sim time grid")
    parser.add_argument("--no-show",      action="store_true",
                        help="Do not open interactive plot windows")
    parser.add_argument("--dpi",          type=int, default=150,
                        help="Figure DPI (default: 150)")
    parser.add_argument("--fmt",          default="png",
                        choices=["png", "pdf", "svg"],
                        help="Output figure format (default: png)")
    args = parser.parse_args()

    show = not args.no_show
    os.makedirs(args.output, exist_ok=True)

    # ── Load ─────────────────────────────────────────────────────────────────
    print(f"Loading sim:        {args.sim_csv}")
    print(f"Loading validation: {args.validation_csv}")
    df_sim = load_csv(args.sim_csv)
    df_val = load_csv(args.validation_csv)

    if args.time_col not in df_sim.columns:
        sys.exit(f"ERROR: time column '{args.time_col}' not found in sim CSV. "
                 f"Available: {list(df_sim.columns)}")
    if args.time_col not in df_val.columns:
        sys.exit(f"ERROR: time column '{args.time_col}' not found in validation CSV. "
                 f"Available: {list(df_val.columns)}")

    # ── Align ────────────────────────────────────────────────────────────────
    df_sim, df_val = align_dataframes(df_sim, df_val,
                                      args.time_col, args.interp)
    time = df_sim[args.time_col].values.astype(float)

    # ── Determine columns to compare ─────────────────────────────────────────
    shared_numeric = [
        c for c in df_sim.columns
        if c != args.time_col
        and c in df_val.columns
        and pd.api.types.is_numeric_dtype(df_sim[c])
        and pd.api.types.is_numeric_dtype(df_val[c])
    ]

    if args.columns:
        missing = [c for c in args.columns if c not in shared_numeric]
        if missing:
            print(f"WARNING: these columns not found / not numeric: {missing}")
        columns = [c for c in args.columns if c in shared_numeric]
    else:
        columns = shared_numeric

    if not columns:
        sys.exit("ERROR: No shared numeric columns to compare.")

    print(f"\nComparing {len(columns)} columns over {len(time)} time steps.")
    print(f"Output → {os.path.abspath(args.output)}\n")

    # ── Individual plots ──────────────────────────────────────────────────────
    for col in columns:
        s = df_sim[col].values.astype(float)
        v = df_val[col].values.astype(float)
        fname = plot_column(time, s, v, col,
                            args.error_type, args.output,
                            args.dpi, args.fmt, show)
        print(f"  saved: {fname}")

        # also plot relative error if --error-type both
        if args.error_type == "both":
            plot_column(time, s, v, f"{col}_relative",
                        "relative", args.output,
                        args.dpi, args.fmt, show)

    # ── Summary dashboard ─────────────────────────────────────────────────────
    print("\nGenerating overview dashboard…")
    dash = plot_summary(time, columns, df_sim, df_val,
                        args.error_type, args.output,
                        args.dpi, args.fmt, show)
    print(f"  saved: {dash}")

    # ── Error table ───────────────────────────────────────────────────────────
    csv_out = save_error_table(time, columns, df_sim, df_val, args.output)
    print(f"  saved: {csv_out}")

    print("\nDone.")


if __name__ == "__main__":
    main()