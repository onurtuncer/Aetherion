#!/usr/bin/env python3
"""Figures for doc/sensor_fusion.rst, "Tuning and observability (step 7)".

    python scripts/plot_step7_findings.py --exe <EkfReplay> --root <logs> \
        --out doc/_static/sensor_fusion [--seed 2]

<logs> is a step-7 batch directory (see step7_replay_batch.py). Writes

  step7_imu_stamp.png   the IMU samples against the truth log at a range of
                        time shifts (case 13.4), and what the offset does to
                        the accelerometer-bias estimate
  step7_case12.png      case 12, barometer only: tilt error against the
                        filter's 1-sigma, and the first- and second-order
                        vertical accelerations a gyro-bias error produces
"""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

G = 9.80665
EARTH_RADIUS_M = 6.371e6


def read_truth(d: Path) -> pd.DataFrame:
    tr = pd.read_csv(d / "f16_truth.csv")
    tr.columns = [c.split("[")[0].strip() for c in tr.columns]
    return tr


def replay(exe: Path, d: Path, tmp: Path, name: str, extra: list[str]) -> pd.DataFrame:
    prefix = tmp / name
    cmd = [str(exe), "--logs", str(d), "--truth", str(d / "f16_truth.csv"), "--config", "nav-baro-gnss16",
           "--every", "0.2", "--output", str(prefix), *extra]  # fmt: skip
    subprocess.run(cmd, check=True, capture_output=True)
    return pd.read_csv(f"{prefix}_states.csv")


def stamp_figure(exe: Path, d: Path, tmp: Path, out: Path) -> None:
    tr, im = read_truth(d), pd.read_csv(d / "imu_samples.csv")
    t, ti = tr["time"].to_numpy(), im["part_time_s"].to_numpy()
    taus = np.arange(-0.03, 0.0301, 0.0025)
    pairs = [("accel_x_mps2", "imu::f_x_mps2", "imu::accel_bias_x_mps2", "accelerometer x"),
             ("gyro_x_rad_s", "f16::out.p_rad_s", "imu::gyro_bias_x_rad_s", "gyro x")]  # fmt: skip

    fig, (a0, a1) = plt.subplots(1, 2, figsize=(11, 4))
    for sample, truth, bias, label in pairs:
        y, x, b = im[sample].to_numpy(), tr[truth].to_numpy(), tr[bias].to_numpy()
        sd = np.array([(y - np.interp(ti + tau, t, x) - np.interp(ti + tau, t, b)).std() for tau in taus])
        a0.semilogy(taus * 1e3, sd / sd.min(), marker="o", ms=3, label=label)
    a0.axvline(-10, color="k", lw=0.8, ls=":")
    a0.set_xlabel("shift added to the IMU stamp [ms]")
    a0.set_ylabel("residual sd / sensor noise floor")
    a0.set_title("IMU sample minus truth, case 13.4")
    a0.legend()
    a0.grid(True, which="both", alpha=0.3)

    for extra, label, color in [([], "as written", "C3"), (["--imu-time-shift", "-0.01"], "stamps − 10 ms", "C0")]:
        s = replay(exe, d, tmp, "stamp" + str(len(extra)), extra)
        a1.plot(s.t_s, s.err_12 * 1e3, color=color, label=f"error, {label}")
        a1.fill_between(s.t_s, -3e3 * s.sigma_12, 3e3 * s.sigma_12, color=color, alpha=0.12, lw=0)
    a1.axhline(0, color="k", lw=0.6)
    a1.set_xlabel("time [s]")
    a1.set_ylabel("accel bias x, truth − estimate [mm/s²]")
    a1.set_title("Bands: the filter's ±3σ")
    a1.set_ylim(-150, 150)
    a1.legend(loc="upper right")
    a1.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "step7_imu_stamp.png", dpi=150)
    plt.close(fig)


def case12_figure(exe: Path, d: Path, tmp: Path, out: Path) -> None:
    tr = read_truth(d)
    v = float(tr["f16::out.vt_m_s"].iloc[0])
    kappa = 2.0 * v / EARTH_RADIUS_M  # vertical acceleration per m/s of along-track velocity error
    s = replay(exe, d, tmp, "c12", ["--convention", "li", "--imu-time-shift", "-0.01"])
    tilt = np.hypot(s.err_pitch_deg, s.err_roll_deg)
    sig = np.degrees(np.hypot(s.sigma_0, s.sigma_1))

    fig, (a0, a1) = plt.subplots(1, 2, figsize=(11, 4))
    a0.semilogy(s.t_s, tilt, color="C3", label="tilt error")
    a0.semilogy(s.t_s, sig, color="C0", label="filter 1σ (attitude x, y)")
    a0.set_xlabel("time [s]")
    a0.set_ylabel("[deg]")
    a0.set_title("Case 12, barometer only (left-invariant)")
    a0.legend()
    a0.grid(True, which="both", alpha=0.3)

    t = np.linspace(1, 200, 200)
    for b, ls in [(1e-3, "-"), (2e-4, "--"), (2e-5, ":")]:
        a1.loglog(t, kappa * G * b * t**2 / 2, color="C0", ls=ls, label=f"first order, b = {b:g} rad/s")
        a1.loglog(t, G * (b * t) ** 2 / 2, color="C3", ls=ls, label=f"second order, b = {b:g} rad/s")
    a1.set_xlabel("time since the gyro bias went unobserved [s]")
    a1.set_ylabel("vertical acceleration error [m/s²]")
    a1.set_title(f"κ g b t²/2 against g (b t)²/2, κ = 2v/R = {kappa:.2g} 1/s")
    a1.legend(fontsize=7)
    a1.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "step7_case12.png", dpi=150)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", required=True, type=Path)
    ap.add_argument("--root", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--seed", type=int, default=2)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        stamp_figure(args.exe, args.root / "case13.4" / f"seed-{args.seed}", Path(tmp), args.out)
        case12_figure(args.exe, args.root / "case12" / f"seed-{args.seed}", Path(tmp), args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
