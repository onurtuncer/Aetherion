#!/usr/bin/env python3
"""Replay a step-7 log batch with EkfReplay and tabulate it over seeds.

Sensor fusion step 7 (TODO-sensor-fusion.md): every tuning change is judged
over seeds and over all cases, never on one realisation. The batch directory
holds one log set per case and seed, as Hemerion's `step7-log-batch.yml`
uploads them:

    <root>/case<case>/seed-<n>/{gps_fixes,imu_samples,baro_samples,f16_truth}.csv
    <root>/case<case>/seed-<n>/f16_truth.config

Everything after `--` is passed to EkfReplay unchanged, so a tuning is the
command line that produced its table:

    python scripts/step7_replay_batch.py --exe <EkfReplay> --root <root> \
        --out <dir> --label baro034 -- --baro-sigma 0.34

Writes <out>/<label>_runs.csv (one row per case, seed and convention) and
<out>/<label>.md (per case and convention: mean, sd, median, max and the
number of seeds above 2), and prints the Markdown. All figures are
second-half, as EkfReplay reports them.
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

NUM = r"([-+0-9.eE]+|nan|inf|-nan\(ind\)|-inf)"

# Summary line -> columns, in EkfReplay's own wording.
PATTERNS = {
    "updates": re.compile(
        rf"duration: {NUM} s, (\d+) IMU steps, (\d+) GNSS position, (\d+) GNSS velocity, (\d+) barometer updates"
    ),
    "nis": re.compile(rf"ANEES \(nav core, /9\) {NUM}, NIS/dim GNSS pos {NUM}, GNSS vel {NUM}, baro {NUM}"),
    "rms": re.compile(rf"second half RMS: position {NUM} m, velocity {NUM} m/s, tilt {NUM} deg, yaw {NUM} deg"),
    "blocks": re.compile(
        rf"ANEES \(/dim\) nav core {NUM}, gyro bias {NUM}, accel bias {NUM}, baro bias {NUM}, full {NUM}"
    ),
    "parts": re.compile(rf"attitude {NUM}, velocity {NUM}, position {NUM}"),
    "baro_end": re.compile(rf"baro bias at the end: estimate {NUM} \+- {NUM} m, truth {NUM} m"),
    "bias_sigma": re.compile(
        rf"gyro rms .*? against bias 1-sigma \({NUM} {NUM} {NUM}\); accel rms .*? against bias 1-sigma "
        rf"\({NUM} {NUM} {NUM}\)"
    ),
}

COLUMNS = [
    "case", "seed", "convention", "duration_s", "gnss_pos_updates", "gnss_vel_updates", "baro_updates",
    "anees_nav", "anees_attitude", "anees_velocity", "anees_position",
    "anees_gyro_bias", "anees_accel_bias", "anees_baro_bias", "anees_full",
    "nis_gnss_pos", "nis_gnss_vel", "nis_baro",
    "rms_pos_m", "rms_vel_m_s", "rms_tilt_deg", "rms_yaw_deg",
    "baro_est_m", "baro_sigma_m", "baro_truth_m",
    "sigma_bg_x", "sigma_bg_y", "sigma_bg_z", "sigma_ba_x", "sigma_ba_y", "sigma_ba_z",
]  # fmt: skip

# What the Markdown table shows: (column, heading, format).
TABLE = [
    ("anees_nav", "nav /9", "{:.2f}"),
    ("anees_attitude", "att /3", "{:.2f}"),
    ("anees_velocity", "vel /3", "{:.2f}"),
    ("anees_position", "pos /3", "{:.2f}"),
    ("anees_gyro_bias", "b_g /3", "{:.2f}"),
    ("anees_accel_bias", "b_a /3", "{:.2f}"),
    ("anees_baro_bias", "b_p /1", "{:.2f}"),
    ("anees_full", "full /16", "{:.2f}"),
    ("nis_gnss_pos", "NIS pos", "{:.2f}"),
    ("nis_gnss_vel", "NIS vel", "{:.2f}"),
    ("nis_baro", "NIS baro", "{:.3f}"),
    ("rms_pos_m", "pos m", "{:.3g}"),
    ("rms_vel_m_s", "vel m/s", "{:.3g}"),
    ("rms_tilt_deg", "tilt deg", "{:.3g}"),
    ("rms_yaw_deg", "yaw deg", "{:.3g}"),
]
ANEES_COLUMNS = [c for c, _, _ in TABLE if c.startswith("anees_")]


def parse_summary(text: str) -> dict[str, float]:
    row: dict[str, float] = {}

    def take(key: str, names: list[str]) -> None:
        m = PATTERNS[key].search(text)
        if not m:
            return
        for name, value in zip(names, m.groups()):
            try:
                row[name] = float(value)
            except ValueError:
                row[name] = float("nan")

    take("updates", ["duration_s", "_imu", "gnss_pos_updates", "gnss_vel_updates", "baro_updates"])
    take("nis", ["anees_nav", "nis_gnss_pos", "nis_gnss_vel", "nis_baro"])
    take("rms", ["rms_pos_m", "rms_vel_m_s", "rms_tilt_deg", "rms_yaw_deg"])
    take("blocks", ["_nav", "anees_gyro_bias", "anees_accel_bias", "anees_baro_bias", "anees_full"])
    take("parts", ["anees_attitude", "anees_velocity", "anees_position"])
    take("baro_end", ["baro_est_m", "baro_sigma_m", "baro_truth_m"])
    take("bias_sigma", ["sigma_bg_x", "sigma_bg_y", "sigma_bg_z", "sigma_ba_x", "sigma_ba_y", "sigma_ba_z"])
    return row


def seed_dirs(root: Path, cases: list[str]) -> list[tuple[str, int, Path]]:
    found = []
    for case in cases:
        for d in sorted((root / f"case{case}").glob("seed-*"), key=lambda p: int(p.name.split("-")[1])):
            if (d / "f16_truth.csv").exists():
                found.append((case, int(d.name.split("-")[1]), d))
    return found


def run_one(exe: Path, case: str, seed: int, d: Path, convention: str, extra: list[str]) -> dict:
    cmd = [str(exe), "--logs", str(d), "--truth", str(d / "f16_truth.csv"), "--config", "nav-baro-gnss16",
           "--convention", convention, "--csv", "off", *extra]  # fmt: skip
    p = subprocess.run(cmd, capture_output=True, text=True, check=False)
    row: dict = {"case": case, "seed": seed, "convention": convention}
    if p.returncode != 0:
        row["error"] = (p.stderr or p.stdout).strip().splitlines()[-1:] or ["failed"]
        return row
    row.update(parse_summary(p.stdout))
    return row


def stats_line(values: list[float]) -> tuple[float, float, float, float]:
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return statistics.fmean(values), sd, statistics.median(values), max(values)


def markdown(rows: list[dict], cases: list[str], conventions: list[str], command: str) -> str:
    out = [f"Command: `{command}`", ""]
    for stat in ("mean", "median", "max"):
        out.append(f"**{stat.capitalize()} over seeds, second half** (n = seeds; >2 = seeds with nav ANEES above 2)")
        out.append("")
        out.append("| Case | Conv | n | >2 | " + " | ".join(h for _, h, _ in TABLE) + " |")
        out.append("|---|---|---|---|" + "---|" * len(TABLE))
        for case in cases:
            for conv in conventions:
                sel = [r for r in rows if r["case"] == case and r["convention"] == conv and "anees_nav" in r]
                if not sel:
                    continue
                above = sum(1 for r in sel if r["anees_nav"] > 2.0)
                cells = []
                for col, _, fmt in TABLE:
                    vals = [r[col] for r in sel if col in r]
                    # A measurement that never updated (case 12 has no GNSS) reports 0: show a dash.
                    if not vals or (col.startswith("nis_") and all(v == 0.0 for v in vals)):
                        cells.append("—")
                        continue
                    mean, _, median, mx = stats_line(vals)
                    cells.append(fmt.format({"mean": mean, "median": median, "max": mx}[stat]))
                out.append(f"| {case} | {conv} | {len(sel)} | {above} | " + " | ".join(cells) + " |")
        out.append("")
    # Seeds above 2 in any block, for the acceptance band "no seed above 2".
    out.append("**Seeds above 2, per block ANEES**")
    out.append("")
    out.append("| Case | Conv | " + " | ".join(h for c, h, _ in TABLE if c in ANEES_COLUMNS) + " |")
    out.append("|---|---|" + "---|" * len(ANEES_COLUMNS))
    for case in cases:
        for conv in conventions:
            sel = [r for r in rows if r["case"] == case and r["convention"] == conv and "anees_nav" in r]
            if not sel:
                continue
            cells = [str(sum(1 for r in sel if r.get(col, 0.0) > 2.0)) for col in ANEES_COLUMNS]
            out.append(f"| {case} | {conv} | " + " | ".join(cells) + " |")
    out.append("")
    failed = [r for r in rows if "error" in r]
    if failed:
        out.append(f"**{len(failed)} replays failed:** " + ", ".join(f"{r['case']}/{r['seed']}/{r['convention']}" for r in failed))
        out.append("")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exe", required=True, type=Path, help="EkfReplay executable")
    ap.add_argument("--root", required=True, type=Path, help="batch directory (case<case>/seed-<n>/...)")
    ap.add_argument("--out", required=True, type=Path, help="directory for the tables")
    ap.add_argument("--label", required=True, help="name of this tuning; prefix of the files written")
    ap.add_argument("--cases", default="11,12,13.1,13.3,13.4")
    ap.add_argument("--conventions", default="ri,li")
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("extra", nargs="*", help="after --: options passed to EkfReplay")
    args = ap.parse_args()

    cases = args.cases.split(",")
    conventions = args.conventions.split(",")
    jobs = [(c, s, d, conv) for c, s, d in seed_dirs(args.root, cases) for conv in conventions]
    if not jobs:
        print(f"no log sets under {args.root}", file=sys.stderr)
        return 1
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(lambda j: run_one(args.exe, j[0], j[1], j[2], j[3], args.extra), jobs))

    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / f"{args.label}_runs.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    command = "EkfReplay --config nav-baro-gnss16 --convention <ri|li> " + " ".join(args.extra)
    md = markdown(rows, cases, conventions, command.strip())
    (args.out / f"{args.label}.md").write_text(md + "\n", encoding="utf-8")
    sys.stdout.reconfigure(encoding="utf-8")
    print(md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
