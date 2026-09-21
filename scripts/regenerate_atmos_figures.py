#!/usr/bin/env python3
"""
regenerate_atmos_figures.py
Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
SPDX-License-Identifier: MIT
---------------------------
Regenerate the per-scenario documentation figures for NASA TM-2015-218675
Atmospheric Scenarios 1, 2, 3, 6, 7, 8, 9 and 10.

Why this exists
---------------
Every other scenario family had a checked-in regeneration path -- the F-16
cases have ``plot_f16_s*_nasa02.py`` and the two-stage rocket has
``plot_atmos17_scenario17.py`` -- but scenarios 1--10 had none.  Their figures
under ``doc/_static/atmos01`` ... ``atmos10`` were therefore stranded at
whatever state they were generated in, and after the RKMK trivialisation fix
(``dexp^-1`` evaluated at ``-eta_i``) there was no way to bring them current
short of reconstructing the commands by hand.

These eight scenarios are uniform in a way the F-16 cases are not: the same
run parameters, the same 31-column SI schema, the same figure set.  A single
table-driven driver is therefore the honest shape for this, rather than eight
near-identical copies of one script.

What it does
------------
For each requested scenario:

  1. runs the corresponding Aetherion example at the parameters documented in
     ``doc/examples.rst`` (30 s, dt = 2 ms, one sample per 50 steps);
  2. compares the output against the NASA reference run nominated by the docs,
     ``Atmos_XX_sim_01_si_units.csv``, via ``compare_sim_validation.py``;
  3. writes the per-channel plots, the overview dashboard and
     ``error_summary.csv`` into ``doc/_static/atmosXX``.

The reference files ending in ``_si_units`` are pre-converted to SI and carry
exactly the same column names as the Aetherion output, so no unit conversion
or column mapping happens here.  That is deliberate: the unit handling lives in
``convert_to_si_sim_*.py`` and is not duplicated.

Usage
-----
    # everything
    python scripts/regenerate_atmos_figures.py

    # one scenario, or a few
    python scripts/regenerate_atmos_figures.py --scenario 6
    python scripts/regenerate_atmos_figures.py --scenario 1 2 3

    # reuse simulation CSVs from a previous invocation
    python scripts/regenerate_atmos_figures.py --sim-dir out/atmos_csv --skip-sim

    # see what would run, without running it
    python scripts/regenerate_atmos_figures.py --dry-run

Dependencies: numpy, pandas, matplotlib (via compare_sim_validation.py).
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# The example executables and compare_sim_validation.py both print non-ASCII
# (arrows, degree signs).  On a Windows console whose ANSI code page is not
# UTF-8 -- cp1254 on a Turkish-locale machine, for instance -- writing those to
# a captured pipe raises UnicodeEncodeError and the child dies with output
# already half-written.  Force UTF-8 on the children rather than making the
# shared scripts ASCII-only.
CHILD_ENV = {**os.environ, "PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1"}


def run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", env=CHILD_ENV)


REPO = Path(__file__).resolve().parent.parent
SCRIPTS = REPO / "scripts"
DATA = REPO / "data"
STATIC = REPO / "doc" / "_static"

# Run parameters, from the documented commands in doc/examples.rst.  All eight
# scenarios share them; writeInterval 50 at dt = 2 ms gives one sample per
# 0.1 s, which is the cadence of the reference files (301 rows over 30 s).
END_TIME = 30.0
TIME_STEP = 0.002
WRITE_INTERVAL = 50

# Build trees to search for the example executables, most specific first.
BUILD_DIRS = (
    REPO / "out" / "build" / "windows-release" / "src" / "Examples",
    REPO / "out" / "build" / "windows-debug" / "src" / "Examples",
    REPO / "build" / "gcc-release" / "src" / "Examples",
    REPO / "build" / "gcc-debug" / "src" / "Examples",
    REPO / "build" / "src" / "Examples",
)


class Scenario:
    """One NASA scenario: where its inputs, reference and outputs live."""

    def __init__(self, number: int, data_dir: str, executable: str, json_name: str):
        self.number = number
        self.tag = f"{number:02d}"
        self.data_dir = DATA / data_dir
        self.executable = executable
        self.json_name = json_name

    @property
    def input_json(self) -> Path:
        return self.data_dir / self.json_name

    @property
    def reference(self) -> Path:
        return self.data_dir / f"Atmos_{self.tag}_sim_01_si_units.csv"

    @property
    def out_dir(self) -> Path:
        return STATIC / f"atmos{self.tag}"

    def find_executable(self) -> Path | None:
        for base in BUILD_DIRS:
            for name in (f"{self.executable}.exe", self.executable):
                candidate = base / self.executable / name
                if candidate.exists():
                    return candidate
        return None

    def __str__(self) -> str:
        return f"scenario {self.number} ({self.executable})"


SCENARIOS = {
    s.number: s
    for s in (
        Scenario(1,  "Atmos_01_DroppedSphere",          "DraglessSphere",
                 "nasa_2015_scenario1_dragless_sphere.json"),
        Scenario(2,  "Atmos_02_TumblingBrickNoDamping", "TumblingBrickNoDamping",
                 "nasa_2015_scenario2_tumbling_brick_no_damping.json"),
        Scenario(3,  "Atmos_03_TumblingBrickWithDamping", "TumblingBrickWithDamping",
                 "nasa_2015_scenario3_tumbling_brick_with_damping.json"),
        Scenario(6,  "Atmos_06_SphereWithDrag",         "SphereWithAtmosphericDrag",
                 "nasa_2015_scenario6_sphere_with_drag.json"),
        Scenario(7,  "Atmos_07_DroppedSphereSteadyWind", "DroppedSphereSteadyWind",
                 "nasa_2015_scenario7_dropped_sphere_steady_wind.json"),
        Scenario(8,  "Atmos_08_DroppedSphere2DWindShear", "DroppedSphere2DWindShear",
                 "nasa_2015_scenario8_dropped_sphere_2d_wind_shear.json"),
        Scenario(9,  "Atmos_09_EastwardCannonball",     "EastwardCannonball",
                 "nasa_2015_scenario9_eastward_cannonball.json"),
        Scenario(10, "Atmos_10_NorthwardCannonball",    "NorthwardCannonball",
                 "nasa_2015_scenario10_northward_cannonball.json"),
    )
}


def run_simulation(sc: Scenario, exe: Path, out_csv: Path, dry_run: bool) -> bool:
    cmd = [
        str(exe),
        "--inputFileName",  str(sc.input_json),
        "--outputFileName", str(out_csv),
        "--startTime",      "0.0",
        "--endTime",        str(END_TIME),
        "--timeStep",       str(TIME_STEP),
        "--writeInterval",  str(WRITE_INTERVAL),
    ]
    if dry_run:
        print("   would run:", " ".join(cmd))
        return True

    proc = run(cmd)
    if proc.returncode != 0:
        print(f"   ERROR: {sc.executable} exited {proc.returncode}")
        print((proc.stderr or proc.stdout or "").strip()[-800:])
        return False
    if not out_csv.exists():
        print(f"   ERROR: {sc.executable} produced no output at {out_csv}")
        return False
    return True


def run_comparison(sc: Scenario, sim_csv: Path, dry_run: bool) -> bool:
    cmd = [
        sys.executable, str(SCRIPTS / "compare_sim_validation.py"),
        str(sim_csv), str(sc.reference),
        "--output", str(sc.out_dir),
        "--interp", "--no-show",
    ]
    if dry_run:
        print("   would run:", " ".join(cmd))
        return True

    proc = run(cmd)
    if proc.returncode != 0:
        print(f"   ERROR: comparison exited {proc.returncode}")
        print((proc.stderr or proc.stdout or "").strip()[-800:])
        return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Regenerate doc/_static/atmosXX figures for NASA scenarios 1-10.")
    ap.add_argument("--scenario", type=int, nargs="+", choices=sorted(SCENARIOS),
                    help="scenario numbers to regenerate (default: all)")
    ap.add_argument("--sim-dir", type=Path, default=None,
                    help="directory for simulation CSVs (default: a temporary one)")
    ap.add_argument("--skip-sim", action="store_true",
                    help="reuse existing CSVs in --sim-dir instead of re-running")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the commands without executing them")
    args = ap.parse_args()

    if args.skip_sim and args.sim_dir is None:
        ap.error("--skip-sim requires --sim-dir")

    wanted = args.scenario or sorted(SCENARIOS)

    tmp = None
    if args.sim_dir is not None:
        sim_dir = args.sim_dir
        sim_dir.mkdir(parents=True, exist_ok=True)
    else:
        tmp = tempfile.mkdtemp(prefix="aetherion_atmos_")
        sim_dir = Path(tmp)

    ok, failed = [], []
    try:
        for number in wanted:
            sc = SCENARIOS[number]
            print(f"\n=== {sc} -> {sc.out_dir.relative_to(REPO)} ===")

            if not sc.input_json.exists():
                print(f"   SKIP: missing input {sc.input_json}")
                failed.append(number)
                continue
            if not sc.reference.exists():
                print(f"   SKIP: missing reference {sc.reference}")
                failed.append(number)
                continue

            sim_csv = sim_dir / f"atmos_{sc.tag}_output.csv"

            if args.skip_sim:
                if not sim_csv.exists():
                    print(f"   SKIP: --skip-sim but {sim_csv} is absent")
                    failed.append(number)
                    continue
                print(f"   reusing {sim_csv}")
            else:
                exe = sc.find_executable()
                if exe is None:
                    print(f"   SKIP: no built executable for {sc.executable}; "
                          f"build it first, e.g.\n"
                          f"      cmake --build out/build/windows-release "
                          f"--target {sc.executable}")
                    failed.append(number)
                    continue
                print(f"   running {exe.name}")
                if not run_simulation(sc, exe, sim_csv, args.dry_run):
                    failed.append(number)
                    continue

            print(f"   comparing against {sc.reference.name}")
            if not run_comparison(sc, sim_csv, args.dry_run):
                failed.append(number)
                continue

            if not args.dry_run:
                n = len(list(sc.out_dir.glob("*.png")))
                print(f"   wrote {n} figures + error_summary.csv")
            ok.append(number)
    finally:
        if tmp is not None:
            shutil.rmtree(tmp, ignore_errors=True)

    print(f"\nregenerated: {ok if ok else 'none'}")
    if failed:
        print(f"failed/skipped: {failed}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
