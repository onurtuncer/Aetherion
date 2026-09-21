#!/usr/bin/env python3
"""
F16Autopilot FMU closed-loop test — FMPy as the FMI 2.0 Co-Simulation master.

F16Plant and F16Autopilot are wired port-for-port (every fb.* input from the
identically named out.* output, ctrl.* back to ctrl.*) and stepped in lock-step.

Tests
─────
1. Initialisation-mode inputs are honoured
   Write cmd.* and fb.* during initialisation mode and read them back after
   fmi2ExitInitializationMode.  Guards against the FMU re-seeding its inputs to
   the Scenario-11 trim point, which silently replaced a 10 000 ft altitude
   command with 10 013 ft.

2. Course hold (circumnavigate = false)
   Scenario-11 defaults, 30 s.  Altitude, airspeed and course stay at trim and
   the navigator ports are inert.

3. Scenario 15 — circle the North Pole        (circumnavigate, circlePoleSW = 1)
4. Scenario 16 — circle the equator/date line (circumnavigate, circlePoleSW = 0)
   180 s against the NASA TM-2015-218675 reference trajectory.  The vehicle must
   capture the 3 nmi circle, hold 10 000 ft, and end within tolerance of the
   reference distance-from-target and bank angle.

Usage
─────
  pip install fmpy numpy
  python test_f16autopilot.py
  python test_f16autopilot.py --build-dir path/to/build/tree
"""

import argparse
import csv
import math
import pathlib
import shutil
import sys
import tempfile

try:
    from fmpy import read_model_description, extract
    from fmpy.fmi2 import FMU2Slave
except ImportError:
    sys.exit(
        "FMPy not found.  Install it with:\n"
        "  pip install fmpy\n"
    )

# ── Default paths ─────────────────────────────────────────────────────────────
_SCRIPT_DIR        = pathlib.Path(__file__).resolve().parent
_REPO_ROOT         = _SCRIPT_DIR.parents[2]
_DEFAULT_BUILD_DIR = _REPO_ROOT / "out" / "build" / "windows-debug"
_DATA_DIR          = _REPO_ROOT / "data"

# ── Constants ─────────────────────────────────────────────────────────────────
_FT_M        = 0.3048
_KT_MS       = 0.514444
_RHO_SL      = 1.225
_NMI_FT      = 6076.12
_DEG_TO_FT   = 60.0 * _NMI_FT          # F16_gnc.dml deg_to_ft
_RADIUS_FT   = 3.0 * _NMI_FT           # F16_gnc.dml circleRadius_ft

# Plant output → autopilot input, port for port
_FEEDBACK = [
    ("out.alt_m",     "fb.alt_m"),
    ("out.vt_m_s",    "fb.vt_m_s"),
    ("out.rho_kg_m3", "fb.rho_kg_m3"),
    ("out.alpha_deg", "fb.alpha_deg"),
    ("out.beta_deg",  "fb.beta_deg"),
    ("out.roll_rad",  "fb.roll_rad"),
    ("out.pitch_rad", "fb.pitch_rad"),
    ("out.yaw_rad",   "fb.yaw_rad"),
    ("out.p_rad_s",   "fb.p_rad_s"),
    ("out.q_rad_s",   "fb.q_rad_s"),
    ("out.r_rad_s",   "fb.r_rad_s"),
    ("out.lat_deg",   "fb.lat_deg"),
    ("out.lon_deg",   "fb.lon_deg"),
]
_CONTROLS = ["ctrl.el_deg", "ctrl.ail_deg", "ctrl.rdr_deg", "ctrl.pwr_pct"]

# Scenarios 15 and 16 share the flight condition; only position, heading and
# the navigator switch differ.
_CIRCLE_VT_FPS = 563.643
_CIRCLE_ALT_FT = 10000.0


# ─────────────────────────────────────────────────────────────────────────────
# FMU wrapper
# ─────────────────────────────────────────────────────────────────────────────
class Fmu:
    """One instantiated FMU, addressed by variable name."""

    def __init__(self, fmu_path: pathlib.Path, instance_name: str):
        self.unzipdir = tempfile.mkdtemp(prefix=f"{instance_name}_")
        extract(str(fmu_path), self.unzipdir)
        self.md = read_model_description(str(fmu_path))
        self.vars = {v.name: v for v in self.md.modelVariables}
        self.slave = FMU2Slave(
            guid=self.md.guid,
            unzipDirectory=self.unzipdir,
            modelIdentifier=self.md.coSimulation.modelIdentifier,
            instanceName=instance_name,
        )
        self.slave.instantiate()
        self.slave.setupExperiment(startTime=0.0)
        self.slave.enterInitializationMode()

    def set(self, name: str, value):
        v = self.vars[name]
        if v.type == "Boolean":
            self.slave.setBoolean([v.valueReference], [bool(value)])
        else:
            self.slave.setReal([v.valueReference], [float(value)])

    def get(self, name: str) -> float:
        return self.slave.getReal([self.vars[name].valueReference])[0]

    def close(self):
        try:
            self.slave.terminate()
            self.slave.freeInstance()
        finally:
            shutil.rmtree(self.unzipdir, ignore_errors=True)


def _keas_kt(vt_m_s: float, rho_kg_m3: float) -> float:
    return vt_m_s * math.sqrt(rho_kg_m3 / _RHO_SL) / _KT_MS


def _fly(plant: Fmu, ap: Fmu, dt: float, t_end: float, on_step=None):
    """Lock-step co-simulation: plant → autopilot → plant, zero-order hold."""
    t = 0.0
    while t < t_end - 1e-10:
        for src, dst in _FEEDBACK:
            ap.set(dst, plant.get(src))
        ap.slave.doStep(t, dt, True)
        for name in _CONTROLS:
            plant.set(name, ap.get(name))
        plant.slave.doStep(t, dt, True)
        t += dt
        if on_step:
            on_step(t)


def _report(ok: bool, failures) -> bool:
    if ok:
        print("\n  PASS")
    else:
        print(f"\n  FAIL — {len(failures)} check(s):")
        for f in failures[:10]:
            print(f"    {f}")
    return ok


# ─────────────────────────────────────────────────────────────────────────────
# Test 1 — Initialisation-mode inputs are honoured
# ─────────────────────────────────────────────────────────────────────────────
def test_init_inputs_honoured(ap_path: pathlib.Path) -> bool:
    print(f"\n{'─'*60}\nTest 1: initialisation-mode inputs are honoured\n{'─'*60}")

    written = {
        "cmd.altCmd_ft":    10000.0,
        "cmd.keasCmd_kt":   286.5,
        "cmd.circlePoleSW": 1.0,
        "fb.alt_m":         3048.0,
        "fb.lat_deg":       89.95,
        "fb.lon_deg":      -45.0,
    }

    ap = Fmu(ap_path, "F16Autopilot_init")
    failures = []
    try:
        for name, value in written.items():
            ap.set(name, value)
        ap.slave.exitInitializationMode()
        for name, value in written.items():
            got = ap.get(name)
            print(f"  {name:<18} wrote {value:>10.3f}   read {got:>10.3f}")
            if got != value:
                failures.append(f"{name}: wrote {value}, read back {got}")
    finally:
        ap.close()
    return _report(not failures, failures)


# ─────────────────────────────────────────────────────────────────────────────
# Test 2 — Course hold, navigator ports inert
# ─────────────────────────────────────────────────────────────────────────────
def test_course_hold(plant_path, ap_path, dt: float, t_end: float) -> bool:
    print(f"\n{'─'*60}\nTest 2: course hold (circumnavigate = false)  "
          f"dt={dt} s  t_end={t_end} s\n{'─'*60}")

    def run(circle_pole_sw: float):
        plant = Fmu(plant_path, "F16Plant_hold")
        ap    = Fmu(ap_path,    "F16Autopilot_hold")
        try:
            plant.slave.exitInitializationMode()
            ap.set("cmd.keasCmd_kt",
                   _keas_kt(plant.get("out.vt_m_s"), plant.get("out.rho_kg_m3")))
            ap.set("cmd.circlePoleSW", circle_pole_sw)
            ap.slave.exitInitializationMode()
            _fly(plant, ap, dt, t_end)
            return {n: plant.get(n) for n in
                    ("out.alt_m", "out.vt_m_s", "out.yaw_rad", "out.lat_deg", "out.lon_deg")}
        finally:
            plant.close()
            ap.close()

    a = run(0.0)
    b = run(1.0)

    alt_err_ft = abs(a["out.alt_m"] / _FT_M - 10013.0)
    vt_err     = abs(a["out.vt_m_s"] - 565.685 * _FT_M)
    yaw_err    = abs(math.degrees(a["out.yaw_rad"]) - 45.0)
    print(f"  after {t_end:.0f} s:  alt err = {alt_err_ft:.3f} ft   "
          f"vt err = {vt_err:.4f} m/s   yaw err = {yaw_err:.4f} deg")

    failures = []
    if alt_err_ft > 5.0:  failures.append(f"altitude drifted {alt_err_ft:.2f} ft")
    if vt_err     > 0.5:  failures.append(f"airspeed drifted {vt_err:.3f} m/s")
    if yaw_err    > 1.0:  failures.append(f"heading drifted {yaw_err:.3f} deg")
    if a != b:
        failures.append("cmd.circlePoleSW changed the course-hold trajectory")
    else:
        print("  cmd.circlePoleSW has no effect on the trajectory (bitwise)")
    return _report(not failures, failures)


# ─────────────────────────────────────────────────────────────────────────────
# Tests 3/4 — Circumnavigation against the NASA reference
# ─────────────────────────────────────────────────────────────────────────────
def _distance_from_target_ft(circle_pole_sw: float, lat_deg: float, lon_deg: float) -> float:
    """Distance from the circle centre, with the navigator's own flat-Earth metric."""
    if circle_pole_sw > 0.5:
        return _DEG_TO_FT * (90.0 - lat_deg)
    east_deg = lon_deg - 180.0 if lon_deg > 0.0 else lon_deg + 180.0
    north_ft = lat_deg * _DEG_TO_FT
    east_ft  = east_deg * _DEG_TO_FT * math.cos(math.radians(lat_deg))
    return math.hypot(north_ft, east_ft)


def _load_reference(csv_path: pathlib.Path):
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    return rows[-1]


def test_circle(plant_path, ap_path, dt: float, *, label: str, circle_pole_sw: float,
                lat0: float, lon0: float, heading0: float, ref_csv: pathlib.Path) -> bool:
    t_end = 180.0
    print(f"\n{'─'*60}\n{label}  dt={dt} s  t_end={t_end} s\n{'─'*60}")

    plant = Fmu(plant_path, "F16Plant_circle")
    ap    = Fmu(ap_path,    "F16Autopilot_circle")
    failures = []
    try:
        for name, value in (("vt0_fps", _CIRCLE_VT_FPS), ("alt0_ft", _CIRCLE_ALT_FT),
                            ("lat0_deg", lat0), ("lon0_deg", lon0),
                            ("heading0_deg", heading0), ("roll0_deg", 0.0)):
            plant.set(name, value)
        plant.slave.exitInitializationMode()

        print(f"  plant trim: alpha = {plant.get('out.alpha_deg'):.4f} deg   "
              f"el = {plant.get('ctrl.el_deg'):.4f} deg   pwr = {plant.get('ctrl.pwr_pct'):.4f} %")

        ap.set("circumnavigate",   True)
        ap.set("cmd.circlePoleSW", circle_pole_sw)
        ap.set("cmd.altCmd_ft",    _CIRCLE_ALT_FT)
        ap.set("cmd.keasCmd_kt",
               _keas_kt(plant.get("out.vt_m_s"), plant.get("out.rho_kg_m3")))
        for src, dst in _FEEDBACK:
            ap.set(dst, plant.get(src))
        ap.slave.exitInitializationMode()

        track = {"max_alt_err_ft": 0.0, "late_radius_err_ft": 0.0}

        def on_step(t):
            alt_err = abs(plant.get("out.alt_m") / _FT_M - _CIRCLE_ALT_FT)
            track["max_alt_err_ft"] = max(track["max_alt_err_ft"], alt_err)
            if t >= 90.0:   # circle captured by then in the reference
                r = _distance_from_target_ft(circle_pole_sw,
                                             plant.get("out.lat_deg"), plant.get("out.lon_deg"))
                track["late_radius_err_ft"] = max(track["late_radius_err_ft"],
                                                  abs(r - _RADIUS_FT))

        _fly(plant, ap, dt, t_end, on_step)

        ref      = _load_reference(ref_csv)
        ref_r    = _distance_from_target_ft(circle_pole_sw,
                                            float(ref["latitude_deg"]), float(ref["longitude_deg"]))
        ref_roll = float(ref["eulerAngle_deg_Roll"])
        ref_alt  = float(ref["altitudeMsl_ft"])

        end_r    = _distance_from_target_ft(circle_pole_sw,
                                            plant.get("out.lat_deg"), plant.get("out.lon_deg"))
        end_roll = math.degrees(plant.get("out.roll_rad"))
        end_alt  = plant.get("out.alt_m") / _FT_M

        print(f"  at t = {t_end:.0f} s          this run      NASA sim_05")
        print(f"    distance from target  {end_r:>10.1f} ft {ref_r:>10.1f} ft   "
              f"(circle = {_RADIUS_FT:.1f} ft)")
        print(f"    bank angle            {end_roll:>10.3f} deg {ref_roll:>9.3f} deg")
        print(f"    altitude              {end_alt:>10.2f} ft {ref_alt:>10.2f} ft")
        print(f"  max |alt − cmd| over the run      = {track['max_alt_err_ft']:.2f} ft")
        print(f"  max |radius − 3 nmi| for t ≥ 90 s = {track['late_radius_err_ft']:.1f} ft")

        if abs(end_r - ref_r)       > 50.0:  failures.append(f"radius {end_r:.0f} ft vs reference {ref_r:.0f} ft")
        if abs(end_roll - ref_roll) > 0.25:  failures.append(f"bank {end_roll:.2f} deg vs reference {ref_roll:.2f} deg")
        if abs(end_alt - ref_alt)   > 1.0:   failures.append(f"altitude {end_alt:.1f} ft vs reference {ref_alt:.1f} ft")
        if track["max_alt_err_ft"]  > 25.0:  failures.append(f"altitude excursion {track['max_alt_err_ft']:.1f} ft")
        if track["late_radius_err_ft"] > 0.1 * _RADIUS_FT:
            failures.append(f"circle not held: radius error {track['late_radius_err_ft']:.0f} ft after t = 90 s")
    finally:
        plant.close()
        ap.close()
    return _report(not failures, failures)


# ─────────────────────────────────────────────────────────────────────────────
def main() -> int:
    # The report uses box-drawing characters; a legacy Windows code page cannot
    # encode them once stdout is redirected.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build-dir", type=pathlib.Path, default=_DEFAULT_BUILD_DIR,
                        help="CMake build tree holding models/fmi2/<Model>/<Model>.fmu")
    parser.add_argument("--dt", type=float, default=0.02,
                        help="communication step [s] (the LQR loop is stiff; 0.1 diverges)")
    args = parser.parse_args()

    fmu_dir    = args.build_dir / "models" / "fmi2"
    plant_path = fmu_dir / "F16Plant" / "F16Plant.fmu"
    ap_path    = fmu_dir / "F16Autopilot" / "F16Autopilot.fmu"
    for p in (plant_path, ap_path):
        if not p.exists():
            sys.exit(f"FMU not found: {p}")

    results = [
        test_init_inputs_honoured(ap_path),
        test_course_hold(plant_path, ap_path, args.dt, 30.0),
        test_circle(plant_path, ap_path, args.dt,
                    label="Test 3: Scenario 15 — circle the North Pole",
                    circle_pole_sw=1.0, lat0=89.95, lon0=-45.0, heading0=90.0,
                    ref_csv=_DATA_DIR / "Atmos_15_CircleNorthPole" / "Atmos_15_sim_05.csv"),
        test_circle(plant_path, ap_path, args.dt,
                    label="Test 4: Scenario 16 — circle the equator/date-line crossing",
                    circle_pole_sw=0.0, lat0=0.0, lon0=-179.95, heading0=0.0,
                    ref_csv=_DATA_DIR / "Atmos_16_CircleEquatorDateLine" / "Atmos_16_sim_05.csv"),
    ]

    print(f"\n{'═'*60}\n{sum(results)}/{len(results)} tests passed\n{'═'*60}")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
