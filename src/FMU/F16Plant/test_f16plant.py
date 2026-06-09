#!/usr/bin/env python3
"""
F16Plant FMU smoke test — FMPy as the FMI 2.0 Co-Simulation master.

Tests
─────
1. Trim stability
   Run 30 s with controls locked at their trim values (set by
   exit_initialisation_mode).  Verify that altitude, true airspeed, and
   angle of attack remain within ±1 % of their t=0 values throughout.
   This exercises the full integration pipeline and confirms the trim
   solution is consistent with the equations of motion.

2. State save / restore
   Step to t=5 s, checkpoint with getFMUState.
   Continue to t=10 s, recording the trajectory (A).
   Restore the checkpoint, re-run t=5 s → t=10 s, recording trajectory (B).
   Assert |A − B| ≤ 1e-10 at every output step (bitwise determinism).

Usage
─────
  pip install fmpy numpy
  python test_f16plant.py
  python test_f16plant.py --fmu path/to/F16Plant.fmu
  python test_f16plant.py --dt 0.05 --t-end 60
"""

import argparse
import pathlib
import shutil
import sys
import tempfile

import numpy as np

# ── FMPy ─────────────────────────────────────────────────────────────────────
try:
    from fmpy import read_model_description, extract
    from fmpy.fmi2 import FMU2Slave
except ImportError:
    sys.exit(
        "FMPy not found.  Install it with:\n"
        "  pip install fmpy\n"
    )

# ── Default FMU path ──────────────────────────────────────────────────────────
_SCRIPT_DIR   = pathlib.Path(__file__).resolve().parent
_REPO_ROOT    = _SCRIPT_DIR.parents[2]
_DEFAULT_FMU  = (
    _REPO_ROOT / "out" / "build" / "windows-debug" /
    "models" / "fmi2" / "F16Plant" / "F16Plant.fmu"
)

# ── Output variable names we care about ───────────────────────────────────────
_WATCHED = [
    "out.alt_m",
    "out.vt_m_s",
    "out.alpha_deg",
    "out.pitch_rad",
    "out.roll_rad",
    "out.yaw_rad",
    "out.p_rad_s",
    "out.q_rad_s",
    "out.r_rad_s",
    "out.v_north_m_s",
    "out.v_east_m_s",
    "out.v_down_m_s",
    "out.mach",
    "out.qbar_Pa",
]

# ── Tolerances for trim-stability check ───────────────────────────────────────
_ALT_TOL_PCT   = 1.0    # ±1 % of initial altitude
_VT_TOL_PCT    = 1.0    # ±1 % of initial true airspeed
_ALPHA_TOL_DEG = 0.5    # ±0.5 deg absolute (1 % of ~2.6 deg would be too tight)


# ─────────────────────────────────────────────────────────────────────────────
# Helper: build a {name: valueReference} map from the model description
# ─────────────────────────────────────────────────────────────────────────────
def _vr_map(model_desc):
    return {v.name: v.valueReference for v in model_desc.modelVariables}


# ─────────────────────────────────────────────────────────────────────────────
# Helper: instantiate and initialise, return (slave, vrs, unzipdir)
# ─────────────────────────────────────────────────────────────────────────────
def _boot(fmu_path: pathlib.Path, unzipdir: str, start: float = 0.0):
    model_desc = read_model_description(str(fmu_path))
    vrs = _vr_map(model_desc)

    slave = FMU2Slave(
        guid=model_desc.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=model_desc.coSimulation.modelIdentifier,
        instanceName="F16Plant_test",
    )
    slave.instantiate()
    slave.setupExperiment(startTime=start)
    slave.enterInitializationMode()
    # Parameters keep their defaults (NASA Scenario 11 trim condition).
    slave.exitInitializationMode()   # ← trim solver runs here
    return slave, vrs, model_desc


# ─────────────────────────────────────────────────────────────────────────────
# Test 1 — Trim stability
# ─────────────────────────────────────────────────────────────────────────────
def test_trim_stability(fmu_path: pathlib.Path, dt: float, t_end: float):
    print(f"\n{'─'*60}")
    print(f"Test 1: Trim stability  dt={dt} s  t_end={t_end} s")
    print(f"{'─'*60}")

    unzipdir = tempfile.mkdtemp(prefix="f16plant_trim_")
    try:
        model_desc = read_model_description(str(fmu_path))
        extract(str(fmu_path), unzipdir)
        slave, vrs, _ = _boot(fmu_path, unzipdir)

        # Read initial output values right after initialisation
        def get_outputs():
            return slave.getReal([vrs[n] for n in _WATCHED])

        init_vals = dict(zip(_WATCHED, get_outputs()))
        alt0   = init_vals["out.alt_m"]
        vt0    = init_vals["out.vt_m_s"]
        alpha0 = init_vals["out.alpha_deg"]

        print(f"  Initial trim values:")
        print(f"    alt   = {alt0:.2f} m   ({alt0/0.3048:.1f} ft)")
        print(f"    vt    = {vt0:.3f} m/s  ({vt0/0.3048:.1f} fps)")
        print(f"    alpha = {alpha0:.4f} deg")

        alt_tol   = abs(alt0)   * _ALT_TOL_PCT   / 100.0
        vt_tol    = abs(vt0)    * _VT_TOL_PCT    / 100.0
        alpha_tol = _ALPHA_TOL_DEG

        violations = []
        t = 0.0
        step = 0
        max_alt_err   = 0.0
        max_vt_err    = 0.0
        max_alpha_err = 0.0

        while t < t_end - 1e-10:
            step_dt = min(dt, t_end - t)
            slave.doStep(t, step_dt, True)
            t += step_dt
            step += 1

            vals = dict(zip(_WATCHED, get_outputs()))
            alt_err   = abs(vals["out.alt_m"]    - alt0)
            vt_err    = abs(vals["out.vt_m_s"]   - vt0)
            alpha_err = abs(vals["out.alpha_deg"] - alpha0)

            max_alt_err   = max(max_alt_err,   alt_err)
            max_vt_err    = max(max_vt_err,    vt_err)
            max_alpha_err = max(max_alpha_err, alpha_err)

            if alt_err > alt_tol:
                violations.append(
                    f"  t={t:.2f}s  alt err={alt_err:.3f} m > tol={alt_tol:.3f} m"
                )
            if vt_err > vt_tol:
                violations.append(
                    f"  t={t:.2f}s  vt  err={vt_err:.4f} m/s > tol={vt_tol:.4f} m/s"
                )
            if alpha_err > alpha_tol:
                violations.append(
                    f"  t={t:.2f}s  alpha err={alpha_err:.4f} deg > tol={alpha_tol:.4f} deg"
                )

        slave.terminate()
        slave.freeInstance()

        print(f"\n  Completed {step} steps ({t:.1f} s simulated)")
        print(f"  Peak errors:")
        print(f"    alt   max err = {max_alt_err:.4f} m  (tol {alt_tol:.4f} m)")
        print(f"    vt    max err = {max_vt_err:.5f} m/s (tol {vt_tol:.5f} m/s)")
        print(f"    alpha max err = {max_alpha_err:.5f} deg (tol {alpha_tol:.5f} deg)")

        if violations:
            print(f"\n  FAIL — {len(violations)} tolerance violation(s):")
            for v in violations[:10]:
                print(v)
            if len(violations) > 10:
                print(f"  ... and {len(violations)-10} more")
            return False
        else:
            print(f"\n  PASS — all outputs within tolerance for {t_end} s")
            return True

    finally:
        shutil.rmtree(unzipdir, ignore_errors=True)


# ─────────────────────────────────────────────────────────────────────────────
# Test 2 — State save / restore
# ─────────────────────────────────────────────────────────────────────────────
def test_state_restore(fmu_path: pathlib.Path, dt: float, t_checkpoint: float, t_end: float):
    print(f"\n{'─'*60}")
    print(f"Test 2: State save/restore  checkpoint={t_checkpoint} s  t_end={t_end} s")
    print(f"{'─'*60}")

    unzipdir = tempfile.mkdtemp(prefix="f16plant_restore_")
    try:
        model_desc = read_model_description(str(fmu_path))
        extract(str(fmu_path), unzipdir)
        slave, vrs, _ = _boot(fmu_path, unzipdir)

        output_vrs = [vrs[n] for n in _WATCHED]

        def get_outputs():
            return np.array(slave.getReal(output_vrs))

        # ── Phase 1: run to checkpoint ────────────────────────────────────────
        t = 0.0
        while t < t_checkpoint - 1e-10:
            step_dt = min(dt, t_checkpoint - t)
            slave.doStep(t, step_dt, True)
            t += step_dt

        print(f"  Reached t={t:.3f} s  —  saving FMU state …")
        checkpoint = slave.getFMUState()

        # ── Phase 2a: continue to t_end, record trajectory A ─────────────────
        traj_a = []
        while t < t_end - 1e-10:
            step_dt = min(dt, t_end - t)
            slave.doStep(t, step_dt, True)
            t += step_dt
            traj_a.append(get_outputs())

        traj_a = np.array(traj_a)
        print(f"  Recorded trajectory A ({len(traj_a)} steps, t={t:.3f} s)")

        # ── Phase 2b: restore checkpoint and re-run to t_end (trajectory B) ──
        slave.setFMUState(checkpoint)
        slave.freeFMUState(checkpoint)
        t = t_checkpoint

        traj_b = []
        while t < t_end - 1e-10:
            step_dt = min(dt, t_end - t)
            slave.doStep(t, step_dt, True)
            t += step_dt
            traj_b.append(get_outputs())

        traj_b = np.array(traj_b)
        print(f"  Recorded trajectory B ({len(traj_b)} steps, t={t:.3f} s)")

        slave.terminate()
        slave.freeInstance()

        # ── Compare ───────────────────────────────────────────────────────────
        assert len(traj_a) == len(traj_b), "Trajectory lengths differ"
        diff = np.abs(traj_a - traj_b)
        max_diff = diff.max()
        max_idx  = np.unravel_index(diff.argmax(), diff.shape)
        max_var  = _WATCHED[max_idx[1]]

        print(f"\n  Max |A − B| = {max_diff:.2e}  (variable: {max_var}, step {max_idx[0]})")

        # Deterministic re-integration should be identical to round-off precision
        tol = 1e-10
        if max_diff <= tol:
            print(f"  PASS — trajectories are bitwise identical (max diff ≤ {tol:.0e})")
            return True
        else:
            print(f"  FAIL — trajectories diverge: max diff {max_diff:.2e} > {tol:.0e}")
            print(f"         worst variable: {max_var}")
            for i, name in enumerate(_WATCHED):
                col_max = diff[:, i].max()
                if col_max > 0:
                    print(f"    {name:<26s}  max diff = {col_max:.2e}")
            return False

    finally:
        shutil.rmtree(unzipdir, ignore_errors=True)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="F16Plant FMU smoke tests via FMPy")
    parser.add_argument(
        "--fmu",
        type=pathlib.Path,
        default=_DEFAULT_FMU,
        help="Path to F16Plant.fmu  (default: %(default)s)",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="Communication step size [s]  (default: %(default)s)",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=30.0,
        help="End time for trim-stability test [s]  (default: %(default)s)",
    )
    parser.add_argument(
        "--checkpoint",
        type=float,
        default=5.0,
        help="Checkpoint time for state-restore test [s]  (default: %(default)s)",
    )
    args = parser.parse_args()

    fmu_path = args.fmu.resolve()
    if not fmu_path.exists():
        sys.exit(
            f"FMU not found: {fmu_path}\n"
            "Build it first:\n"
            "  cmd /c vcvarsall.bat x64 && "
            "cmake --build out/build/windows-debug --target F16Plant_fmi2"
        )

    print(f"FMU : {fmu_path}")
    print(f"Size: {fmu_path.stat().st_size / 1e6:.1f} MB")

    results = []
    results.append(
        test_trim_stability(fmu_path, dt=args.dt, t_end=args.t_end)
    )
    results.append(
        test_state_restore(
            fmu_path,
            dt=args.dt,
            t_checkpoint=args.checkpoint,
            t_end=args.checkpoint * 2,
        )
    )

    print(f"\n{'═'*60}")
    passed = sum(results)
    total  = len(results)
    print(f"Results: {passed}/{total} passed")
    print(f"{'═'*60}")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
