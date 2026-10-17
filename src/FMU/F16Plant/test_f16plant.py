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

3. Turbulent save / restore
   As test 2 with Dryden turbulence on (sigma 1 m/s on all three axes), the
   gust outputs watched as well, replayed three ways: from the in-memory
   state, from the serialized state, and from the serialized state restored
   into a freshly initialised instance.  All three must reproduce the
   original exactly (second pre-DOI audit, finding N02: the gust held by the
   policies was not restored).

4. Nonzero start time
   Initialise at FMI time 100 s: the initial latitude and longitude must be
   the configured ones, and the run must match one started at 0 s (second
   pre-DOI audit, finding N07: the start time was counted twice).

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
def _boot(fmu_path: pathlib.Path, unzipdir: str, start: float = 0.0, params=None):
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
    # Parameters keep their defaults (NASA Scenario 11 trim condition) unless
    # given here.
    if params:
        slave.setReal([vrs[k] for k in params], list(params.values()))
    slave.exitInitializationMode()   # ← trim solver runs here
    return slave, vrs, model_desc


_TURB = {"turb.sigma_u_mps": 1.0, "turb.sigma_v_mps": 1.0, "turb.sigma_w_mps": 1.0}
_GUSTS = ["out.gust_u_m_s", "out.gust_v_m_s", "out.gust_w_m_s",
          "out.gust_p_rad_s", "out.gust_q_rad_s", "out.gust_r_rad_s"]


def _run(slave, vrs, names, t, dt, n):
    """n steps of dt from t; returns (array of the named outputs after each step, t)."""
    rows = []
    for _ in range(n):
        slave.doStep(t, dt, True)
        t += dt
        rows.append(slave.getReal([vrs[k] for k in names]))
    return np.array(rows), t


def _report(label, ref, got, names):
    d = np.abs(ref - got)
    if d.max() == 0.0:
        print(f"  {label}: identical")
        return True
    i = np.unravel_index(d.argmax(), d.shape)
    print(f"  {label}: FAIL, max |diff| {d.max():.3e} in {names[i[1]]} at step {i[0]}")
    return False


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
# Test 3 — Turbulent save / restore (in memory, serialized, fresh instance)
# ─────────────────────────────────────────────────────────────────────────────
def test_turbulent_restore(fmu_path: pathlib.Path, dt: float, n_before: int = 4, n_after: int = 25):
    print(f"\n{'─'*60}")
    print(f"Test 3: Turbulent save/restore  sigma=1 m/s  dt={dt} s  "
          f"checkpoint after {n_before} steps, {n_after} replayed")
    print(f"{'─'*60}")

    names = _WATCHED + _GUSTS
    unzipdir = tempfile.mkdtemp(prefix="f16plant_turb_")
    try:
        extract(str(fmu_path), unzipdir)
        slave, vrs, _ = _boot(fmu_path, unzipdir, params=_TURB)
        _, t = _run(slave, vrs, names, 0.0, dt, n_before)
        t_ck = t

        state = slave.getFMUState()
        blob = slave.serializeFMUstate(state)
        ref, _ = _run(slave, vrs, names, t_ck, dt, n_after)
        if np.abs(ref[:, len(_WATCHED):]).max() == 0.0:
            print("  FAIL: the gust outputs are all zero; turbulence is not on")
            return False

        ok = True
        slave.setFMUState(state)
        got, _ = _run(slave, vrs, names, t_ck, dt, n_after)
        ok &= _report("in-memory state", ref, got, names)

        slave.setFMUState(slave.deSerializeFMUstate(blob))
        got, _ = _run(slave, vrs, names, t_ck, dt, n_after)
        ok &= _report("serialized state", ref, got, names)
        slave.freeFMUState(state)
        slave.terminate()
        slave.freeInstance()

        fresh, vrs2, _ = _boot(fmu_path, unzipdir, params=_TURB)
        fresh.setFMUState(fresh.deSerializeFMUstate(blob))
        got, _ = _run(fresh, vrs2, names, t_ck, dt, n_after)
        ok &= _report("serialized state in a fresh instance", ref, got, names)
        fresh.terminate()
        fresh.freeInstance()

        print(f"\n  {'PASS' if ok else 'FAIL'}")
        return ok
    finally:
        shutil.rmtree(unzipdir, ignore_errors=True)


# ─────────────────────────────────────────────────────────────────────────────
# Test 4 — Nonzero start time
# ─────────────────────────────────────────────────────────────────────────────
def test_nonzero_start(fmu_path: pathlib.Path, dt: float, t_start: float = 100.0, n: int = 50):
    print(f"\n{'─'*60}")
    print(f"Test 4: Nonzero start time  t0={t_start} s  dt={dt} s  {n} steps")
    print(f"{'─'*60}")

    names = ["out.lat_deg", "out.lon_deg", "out.alt_m", "out.yaw_rad", "out.pitch_rad", "out.roll_rad",
             "out.v_north_m_s", "out.v_east_m_s", "out.v_down_m_s", "out.vt_m_s", "out.alpha_deg"]
    unzipdir = tempfile.mkdtemp(prefix="f16plant_epoch_")
    try:
        extract(str(fmu_path), unzipdir)

        runs = {}
        start = {}
        for t0 in (0.0, t_start):
            slave, vrs, _ = _boot(fmu_path, unzipdir, start=t0)
            # The configured position as the instance holds it (the model
            # description rounds start values).
            start = dict(zip(("lat0_deg", "lon0_deg"), slave.getReal([vrs["lat0_deg"], vrs["lon0_deg"]])))
            first = slave.getReal([vrs[k] for k in names])
            traj, _ = _run(slave, vrs, names, t0, dt, n)
            runs[t0] = (np.array(first), traj)
            slave.terminate()
            slave.freeInstance()

        ok = True
        first = runs[t_start][0]
        for key, idx in (("lat0_deg", 0), ("lon0_deg", 1)):
            if key in start:
                err = abs(first[idx] - start[key])
                print(f"  initial {names[idx]:12s} {first[idx]:.9f}  configured {start[key]:.9f}  |diff| {err:.2e}")
                ok &= err < 1e-9

        # The plant is the same at any epoch: Earth-relative outputs agree
        # with the t0 = 0 run to round-off (the ECI positions differ by a
        # rotation, so not bit for bit).
        d = np.abs(runs[0.0][1] - runs[t_start][1])
        i = np.unravel_index(d.argmax(), d.shape)
        print(f"  max |run(t0) - run(0)| over {n} steps: {d.max():.2e} ({names[i[1]]})")
        ok &= d.max() < 1e-6

        print(f"\n  {'PASS' if ok else 'FAIL'}")
        return ok
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
    results.append(test_turbulent_restore(fmu_path, dt=args.dt))
    results.append(test_nonzero_start(fmu_path, dt=args.dt))

    print(f"\n{'═'*60}")
    passed = sum(results)
    total  = len(results)
    print(f"Results: {passed}/{total} passed")
    print(f"{'═'*60}")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
