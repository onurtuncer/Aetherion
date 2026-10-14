# Sensor fusion — Aetherion half

Drafted 2026-10-08 against `main` at `9a80aac` (v0.16.0) and Hemerion `main` at
`55a95ce`. The Hemerion half is `TODO-sensor-fusion.md` in that repo; the step
numbers below are shared between the two files so the cross-repo order is
readable from either.

## Where things stand

- `Aetherion/Estimation/{EKF,EstimationState,MeasurementModels,ProcessModel}.h`
  are four 15-line placeholders. There is no filter in the tree.
- `vcpkg.json` lists `cppad` only. CppADCodeGen is not a dependency and nothing
  generates C.
- Truth and environment are done: `F16Plant`, `F16Autopilot`, `TwoStageRocket`
  FMUs with steady wind, Dryden turbulence and atmosphere offsets (v0.16.0).
- Sensor models are **not** here and are not going to be. Hemerion owns all
  five (GPS, IMU, BMP390, MMC5983MA, radar altimeter) as FMUs, together with
  the drivers and arrival stamping.

## Decided (2026-10-08)

- **Ownership.** Aetherion owns truth and the estimator mathematics: state
  blocks, process model, measurement models, the double-precision reference
  filter, the code generator and the consistency analysis. Hemerion owns
  everything a sensor or a target touches.
- **Dependency direction.** Aetherion never depends on Hemerion. It replays
  the logs Hemerion's co-simulation hosts write. (Aetherion is MIT, Hemerion is
  GPL-3.0: generated code may flow into Hemerion, sensor code may not flow
  back.)
- **Filter form.** Lie group EKF. The 15-state quaternion table in Hemerion's
  `modules/gnc/README.md` is superseded.
- **The generator lives here.** Hemerion receives generated plain C plus a
  configuration hash; its planned `tools/gen_ekf_jacobians.cpp` is dropped.
- **State is composed from blocks, not frozen as one vector.** Named blocks
  with fixed dimension, units, frame and process model; a vehicle
  configuration selects blocks at build time; canonical ordering with the
  navigation core first. Availability of a measurement at run time is handled
  by skipping updates, never by resizing the state.
- **First configuration: 16 error states.** Navigation core (attitude,
  velocity, position: 9), gyro bias (3), accelerometer bias (3), barometer
  bias (1). Updates: GNSS position, GNSS velocity, barometer.
- **Barometer bias.** One state, in metres of pressure altitude, random walk
  (not a constant: a temperature offset gives an error that grows with
  height, so it drifts in a climb).
- **No magnetometer block or measurement** until Hemerion's replay
  equivalence (step 9) has passed once.
- **Group: SE₂(3) × ℝ⁷.** The navigation core is one SE₂(3) element
  (R, v, p; 9 error states); the strapdown kinematics are group-affine on
  it (up to the gravity gradient in the LCI frame, below). Biases (gyro 3,
  accel 3, baro 1) stay Euclidean ("imperfect IEKF"). SE₂(3) reuses the
  SO(3) exp/log and Jacobians in `ODE/RKMK/Lie/`.
- **Navigation frame: launch-centred inertial (LCI).** Origin at the launch /
  take-off point, axes equal to local NED at t₀, then frozen in inertial
  space. Related to ECI by a constant rotation and translation.
  - Inertial, so no Earth-rate or Coriolis terms; the only group-affine
    violation is the gravity gradient (Schuler scale, ~1.5e-6 s⁻²),
    negligible. ECEF was rejected: the velocity row fails the group-affine
    test, residual (W R₁ − R₁ W)(v₂ + W p₂) with W = −[Ω]×. Local NED was
    rejected: it ignores Earth rate (~15°/h, not absorbable by the gyro bias
    since it rotates with attitude), Coriolis (~3.6 mg at 250 m/s) and
    gravity-direction tilt (d/R), all of which the ECI truth and an
    inertial-rate IMU contain; and it cannot carry the rocket.
  - Matches the ECI truth and the inertial IMU directly; serves the rocket
    configuration unchanged.
  - Float32-safe: positions stay ~1e5 m (ECEF/ECI magnitudes ~6.4e6 m
    resolve only ~0.5 m in float32).
  - Costs: gravity evaluated at the estimated position (WGS84 model), not
    constant; GNSS and barometer models need the time-dependent
    C_ECEF←LCI(t); heading/yaw covariance reported via rotation into the
    current local NED.

## Still open

- [ ] **Error convention: left- or right-invariant.** Not decided, and no
      longer blocking: step 5 takes it as a template parameter. Evidence so
      far: `scripts/iekf_convention_mc.py` (NumPy Monte Carlo, paired LI/RI
      runs; results in `doc/sensor_fusion.rst`, 2026-10-08): equivalent in
      normal operation; RI clearly better under large corrections (45° and
      120° initial heading error, and at the onset of heading
      observability, where LI becomes overconfident); RI fails in float32
      with a Joseph-form covariance (every run), LI unaffected. Next:
      covariance reset after updates in both; RI with UD / square-root in
      float32; initial-heading sweep (5–20°) for LI; rocket trajectory.
      New evidence 2026-10-13 from the C++ filter on synthetic logs
      (`tests/Estimation/test_Replay.cpp`, float64, Joseph form, 2–5 km
      from the LCI origin): with initial sigmas of 5° yaw and 30 m
      position, RI holds ~0.9° of tilt RMS and ANEES ~2 over 120 s while LI
      settles to 0.11° and ANEES 1.3 on the same samples; with 1° / 10 m
      sigmas RI is at 0.3°. Same direction as finding 3 (the [p̂]× coupling
      of large covariances), already in float64. Worth a dedicated sweep
      over initial sigma × distance from the origin before deciding.
      When decided, update the
      `error=` line of the configuration descriptor and the pinned hash
      (currently provisional `right-invariant`). Trade-off is written up in
      `doc/sensor_fusion.rst`: LI gives constant GNSS position/velocity
      Jacobians and constant bias coupling (−I), and avoids the [p̂]× terms
      (‖p̂‖ up to ~1e5 m in LCI); RI gives constant navigation error dynamics
      and a constant magnetometer Jacobian, but bias coupling −Ad_X̂.
- [ ] **Rates.** Hemerion's README intends 500 Hz predict / 18 Hz GPS; its
      examples run a 100 Hz IMU and 10 Hz GPS. Hemerion's decision (its
      step 4), but the process-noise tuning here depends on it.

## To do, in order

### Step 2 — Block scheme and log contract (shared; this repo is the source of truth)

- [x] Block library: `Aetherion/Estimation/Blocks.h` (`kBlockLibrary`:
      name, dimension, units, frame, process model, canonical rank; nav core
      in right-invariant tangent order phi, nu, rho).
- [x] Vehicle configuration and hash: `Aetherion/Estimation/Configuration.h`.
      Hash is FNV-1a 64 over a canonical text descriptor that also names
      group, error convention and frame. First configuration
      `kNavBaroGnss16`, hash `0x1e59a990edd19e83`, pinned in
      `tests/Estimation/test_Configuration.cpp`.
- [x] Measurement → block dependencies (`kMeasurementLibrary`);
      `Configuration::valid()` rejects a measurement without its blocks.
- [x] Log contract: `Aetherion/Estimation/LogContract.h`. Columns bound by
      name, all missing ones reported at once; required columns depend on
      the configuration. Tested against the header lines Hemerion `c4a3178`
      writes.
- [ ] **Hemerion: log GNSS velocity.** `GpsFix` decodes only `gSpeed` and
      course, although `UbxEmitter` already encodes velN/E/D and sAcc. Until
      `gps_fixes.csv` carries `vel_north_mps`, `vel_east_mps`,
      `vel_down_mps`, `speed_accuracy_mps` (names proposed here),
      `kNavBaroGnss16` does not bind; the contract test asserts that failure.
      Ground speed and course are not a substitute (singular at low speed,
      no vertical component).
- [ ] **Hemerion: GPS altitude datum.** `GpsFix::altitude_m` is documented as
      MSL, but `ubxParser` reads NAV-PVT `height` (offset 32, ellipsoidal),
      and the FMU is driven with Aetherion's geodetic altitude. The
      measurement model assumes ellipsoidal; fix the comment on that side.
- [ ] **Hemerion: rocket truth log has no attitude.** `rocket_gps_ecos` logs
      no Euler angles at all (besides the plant only publishing Euler
      angles). Needed before the rocket configuration.

### Step 5 — Process and measurement models

- [x] `EstimationState.h` and `ProcessModel.h` (2026-10-13), plus `SE23.h`
      (group: compose, inverse, Exp, Log, Adjoint; SO(3) `Log_R` and
      `LeftJacobianInverse` added to `ODE/RKMK/Lie/SO3.h`) and `LciFrame.h`
      (origin from geodetic + ERA, ECI↔LCI, WGS-84 J2 gravity, central-term
      gradient). Scalar-templated; `tests/Estimation/test_ProcessModel.cpp`
      tapes the full step with `CppAD::AD<double>` through Eigen
      (`cppad/example/cppad_eigen.hpp`) and matches finite differences to
      1e-9 — the first Eigen+CppAD use in the tree, so the step-8 risk is
      retired early.
- [x] **Error convention as a template parameter**: `ErrorConvention` in
      `Blocks.h`; `retract<C>`/`error<C>`, `errorJacobians<C>`/`step<C>`.
      `Configuration::descriptor()`/`hash()` take the convention; the
      pinned hash is the right-invariant default, LI hashes differently.
- [x] Trapezoidal F. Verified as a convergence order, not a tolerance:
      ‖Φ − Φ_fd‖ falls 7.9× (RI) / 7.4× (LI) per halving of dt; with a
      start-of-step F the RI ratio is 4.00 (first order), 50–150× worse at
      the same dt. The check also found that the nonlinear step must rotate
      the velocity increment by the **mid-step** attitude R Exp(ω dt/2): with
      the start-of-step attitude (as in the prototype) its linearisation has
      no ∂v/∂b_g term, the residual ratio is 2, and Φ is inconsistent with
      the state propagation at first order.
- [x] `MeasurementModels.h` (2026-10-13), against what Hemerion's drivers
      decode: GNSS position from NAV-PVT lat/lon/ellipsoidal height, GNSS
      velocity NED + transport term ω⊕×r, both through the ERA at the fix's
      time (`LciFrame::earthRotationAngle`); barometer = `pressureAltitude_m`
      (exact inverse of `US1976Atmosphere`) against
      `GeometricAltitude_m(p)` + bias. Innovation, H and R per convention;
      H checked by finite differences and (barometer) CppAD. Note for the
      replay harness: `ECEFToGeodeticWGS84` is declared as a template but
      defined in a `.cpp` for `double` only — fine for logs, not for h(x).
- [x] No antenna lever arm; stated in the header of `MeasurementModels.h`
      together with the time-base assumption (fixes are on simulation time
      when they reach the models).

### Step 6 — Reference filter and replay harness

- [x] `EKF.h` (2026-10-13): `InvariantEKF<S, C>`, Joseph form, dynamic
      Eigen, skipped updates for measurements the configuration lacks,
      NEES/NIS, `covarianceFromPhysical()`. Validated in both conventions on
      an analytic manoeuvring truth (`tests/Estimation/test_EKF.cpp`): NEES
      0.9/0.8, NIS at 1, position at the GNSS floor. Two lessons: (1) an
      unbanked constant-rate turn makes every tilt error indistinguishable
      from a body accelerometer bias — the filter drifted along that
      direction with NEES growing while GNSS held position; any synthetic
      truth for step 7 must bank and pitch; (2) sample the IMU at mid-step
      when generating from an analytic truth, or the ZOH mismatch looks like
      a bias. Yaw from a 5° start is still ~0.5–1° after 240 s: weakly
      observable, as expected without a magnetometer.
- [x] Replay harness (2026-10-13): `Aetherion/Estimation/Replay/{Csv,Logs,
      TimeBase,TruthState,Replay}.h` and the `EkfReplay` executable
      (`--logs <dir> --truth <csv> [--convention ri|li] [--config
      nav-baro-gnss|nav-baro-gnss16]`). Validated on synthetic logs in
      Hemerion's formats (`tests/Estimation/test_Replay.cpp`): ANEES
      1.06/0.83, NIS at 1, position 0.33 m. **Not yet run on real Hemerion
      logs** — none exist on this machine; generating a set needs the
      Hemerion co-sim example built against the Aetherion FMUs. First real
      replay must use `--config nav-baro-gnss` until GNSS velocity is logged.
- [x] Time base. Surveyed Hemerion's writers at `origin/main` 7da6057
      (2026-10-13; the local clone was 193 commits behind): IMU
      `part_time_s` is simulation time unless `--sensor-clock` skew/jitter
      is on; the GPS log's `nominal_time_s` = `fix_count * fix_period` is
      the fix's epoch in simulation time with the latency cancelled; the
      BMP390 `part_time_s` wraps every 512 s; `host_time_s` is burst
      arrival on the flight computer's clock and, unpaced, bears no fixed
      relation to simulation time. So the harness times IMU and baro on
      their part clocks (baro unwrapped), fixes on `nominal_time_s`, and
      keeps the least-squares `host = offset + scale * part` fit
      (`TimeBase.h`) as a cross-check and fallback with its residual
      reported. `fix_type` is Hemerion's enum (0/1/2 = none/2D/3D), not
      UBX's; epochs without a fix are rows of empty fields. All of this is
      in `LogContract.h` now.
- [x] Delayed measurements: the replay reference applies a fix at its
      validity time, splitting the IMU interval exactly (`Replay.h`; the
      "apply at the next IMU step" shortcut cost 0.6 m at 120 m/s).
      Recovering the latency from arrival stamps is the flight filter's
      job (Hemerion step 10) and is judged against this reference by the
      golden vectors.
- [ ] **Replay a real Hemerion log set.** Needs Hemerion built on Windows
      (`cmake --preset examples-native`, `AETHERION_ROOT` or `--f16` at
      `out/build/windows-release/models/fmi2/F16Plant/F16Plant.fmu`), then
      `f16_flight_computer` first and `f16_trim_cosim --case 11 --rtf 1
      [--gps-latency 0.2]` in `build/examples-native/examples/f16_trim_ecos/`;
      logs land in each process's `results/`. Use `--rtf 1`: unpaced, the
      polled barometer yields ~37 conversions per 200 s. Then
      `EkfReplay --logs results --truth results/f16_truth.csv --config
      nav-baro-gnss`. Expect the time-base residual to show the 0.01 s
      burst sawtooth.
- [x] NEES and NIS against truth, in the filter's own coordinates. NEES is
      over the navigation core only until the realized biases are published
      (Hemerion step 3); the summary says so.
- [ ] UKF as a cross-check, after the EKF is consistent.

### Step 7 — Tuning and observability

Run on the cases Hemerion's TODO already names, with turbulence on (calm air
leaves every body rate below one gyro count on case 11).

- [ ] Case 11: baseline for position, velocity, tilt and bias consistency.
- [ ] Case 12: degraded mode (no GPS fix, barometer below its rated floor).
- [ ] Case 13.1: barometer bias drift in the climb.
- [ ] Cases 13.3 and 13.4: heading observability.
- [ ] Expected, not a bug: yaw is weakly observable in trimmed flight without
      a magnetometer, so yaw covariance grows on case 11. Initialise heading
      from truth. Do **not** aid or initialise yaw from GNSS course: Hemerion
      measured 2.28 degrees between course and yaw in a 10 m/s crosswind.
- [ ] Set Q and R; record where states become weakly observable.

### Step 7b — Numerical form

- [ ] Rerun the reference filter in float32.
- [ ] Choose between Joseph-form and UD or square-root covariance, here, where
      truth is available. The choice is an input to Hemerion's skeleton.

### Step 8 — Code generation and golden vectors

- [ ] Add CppADCodeGen as a dependency (host-side generator only).
- [ ] Generator tool: emits plain C for f, h and their Jacobians for one
      configuration, plus a generated header with the dimension, block offsets
      and the configuration hash.
- [ ] Golden vectors: recorded inputs and outputs of the reference filter with
      tolerances, carrying the same hash.
- [ ] Hand both to Hemerion; they are checked in there.

## Things to check

- [x] **Truth attitude on the plant FMUs.** `F16Plant` publishes Euler angles
      (`out.yaw_rad`, `out.pitch_rad`, `out.roll_rad`), which is adequate for
      the F-16 cases. Checked 2026-10-08: `TwoStageRocket` publishes the same
      ZYX Euler angles only, and a vertical launch starts at the 90° pitch
      singularity. **To do** before the rocket becomes the second
      configuration: add a quaternion or rotation-matrix truth output.
- [x] `README.md` listed the EKF (and the auto-Jacobian EKF/UKF pipeline) as
      existing features; both now marked *(Planned)*.
- [x] `vcpkg.json` version bumped from 0.12.0 to 0.16.0.
- [ ] CppAD code-generation compatibility: build the dynamics templates with
      `CppAD::cg::CG<double>` early (before step 8) to surface `CondExp` or
      Eigen interop problems. Half done 2026-10-13: the estimator templates
      tape with `CppAD::AD<double>` through Eigen (test_ProcessModel.cpp);
      `CG<double>` itself still untried (CppADCodeGen is not a dependency
      yet). One AD trap found and documented in `SO3::Log_R`: `acos` has an
      infinite derivative at the identity, so the small-angle switch and
      series must be written in sin²θ = |½ vee(R − Rᵀ)|², never in θ — and
      CondExp discards the dead branch's *value*, not its evaluation: CppAD
      still differentiated `acos` at c = 1, which UBSAN reported as a
      division by zero (CI, 2026-10-13). `c` is now clamped to ±(1 − 1e-12).
      Lesson for the generator: every CondExp branch must be finite and
      differentiable everywhere it is evaluated, not only where selected.

## Deferred

- Magnetometer block (hard iron, soft iron) and measurement: after Hemerion
  step 9.
- Wind block, GNSS antenna lever arm.
- Second vehicle configuration (rocket, then the tail-sitter): after step 9.
  That is the earliest point at which the block scheme is shown to be general.
- Optical flow from ZMX-1: a new measurement model through the same loop.
- Fault scenarios fed back from Hemerion step 11 as injected cases.
