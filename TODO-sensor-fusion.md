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
- **Group and error convention: right-invariant EKF on SE₂(3) × ℝ⁷.** The
  navigation core is one SE₂(3) element (R, v, p; 9 error states), and the
  strapdown kinematics are group-affine on it (up to the gravity gradient in
  the LCI frame, below), so the navigation error dynamics are essentially
  independent of the estimate. Right-invariant error
  η = X X̂⁻¹ because every update in the first configuration is world-frame
  (GNSS position, GNSS velocity, barometric altitude). Biases (gyro 3, accel
  3, baro 1) stay Euclidean ("imperfect IEKF"); their coupling terms depend
  on the estimate. Revisit the convention when the magnetometer (a body-frame
  measurement) arrives. SE₂(3) reuses the SO(3) exp/log and Jacobians in
  `ODE/RKMK/Lie/`.
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

- [ ] Fill `EstimationState.h` and `ProcessModel.h`: scalar-templated and
      AD-friendly, in the same style as the dynamics, so one definition serves
      the reference filter and the generator.
- [ ] Fill `MeasurementModels.h`, written against what Hemerion's drivers
      decode, not against idealised quantities:
      - GNSS position from UBX-NAV-PVT latitude, longitude, altitude;
      - GNSS velocity, NED;
      - barometer: compensated pressure to pressure altitude, plus the bias
        state.
- [ ] No antenna lever arm in the first configuration; state the assumption
      in the header.

### Step 6 — Reference filter and replay harness

- [ ] Fill `EKF.h`: double precision, Eigen, dynamic sizes acceptable.
- [ ] Replay harness that reads the Hemerion logs and the truth log.
- [ ] Time base: `host_time_s` is the flight computer's clock and equals
      simulation time only under `--rtf 1`. Map it onto simulation time with a
      fit over the IMU log, which carries both clocks (Hemerion's TODO
      describes this calibration). Requirement from the LCI frame: residual
      error well below 1 ms, since a timing error δt rotates the
      ECEF↔LCI mapping by Ω·δt (~0.5 m at Earth radius per ms).
- [ ] Delayed-measurement handling for GNSS, exercised with
      `--gps-latency`.
- [ ] NEES and NIS against truth. Bias-state NEES needs the realized bias
      values, which the sensor FMUs do not publish yet (Hemerion step 3).
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
      Eigen interop problems.

## Deferred

- Magnetometer block (hard iron, soft iron) and measurement: after Hemerion
  step 9.
- Wind block, GNSS antenna lever arm.
- Second vehicle configuration (rocket, then the tail-sitter): after step 9.
  That is the earliest point at which the block scheme is shown to be general.
- Optical flow from ZMX-1: a new measurement model through the same loop.
- Fault scenarios fed back from Hemerion step 11 as injected cases.
