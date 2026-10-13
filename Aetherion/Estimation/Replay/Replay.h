// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Replay.h
//
// The replay harness (sensor fusion step 6): runs the reference filter
// (EKF.h) over a LogSet of Hemerion's co-simulation logs and judges it
// against the truth log with NEES and NIS.
//
// Time. Every record is put on the simulation time base before anything
// else happens. What Hemerion's writers stamp (surveyed at its origin/main
// 7da6057, 2026-10-13):
//   IMU   part_time_s  the sensor's own clock; equal to simulation time
//                      unless the FMU's clock skew/jitter options are on.
//         host_time_s  arrival of the burst holding the sample, on the
//                      flight computer's clock -- bursty (one stamp per
//                      communication step) and, unpaced (rtf 0), with no
//                      fixed relation to simulation time at all.
//   baro  part_time_s  the BMP390 SENSORTIME register: simulation time,
//                      wrapping every 512 s. Unwrapped against the
//                      fit-mapped host time.
//   GNSS  nominal_time_s  fix_count * fix_period: the fix's epoch in
//                      simulation time, index-based, so the latency
//                      cancels in it. The validity time.
//         host_time_s  arrival on the flight computer's clock, latency
//                      included (NAV-PVT carries no time of its own). Used
//                      only if nominal_time_s is absent: mapped through
//                      the fit over the IMU log, minus the run's
//                      gps_latency_s.
// The fit host = offset + scale * part over the IMU log (TimeBase.h) is
// therefore a cross-check and a fallback, not the primary clock; its
// residual is reported. This is the replay reference, which may know the
// latency; the flight filter has to recover it from the arrival stamps
// (Hemerion step 10) and is judged against this reference by the golden
// vectors (step 9). The choices are in ReplayTiming so a different log
// convention is one struct away, not a rewrite.
//
// Order. Events are merged by time. Each IMU interval [t_k-1, t_k] is
// propagated with the readings held at the average of its two bounding
// samples (the trapezoidal rule, second order for a smooth signal, where
// holding the earlier sample would leave a ZOH lag of half a period that
// reads as a bias). A measurement whose validity time falls inside the
// interval splits it: predict to that time, apply, predict the remainder.
// Not "apply after the IMU step that reaches it": a fix stamped a few
// microseconds after an IMU boundary would then be applied a whole period
// late, and at 120 m/s a mean lag of half a 10 ms period is a 0.6 m
// along-track bias that rotates with the heading (seen on synthetic logs;
// ANEES 3.8 with every NIS at 1).
//
// Initialisation. From the truth at the first IMU sample: position,
// velocity and attitude (heading from truth, as the plan says; never from
// GNSS course), optionally perturbed; biases at zero. The LCI frame's
// origin is the sidecar's lat0/lon0/alt0, era0 = 0.
//
// Output. One row per IMU sample with the estimate, the truth, the error in
// the filter's own coordinates, its NEES, and the 1-sigma of each state;
// one row per update with its NIS; and a summary. Bias NEES needs the
// realised biases, which the FMUs do not publish yet (Hemerion step 3), so
// the NEES here is over the navigation core only, with that stated in the
// summary.
// ------------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numbers>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <Aetherion/Estimation/EKF.h>
#include <Aetherion/Estimation/Replay/Logs.h>
#include <Aetherion/Estimation/Replay/TimeBase.h>
#include <Aetherion/Estimation/Replay/TruthState.h>

namespace Aetherion::Estimation::Replay
{

/// Which clock each record lives on, as functions of the record, the
/// time-base fit and the run configuration. The defaults are the Hemerion
/// convention as understood on 2026-10-13 (file header).
struct ReplayTiming
{
  std::function<double(const ImuRecord&, const TimeBaseFit&, const RunConfiguration&)> imu =
      [](const ImuRecord& r, const TimeBaseFit&, const RunConfiguration&) { return r.partTime; };

  /// The part clock, unwrapped: the multiple of the wrap period that puts
  /// it nearest the fit-mapped host time. Exact while the host mapping is
  /// good to a quarter period (128 s), which even an unpaced run is.
  std::function<double(const BaroRecord&, const TimeBaseFit&, const RunConfiguration&)> baro =
      [](const BaroRecord& r, const TimeBaseFit& f, const RunConfiguration&) {
        constexpr double W = LogContract::Baro::kPartTimeWrap_s;
        const double ref = f.toPart(r.hostTime);
        return r.partTime + W * std::round((ref - r.partTime) / W);
      };

  /// nominal_time_s when the log has it, else the mapped arrival time less the latency.
  std::function<double(const GpsRecord&, const TimeBaseFit&, const RunConfiguration&)> gps =
      [](const GpsRecord& r, const TimeBaseFit& f, const RunConfiguration& run) {
        if (std::isfinite(r.nominalTime))
          return r.nominalTime;
        return f.toPart(r.hostTime) - run.gpsLatency_s;
      };
};

struct ReplayOptions
{
  ProcessNoise<double> noise{ 2e-4, 5e-3, 2e-6, 2e-5, 0.05 };  ///< Hemerion's IMU densities at 100 Hz (its step 4)
  double baroSigma_m = 1.0;                                    ///< pressure-altitude noise given to the filter
  double gnssVelSigma_m_s = 0.1;                               ///< used when the log carries no speed accuracy
  double gnssHorizScale = 1.0;  ///< sigma_N = sigma_E = gnssHorizScale * horizontal_accuracy_m
  int minFixType = LogContract::Gps::kFixType3D;  ///< Hemerion's enum (0 none, 1 2D, 2 3D); fixes below it are skipped
  /// Initial 1-sigma in physical coordinates: attitude (3, rad), velocity (3), position (3), then the bias blocks.
  Eigen::VectorXd initialSigma;
  /// Perturbation added to the truth at initialisation, in the same physical coordinates (default none).
  Eigen::VectorXd initialError;
  ReplayTiming timing;
  double outputEvery_s = 0.0;  ///< 0: every IMU sample
};

struct UpdateRow
{
  double t = 0.0;
  MeasurementId id{};
  int dim = 0;
  double nis = 0.0;
};

struct StateRow
{
  double t = 0.0;
  EstimationState<double> estimate;
  EstimationState<double> truth;
  Eigen::VectorXd error;         ///< filter coordinates, navigation core + biases (biases against zero truth)
  Eigen::VectorXd sigma;         ///< sqrt(diag P)
  double neesNav = 0.0;          ///< navigation core only (9 dof)
  Eigen::Vector3d eulerErr_rad;  ///< yaw, pitch, roll error in local NED
};

struct ReplaySummary
{
  ErrorConvention convention{};
  TimeBaseFit timeBase;
  std::size_t imuSamples = 0, gnssPositionUpdates = 0, gnssVelocityUpdates = 0, baroUpdates = 0, skippedFixes = 0;
  double duration_s = 0.0;
  double aneesNav = 0.0;  ///< mean NEES of the navigation core over the second half, / 9
  double posRms_m = 0.0, velRms_m_s = 0.0, tiltRms_deg = 0.0, yawRms_deg = 0.0;  ///< second half
  double nisGnssPosition = 0.0, nisGnssVelocity = 0.0, nisBaro = 0.0;            ///< mean NIS / dim, second half
};

struct ReplayResult
{
  ReplaySummary summary;
  std::vector<StateRow> states;
  std::vector<UpdateRow> updates;
};

namespace detail
{
[[nodiscard]] inline GeodeticTruth interpolateTruth(const std::vector<TruthRecord>& truth, double t)
{
  constexpr double kDeg = std::numbers::pi / 180.0;
  auto it =
      std::lower_bound(truth.begin(), truth.end(), t, [](const TruthRecord& r, double tt) { return r.time < tt; });
  const TruthRecord* b = (it == truth.end()) ? &truth.back() : &*it;
  const TruthRecord* a = (it == truth.begin()) ? b : &*(it - 1);
  const double w = (b->time > a->time) ? std::clamp((t - a->time) / (b->time - a->time), 0.0, 1.0) : 0.0;
  auto lerp = [w](double x, double y) { return x + w * (y - x); };
  auto lerpAngle = [w](double x, double y) {
    const double d = std::remainder(y - x, 2.0 * std::numbers::pi);
    return x + w * d;
  };
  GeodeticTruth g;
  g.t = t;
  g.lat_rad = lerp(a->lat_deg, b->lat_deg) * kDeg;
  g.lon_rad = lerpAngle(a->lon_deg * kDeg, b->lon_deg * kDeg);
  g.alt_m = lerp(a->alt_m, b->alt_m);
  g.vNed = a->vNed_m_s + w * (b->vNed_m_s - a->vNed_m_s);
  g.yaw_rad = lerpAngle(a->yaw_rad, b->yaw_rad);
  g.pitch_rad = lerp(a->pitch_rad, b->pitch_rad);
  g.roll_rad = lerp(a->roll_rad, b->roll_rad);
  return g;
}
}  // namespace detail

template <ErrorConvention C>
[[nodiscard]] ReplayResult replay(const LogSet& logs, const Configuration& cfg, const ReplayOptions& opt)
{
  constexpr double kDeg = std::numbers::pi / 180.0;
  if (logs.imu.empty())
    throw std::runtime_error("replay: no IMU samples");
  if (logs.truth.empty())
    throw std::runtime_error("replay: no truth");

  ReplayResult res;
  res.summary.convention = C;

  // Time base from the IMU log.
  {
    std::vector<double> part, host;
    part.reserve(logs.imu.size());
    host.reserve(logs.imu.size());
    for (const ImuRecord& r : logs.imu)
    {
      part.push_back(r.partTime);
      host.push_back(r.hostTime);
    }
    res.summary.timeBase = fitTimeBase(part, host);
  }
  const TimeBaseFit& fit = res.summary.timeBase;

  // Frame and filter.
  const LciFrame<double> frame =
      LciFrame<double>::FromGeodetic(logs.run.lat0_deg * kDeg, logs.run.lon0_deg * kDeg, logs.run.alt0_m, 0.0);
  const MeasurementModels<double> meas(frame);
  InvariantEKF<double, C> ekf(cfg, frame, opt.noise);
  const int n = cfg.dimension();

  // Initial state from the truth at the first IMU sample.
  const double t0 = opt.timing.imu(logs.imu.front(), fit, logs.run);
  EstimationState<double> x0 = truthToState(detail::interpolateTruth(logs.truth, t0), frame);
  Eigen::VectorXd sig = opt.initialSigma;
  if (sig.size() != n)
  {
    sig = Eigen::VectorXd::Zero(n);
    sig.segment<3>(cfg.offset(BlockId::NavCore) + NavCoreLayout::kAttitude).setConstant(1.0 * kDeg);
    sig.segment<3>(cfg.offset(BlockId::NavCore) + NavCoreLayout::kVelocity).setConstant(1.0);
    sig.segment<3>(cfg.offset(BlockId::NavCore) + NavCoreLayout::kPosition).setConstant(10.0);
    if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
      sig.segment<3>(o).setConstant(2e-3);
    if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
      sig.segment<3>(o).setConstant(0.05);
    if (const int o = cfg.offset(BlockId::BaroBias); o >= 0)
      sig[o] = 20.0;
  }
  if (opt.initialError.size() == n)
  {
    // Physical perturbation: world-frame attitude error, dv, dp, biases.
    const int nav = cfg.offset(BlockId::NavCore);
    const Eigen::Vector3d dth = opt.initialError.segment<3>(nav + NavCoreLayout::kAttitude);
    x0.X.R = ODE::RKMK::Lie::SO3::Exp_R(dth) * x0.X.R;
    x0.X.v += opt.initialError.segment<3>(nav + NavCoreLayout::kVelocity);
    x0.X.p += opt.initialError.segment<3>(nav + NavCoreLayout::kPosition);
    if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
      x0.gyroBias += opt.initialError.segment<3>(o);
    if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
      x0.accelBias += opt.initialError.segment<3>(o);
    if (const int o = cfg.offset(BlockId::BaroBias); o >= 0)
      x0.baroBias += opt.initialError[o];
  }
  ekf.initialise(x0, ekf.covarianceFromPhysical(x0, Eigen::MatrixXd(sig.cwiseProduct(sig).asDiagonal())));

  // Measurement events on the simulation time base, in order.
  struct Event
  {
    double t;
    int kind;  // 0 gps, 1 baro
    std::size_t index;
  };
  std::vector<Event> events;
  for (std::size_t i = 0; i < logs.gps.size(); ++i)
    events.push_back({ opt.timing.gps(logs.gps[i], fit, logs.run), 0, i });
  for (std::size_t i = 0; i < logs.baro.size(); ++i)
    events.push_back({ opt.timing.baro(logs.baro[i], fit, logs.run), 1, i });
  std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.t < b.t; });
  std::size_t nextEvent = 0;
  while (nextEvent < events.size() && events[nextEvent].t < t0)
    ++nextEvent;  // measurements before the first IMU sample cannot be applied

  const bool useGnssPos = cfg.has(MeasurementId::GnssPosition);
  const bool useGnssVel = cfg.has(MeasurementId::GnssVelocity);
  const bool useBaro = cfg.has(MeasurementId::Barometer);
  const int nav = cfg.offset(BlockId::NavCore);

  auto applyEvent = [&](const Event& e) {
    if (e.kind == 0)
    {
      const GpsRecord& g = logs.gps[e.index];
      if (!g.valid || g.fixType < opt.minFixType)
      {
        ++res.summary.skippedFixes;
        return;
      }
      const double era = frame.earthRotationAngle(e.t);
      const GnssFix<double> fix{ g.lat_deg * kDeg, g.lon_deg * kDeg, g.alt_m };
      const Eigen::Matrix3d C_LN = meas.nedToLci(fix, era);
      if (useGnssPos)
      {
        const double sh = opt.gnssHorizScale * g.horizontalAccuracy_m;
        const Eigen::Matrix3d SigmaN =
            Eigen::Vector3d(sh * sh, sh * sh, g.verticalAccuracy_m * g.verticalAccuracy_m).asDiagonal();
        const auto r = ekf.updateGnssPosition(meas.gnssPositionLci(fix, era), C_LN * SigmaN * C_LN.transpose());
        if (r.applied)
        {
          res.updates.push_back({ e.t, MeasurementId::GnssPosition, r.dim, r.nis });
          ++res.summary.gnssPositionUpdates;
        }
      }
      if (useGnssVel && g.hasVelocity)
      {
        const double sv = (g.speedAccuracy_m_s > 0.0) ? g.speedAccuracy_m_s : opt.gnssVelSigma_m_s;
        const Eigen::Matrix3d SigmaV = Eigen::Matrix3d::Identity() * sv * sv;
        const auto r =
            ekf.updateGnssVelocity(meas.gnssVelocityLci(fix, g.velNed_m_s, era), C_LN * SigmaV * C_LN.transpose());
        if (r.applied)
        {
          res.updates.push_back({ e.t, MeasurementId::GnssVelocity, r.dim, r.nis });
          ++res.summary.gnssVelocityUpdates;
        }
      }
    }
    else if (useBaro)
    {
      const BaroRecord& b = logs.baro[e.index];
      const auto r = ekf.updateBarometer(pressureAltitude_m(b.pressure_Pa), opt.baroSigma_m);
      if (r.applied)
      {
        res.updates.push_back({ e.t, MeasurementId::Barometer, r.dim, r.nis });
        ++res.summary.baroUpdates;
      }
    }
  };

  auto record = [&](double t) {
    StateRow row;
    row.t = t;
    row.estimate = ekf.state();
    row.truth = truthToState(detail::interpolateTruth(logs.truth, t), frame);
    row.error = ekf.state().template error<C>(cfg, row.truth);
    row.sigma = ekf.covariance().diagonal().cwiseSqrt();
    const Eigen::VectorXd eNav = row.error.segment<9>(nav);
    const Eigen::MatrixXd Pnav = ekf.covariance().template block<9, 9>(nav, nav);
    row.neesNav = eNav.dot(Pnav.ldlt().solve(eNav));
    const Eigen::Vector3d eulerEst = eulerNedFromState(ekf.state(), t, frame);
    const Eigen::Vector3d eulerTruth = eulerNedFromState(row.truth, t, frame);
    for (int i = 0; i < 3; ++i)
      row.eulerErr_rad[i] = std::remainder(eulerEst[i] - eulerTruth[i], 2.0 * std::numbers::pi);
    res.states.push_back(std::move(row));
  };

  // Main loop.
  double tPrev = t0;
  double tLastOut = -1e300;
  record(t0);
  tLastOut = t0;
  for (std::size_t k = 1; k < logs.imu.size(); ++k)
  {
    const ImuRecord& s = logs.imu[k];
    const double t = opt.timing.imu(s, fit, logs.run);
    if (t <= tPrev)
      continue;  // duplicate or out-of-order stamp: skip the sample
    // Readings held over [tPrev, t]: the average of the bounding samples.
    const ImuRecord& u0 = logs.imu[k - 1];
    const ImuSample<double> u{ 0.5 * (u0.gyro_rad_s + s.gyro_rad_s), 0.5 * (u0.accel_m_s2 + s.accel_m_s2) };

    // Split the interval at every measurement inside it.
    double tCur = tPrev;
    while (nextEvent < events.size() && events[nextEvent].t <= t)
    {
      const Event& e = events[nextEvent++];
      if (e.t > tCur)
      {
        ekf.predict(u, e.t - tCur);
        tCur = e.t;
      }
      applyEvent(e);
    }
    if (t > tCur)
      ekf.predict(u, t - tCur);
    tPrev = t;
    ++res.summary.imuSamples;

    if (opt.outputEvery_s <= 0.0 || t - tLastOut >= opt.outputEvery_s - 1e-9)
    {
      record(t);
      tLastOut = t;
    }
  }

  // Summary over the second half.
  ReplaySummary& S = res.summary;
  S.duration_s = tPrev - t0;
  const double tHalf = t0 + 0.5 * S.duration_s;
  double nees = 0.0, pos = 0.0, vel = 0.0, tilt = 0.0, yaw = 0.0;
  std::size_t cnt = 0;
  for (const StateRow& r : res.states)
  {
    if (r.t < tHalf)
      continue;
    nees += r.neesNav;
    pos += (r.estimate.X.p - r.truth.X.p).squaredNorm();
    vel += (r.estimate.X.v - r.truth.X.v).squaredNorm();
    tilt += r.eulerErr_rad[1] * r.eulerErr_rad[1] + r.eulerErr_rad[2] * r.eulerErr_rad[2];
    yaw += r.eulerErr_rad[0] * r.eulerErr_rad[0];
    ++cnt;
  }
  if (cnt > 0)
  {
    const double c = static_cast<double>(cnt);
    S.aneesNav = nees / c / 9.0;
    S.posRms_m = std::sqrt(pos / c);
    S.velRms_m_s = std::sqrt(vel / c);
    S.tiltRms_deg = std::sqrt(tilt / c / 2.0) / kDeg;
    S.yawRms_deg = std::sqrt(yaw / c) / kDeg;
  }
  double nisP = 0.0, nisV = 0.0, nisB = 0.0;
  std::size_t nP = 0, nV = 0, nB = 0;
  for (const UpdateRow& u : res.updates)
  {
    if (u.t < tHalf)
      continue;
    switch (u.id)
    {
      case MeasurementId::GnssPosition:
        nisP += u.nis / u.dim;
        ++nP;
        break;
      case MeasurementId::GnssVelocity:
        nisV += u.nis / u.dim;
        ++nV;
        break;
      case MeasurementId::Barometer:
        nisB += u.nis / u.dim;
        ++nB;
        break;
    }
  }
  S.nisGnssPosition = nP ? nisP / static_cast<double>(nP) : 0.0;
  S.nisGnssVelocity = nV ? nisV / static_cast<double>(nV) : 0.0;
  S.nisBaro = nB ? nisB / static_cast<double>(nB) : 0.0;
  return res;
}

/// Dispatch on a run-time convention.
[[nodiscard]] inline ReplayResult
replay(ErrorConvention c, const LogSet& logs, const Configuration& cfg, const ReplayOptions& opt)
{
  return c == ErrorConvention::RightInvariant ? replay<ErrorConvention::RightInvariant>(logs, cfg, opt) :
                                                replay<ErrorConvention::LeftInvariant>(logs, cfg, opt);
}

// ---- output -----------------------------------------------------------------

inline void writeStates(std::ostream& out, const ReplayResult& r, const Configuration& cfg)
{
  constexpr double kDeg = std::numbers::pi / 180.0;
  const int n = cfg.dimension();
  out << "t_s,est_p_x,est_p_y,est_p_z,est_v_x,est_v_y,est_v_z,truth_p_x,truth_p_y,truth_p_z,truth_v_x,truth_v_y,"
         "truth_v_z,err_yaw_deg,err_pitch_deg,err_roll_deg,nees_nav";
  for (int i = 0; i < n; ++i)
    out << ",err_" << i;
  for (int i = 0; i < n; ++i)
    out << ",sigma_" << i;
  out << ",est_bg_x,est_bg_y,est_bg_z,est_ba_x,est_ba_y,est_ba_z,est_bp\n";
  for (const StateRow& s : r.states)
  {
    out << s.t;
    for (int i = 0; i < 3; ++i)
      out << ',' << s.estimate.X.p[i];
    for (int i = 0; i < 3; ++i)
      out << ',' << s.estimate.X.v[i];
    for (int i = 0; i < 3; ++i)
      out << ',' << s.truth.X.p[i];
    for (int i = 0; i < 3; ++i)
      out << ',' << s.truth.X.v[i];
    for (int i = 0; i < 3; ++i)
      out << ',' << s.eulerErr_rad[i] / kDeg;
    out << ',' << s.neesNav;
    for (int i = 0; i < n; ++i)
      out << ',' << s.error[i];
    for (int i = 0; i < n; ++i)
      out << ',' << s.sigma[i];
    for (int i = 0; i < 3; ++i)
      out << ',' << s.estimate.gyroBias[i];
    for (int i = 0; i < 3; ++i)
      out << ',' << s.estimate.accelBias[i];
    out << ',' << s.estimate.baroBias << '\n';
  }
}

inline void writeUpdates(std::ostream& out, const ReplayResult& r)
{
  out << "t_s,measurement,dim,nis\n";
  for (const UpdateRow& u : r.updates)
    out << u.t << ',' << measurementSpec(u.id).name << ',' << u.dim << ',' << u.nis << '\n';
}

inline void writeSummary(std::ostream& out, const ReplaySummary& s)
{
  out << "convention: " << toString(s.convention) << '\n'
      << "time base: host = " << s.timeBase.offset << " + " << s.timeBase.scale << " * part over " << s.timeBase.samples
      << " IMU samples, residual rms " << s.timeBase.rmsResidual * 1e3 << " ms, max " << s.timeBase.maxResidual * 1e3
      << " ms\n"
      << "duration: " << s.duration_s << " s, " << s.imuSamples << " IMU steps, " << s.gnssPositionUpdates
      << " GNSS position, " << s.gnssVelocityUpdates << " GNSS velocity, " << s.baroUpdates << " barometer updates, "
      << s.skippedFixes << " fixes skipped\n"
      << "second half: ANEES (nav core, /9) " << s.aneesNav << ", NIS/dim GNSS pos " << s.nisGnssPosition
      << ", GNSS vel " << s.nisGnssVelocity << ", baro " << s.nisBaro << '\n'
      << "second half RMS: position " << s.posRms_m << " m, velocity " << s.velRms_m_s << " m/s, tilt " << s.tiltRms_deg
      << " deg, yaw " << s.yawRms_deg << " deg\n"
      << "(bias NEES needs the realised biases, which the sensor FMUs do not publish yet)\n";
}

}  // namespace Aetherion::Estimation::Replay
