// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Sensor fusion step 6: the reference invariant EKF against an analytic
// truth. The truth is a level, coordinated turn in the LCI frame (so heading
// is observable), the IMU is MEMS-grade with constant biases, GNSS gives
// position and velocity at 5 Hz, the barometer pressure altitude at 10 Hz
// with a constant bias. Deterministic noise. The filter is run in both
// conventions on the same samples, from a deliberately wrong initial state.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <vector>

#include <Aetherion/Estimation/EKF.h>

#include "SyntheticTruth.h"

using namespace Aetherion::Estimation;
namespace SO3 = Aetherion::ODE::RKMK::Lie::SO3;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

namespace
{
constexpr double kDeg = std::numbers::pi / 180.0;
const Configuration& cfg = kNavBaroGnss16;

LciFrame<double> testFrame() { return LciFrame<double>::FromGeodetic(36.0192 * kDeg, -75.6744 * kDeg, 3.0, 0.3); }

// The manoeuvring analytic truth and the platform-independent Gaussian
// generator shared with the replay test; see SyntheticTruth.h for why the
// truth must bank and pitch and why std::normal_distribution is not used.
using Truth = Aetherion::Estimation::Testing::SyntheticTruth;
using PortableGaussian = Aetherion::Estimation::Testing::PortableGaussian;

struct Sensors
{
  Vec3 gyroBias{ 0.002, -0.001, 0.0015 };
  Vec3 accelBias{ 0.05, -0.02, 0.03 };
  double baroBias = 12.0;
  double gyroNoise = 5e-5;   // rad/s/sqrt(Hz)
  double accelNoise = 7e-4;  // m/s^2/sqrt(Hz)
  double gnssPosSigmaH = 1.5, gnssPosSigmaV = 3.0, gnssVelSigma = 0.1;
  double baroSigma = 1.0;
};

// Run one filter over the scenario; returns (final NEES-average over the
// last half, final state, final P) and checks each update's NIS is sane.
template <ErrorConvention C>
struct Run
{
  EstimationState<double> xEnd;
  MatX P;
  double neesMean = 0.0;
  double nisPosMean = 0.0;
  double nisBaroMean = 0.0;
  int updates = 0;
};

template <ErrorConvention C>
Run<C> runFilter(const Truth& truth,
                 const Sensors& sn,
                 const EstimationState<double>& x0,
                 const MatX& P0_phys,
                 double T,
                 unsigned seed)
{
  const LciFrame<double> frame = testFrame();
  const MeasurementModels<double> meas(frame);
  const ProcessNoise<double> q{ sn.gyroNoise, sn.accelNoise, 2e-6, 2e-5, 0.05 };
  InvariantEKF<double, C> ekf(cfg, frame, q);
  ekf.initialise(x0, ekf.covarianceFromPhysical(x0, P0_phys));

  PortableGaussian N01(seed);
  auto noise3 = [&](double s) { return Vec3(s * N01(), s * N01(), s * N01()); };

  const double dt = 0.01;
  const int n = static_cast<int>(std::lround(T / dt));
  const int gnssEvery = 20, baroEvery = 10;
  const double sq = std::sqrt(1.0 / dt);

  Run<C> out;
  double neesSum = 0.0;
  int neesCount = 0, nPos = 0, nBaro = 0;
  const Mat3 SigmaPos = Vec3(sn.gnssPosSigmaH * sn.gnssPosSigmaH,
                             sn.gnssPosSigmaH * sn.gnssPosSigmaH,
                             sn.gnssPosSigmaV * sn.gnssPosSigmaV)
                            .asDiagonal();
  const Mat3 SigmaVel = Mat3::Identity() * sn.gnssVelSigma * sn.gnssVelSigma;

  for (int k = 0; k < n; ++k)
  {
    // IMU sampled at mid-step: the zero-order hold of propagate() is then
    // a midpoint rule and the truth/model mismatch is third order in dt.
    const ImuSample<double> ideal = truth.imu((k + 0.5) * dt, frame);
    const ImuSample<double> u{ ideal.gyro_rad_s + sn.gyroBias + noise3(sn.gyroNoise * sq),
                               ideal.accel_m_s2 + sn.accelBias + noise3(sn.accelNoise * sq) };
    ekf.predict(u, dt);

    const double t1 = (k + 1) * dt;
    const EstimationState<double> xt1 = truth.at(t1);
    if ((k + 1) % gnssEvery == 0)
    {
      const Vec3 yp = xt1.X.p + Vec3(sn.gnssPosSigmaH * N01(), sn.gnssPosSigmaH * N01(), sn.gnssPosSigmaV * N01());
      const Vec3 yv = xt1.X.v + noise3(sn.gnssVelSigma);
      const auto rp = ekf.updateGnssPosition(yp, SigmaPos);
      const auto rv = ekf.updateGnssVelocity(yv, SigmaVel);
      REQUIRE(rp.applied);
      REQUIRE(rv.applied);
      REQUIRE(rp.dim == 3);
      if (t1 > T / 2)
      {
        out.nisPosMean += rp.nis;
        ++nPos;
      }
    }
    if ((k + 1) % baroEvery == 0)
    {
      const double yb = meas.ellipsoidalHeight(xt1.X.p) + sn.baroBias + sn.baroSigma * N01();
      const auto rb = ekf.updateBarometer(yb, sn.baroSigma);
      REQUIRE(rb.applied);
      if (t1 > T / 2)
      {
        out.nisBaroMean += rb.nis;
        ++nBaro;
      }
    }
    if (t1 > T / 2)
    {
      EstimationState<double> xtb = xt1;
      xtb.gyroBias = sn.gyroBias;
      xtb.accelBias = sn.accelBias;
      xtb.baroBias = sn.baroBias;
      neesSum += ekf.nees(xtb);
      ++neesCount;
    }
  }
  out.xEnd = ekf.state();
  out.P = ekf.covariance();
  out.neesMean = neesSum / neesCount;
  out.nisPosMean /= nPos;
  out.nisBaroMean /= nBaro;
  out.updates = nPos + nBaro;
  return out;
}

MatX physicalP0()
{
  VecX s(cfg.dimension());
  s << 2.0 * kDeg, 2.0 * kDeg, 10.0 * kDeg, 2.0, 2.0, 2.0, 30.0, 30.0, 30.0, 0.005, 0.005, 0.005, 0.1, 0.1, 0.1, 20.0;
  return s.cwiseProduct(s).asDiagonal();
}

EstimationState<double> perturbedStart(const Truth& truth)
{
  EstimationState<double> x = truth.at(0.0);
  x.X.R = SO3::Exp_R(Vec3(1.0 * kDeg, -0.5 * kDeg, 5.0 * kDeg)) * x.X.R;  // 5 deg of heading, ~1 deg tilt
  x.X.v += Vec3(1.0, -1.0, 0.5);
  x.X.p += Vec3(20.0, -15.0, 10.0);
  // biases start at zero: the truth's are to be learned
  return x;
}

template <class R>
void checkConverged(const R& run, const Truth& truth, const Sensors& sn, double T)
{
  const EstimationState<double> xt = truth.at(T);
  const Vec3 dp = run.xEnd.X.p - xt.X.p;
  const Vec3 dv = run.xEnd.X.v - xt.X.v;
  const Vec3 dth = SO3::Log_R(Mat3(xt.X.R * run.xEnd.X.R.transpose()));
  INFO("pos err " << dp.transpose() << " | vel err " << dv.transpose() << " | att err [deg] "
                  << (dth / kDeg).transpose());
  INFO("bias err gyro " << (run.xEnd.gyroBias - sn.gyroBias).transpose() << " accel "
                        << (run.xEnd.accelBias - sn.accelBias).transpose() << " baro "
                        << run.xEnd.baroBias - sn.baroBias);
  INFO("NEES mean " << run.neesMean << " (16 dof), NIS pos " << run.nisPosMean << " (3), baro " << run.nisBaroMean
                    << " (1)");
  // Position and velocity are GNSS-limited; tilt is levelled by the
  // manoeuvre; yaw, with no magnetometer, is the weakly observable one
  // (doc/sensor_fusion.rst, "Validation plan") and comes down from 5 deg
  // slowly. The NEES/NIS checks below are what say these errors are the
  // size the filter claims.
  CHECK(dp.norm() < 1.0);
  CHECK(dv.norm() < 0.2);
  CHECK(std::abs(dth[0]) < 0.8 * kDeg);
  CHECK(std::abs(dth[1]) < 0.8 * kDeg);
  CHECK(std::abs(dth[2]) < 1.5 * kDeg);
  CHECK((run.xEnd.gyroBias - sn.gyroBias).norm() < 1.5e-3);
  CHECK((run.xEnd.accelBias - sn.accelBias).norm() < 0.03);
  CHECK(std::abs(run.xEnd.baroBias - sn.baroBias) < 2.0);
  // Consistency: NEES/16 and NIS/dim of order one.
  CHECK(run.neesMean / 16.0 > 0.2);
  CHECK(run.neesMean / 16.0 < 4.0);
  CHECK(run.nisPosMean / 3.0 > 0.3);
  CHECK(run.nisPosMean / 3.0 < 3.0);
  CHECK(run.nisBaroMean > 0.3);
  CHECK(run.nisBaroMean < 3.0);
  // P symmetric positive definite.
  CHECK((run.P - run.P.transpose()).norm() < 1e-12 * run.P.norm());
  CHECK(run.P.ldlt().isPositive());
}
}  // namespace

// ---------------------------------------------------------------------------
TEST_CASE("Exact sensors and exact start: the filter stays on the truth", "[estimation][ekf]")
{
  const Truth truth;
  Sensors sn;
  sn.gyroNoise = sn.accelNoise = 0.0;
  sn.gnssPosSigmaH = sn.gnssPosSigmaV = 1e-6;  // "noise" only in the covariance scale
  sn.gnssVelSigma = 1e-6;
  sn.baroSigma = 1e-6;
  sn.gyroBias.setZero();
  sn.accelBias.setZero();
  sn.baroBias = 0.0;

  const LciFrame<double> frame = testFrame();
  const ProcessNoise<double> q{ 1e-5, 1e-4, 1e-7, 1e-6, 1e-3 };
  InvariantEKF<double, ErrorConvention::RightInvariant> ekf(cfg, frame, q);
  ekf.initialise(truth.at(0.0), MatX::Identity(16, 16) * 1e-6);

  // The analytic body rate against a numerical one, at a few times.
  for (const double t : { 0.0, 1.3, 7.9 })
  {
    const double hh = 1e-6;
    const Vec3 numeric = SO3::Log_R(Mat3(truth.at(t).X.R.transpose() * truth.at(t + hh).X.R)) / hh;
    REQUIRE((truth.bodyRate(t + hh / 2) - numeric).norm() < 1e-7);
  }

  const double dt = 0.01;
  for (int k = 0; k < 2000; ++k)
    ekf.predict(truth.imu((k + 0.5) * dt, frame), dt);
  // 20 s of pure dead reckoning on an exact IMU: only the midpoint-rule
  // error of the propagation remains, cm over a 0.05 rad/s turn at 10 ms.
  const auto xt = truth.at(20.0);
  INFO("dead-reckoning error: pos " << (ekf.state().X.p - xt.X.p).norm() << " m, vel "
                                    << (ekf.state().X.v - xt.X.v).norm() << " m/s, att "
                                    << SO3::Log_R(Mat3(xt.X.R * ekf.state().X.R.transpose())).norm() / kDeg << " deg");
  REQUIRE((ekf.state().X.p - xt.X.p).norm() < 0.05);
  REQUIRE((ekf.state().X.v - xt.X.v).norm() < 0.005);
  REQUIRE(SO3::Log_R(Mat3(xt.X.R * ekf.state().X.R.transpose())).norm() < 0.01 * kDeg);
}

TEST_CASE("Right-invariant filter converges from a wrong start and is consistent", "[estimation][ekf]")
{
  const Truth truth;
  const Sensors sn;
  const double T = 240.0;
  const auto run = runFilter<ErrorConvention::RightInvariant>(truth, sn, perturbedStart(truth), physicalP0(), T, 1);
  checkConverged(run, truth, sn, T);
}

TEST_CASE("Left-invariant filter converges from a wrong start and is consistent", "[estimation][ekf]")
{
  const Truth truth;
  const Sensors sn;
  const double T = 240.0;
  const auto run = runFilter<ErrorConvention::LeftInvariant>(truth, sn, perturbedStart(truth), physicalP0(), T, 1);
  checkConverged(run, truth, sn, T);
}

TEST_CASE("Both conventions agree in normal operation", "[estimation][ekf]")
{
  const Truth truth;
  const Sensors sn;
  const double T = 240.0;
  const auto ri = runFilter<ErrorConvention::RightInvariant>(truth, sn, perturbedStart(truth), physicalP0(), T, 1);
  const auto li = runFilter<ErrorConvention::LeftInvariant>(truth, sn, perturbedStart(truth), physicalP0(), T, 1);
  const Vec3 dAtt = SO3::Log_R(Mat3(ri.xEnd.X.R * li.xEnd.X.R.transpose())) / kDeg;
  INFO("RI - LI: pos " << (ri.xEnd.X.p - li.xEnd.X.p).norm() << " m, vel " << (ri.xEnd.X.v - li.xEnd.X.v).norm()
                       << " m/s, att [deg] " << dAtt.transpose());
  // Same samples, same start: the two differ only by the second-order
  // update terms (doc/sensor_fusion.rst "First-order equivalence"), which
  // matter where the correction was large -- the 5 deg of initial heading
  // that yaw, weakly observable, is still working off. Everything the
  // measurements pin down agrees closely.
  CHECK((ri.xEnd.X.p - li.xEnd.X.p).norm() < 0.1);
  CHECK((ri.xEnd.X.v - li.xEnd.X.v).norm() < 0.02);
  // Attitude: both filters are consistent (previous test cases) and each is
  // within the convergence bounds of checkConverged(); their difference can
  // be up to the sum of the two errors when they straddle the truth, which
  // yaw -- weakly observable, still working off the 5 deg start -- does.
  CHECK(std::abs(dAtt[0]) < 1.6);
  CHECK(std::abs(dAtt[1]) < 1.6);
  CHECK(std::abs(dAtt[2]) < 3.0);
  // And the covariances agree through the adjoint of the estimate.
  const MatX Ad = [&] {
    MatX A = MatX::Identity(16, 16);
    A.block<9, 9>(0, 0) = SE23<double>::Adjoint(li.xEnd.X);
    return A;
  }();
  const MatX P_li_in_ri = Ad * li.P * Ad.transpose();
  INFO("|Ad P_LI Ad^T - P_RI| / |P_RI| = " << (P_li_in_ri - ri.P).norm() / ri.P.norm());
  CHECK((P_li_in_ri - ri.P).norm() < 0.1 * ri.P.norm());
}

TEST_CASE("A measurement the configuration does not carry is skipped", "[estimation][ekf]")
{
  const Configuration noBaro{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias },
                              { MeasurementId::GnssPosition, MeasurementId::GnssVelocity } };
  const Truth truth;
  // Left-invariant: H for position is [0 0 I], so with P = I and R = I the
  // innovation covariance is exactly 2I and the NIS of z = (1,0,0) is 1/2.
  InvariantEKF<double, ErrorConvention::LeftInvariant> ekf(
      noBaro, testFrame(), ProcessNoise<double>{ 1e-4, 1e-3, 1e-6, 1e-5, 0.0 });
  const MatX P0 = MatX::Identity(noBaro.dimension(), noBaro.dimension());
  ekf.initialise(truth.at(0.0), P0);

  const auto r = ekf.updateBarometer(123.0, 1.0);
  REQUIRE_FALSE(r.applied);
  REQUIRE(r.dim == 0);
  REQUIRE((ekf.covariance() - P0).norm() == 0.0);
  REQUIRE((ekf.state().X.p - truth.at(0.0).X.p).norm() == 0.0);

  const auto rp = ekf.updateGnssPosition(truth.at(0.0).X.p + truth.at(0.0).X.R * Vec3(1.0, 0.0, 0.0), Mat3::Identity());
  REQUIRE(rp.applied);
  REQUIRE(rp.dim == 3);
  REQUIRE(rp.nis == Catch::Approx(0.5));
  // K = P H^T S^-1 = 1/2 on the position block: the estimate moves half way.
  REQUIRE((ekf.state().X.p - truth.at(0.0).X.p).norm() == Catch::Approx(0.5).margin(1e-12));
}
