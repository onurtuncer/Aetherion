// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// EKF.h
//
// The reference invariant EKF on SE2(3) x R^7 (sensor fusion step 6;
// doc/sensor_fusion.rst "Filter form"). Double precision is the intent --
// this is the specification the generated flight code is checked against
// (step 9) -- but it is scalar-templated like everything under it so the
// float32 rerun of step 7b is the same code.
//
// The convention C (left- or right-invariant, Blocks.h) is a template
// parameter: InvariantEKF<double, ErrorConvention::RightInvariant> and
// <..., LeftInvariant> are two filters on the same models, and the decision
// between them is taken on evidence, not here.
//
//   predict(u, dt)      x <- propagate(x, u, dt);  P <- Phi P Phi^T + Qd
//                       (ProcessModel::step<C>, trapezoidal F).
//   update*(...)        z, H, R from MeasurementModels<S>::*<C>;
//                       S = H P H^T + R,  K = P H^T S^-1,
//                       x <- x.retract<C>(K z),
//                       P <- (I - K H) P (I - K H)^T + K R K^T  (Joseph form).
//                       A measurement the configuration does not carry is
//                       skipped (state dimension never changes); the result
//                       says so.
//
// P is kept in the convention's error coordinates (for RI, world-frame
// quantities coupled through [p]x and [v]x; see the float32 finding in
// doc/sensor_fusion.rst). covarianceFromPhysical() maps an initial
// covariance given in physical coordinates (world attitude error, dv, dp,
// biases) into them, so both conventions can be started equivalently.
//
// No covariance reset after updates (standard IEKF practice; the reset is
// one of the open experiments). No delayed-measurement handling here: the
// replay harness owns time and feeds measurements in order.
//
// Consistency: nis of each update is returned; nees(truth) evaluates
// e^T P^-1 e with e = x.error<C>(truth), i.e. the true error in the filter's
// own coordinates, which is what the covariance claims to describe.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <Aetherion/Estimation/Blocks.h>
#include <Aetherion/Estimation/Configuration.h>
#include <Aetherion/Estimation/EstimationState.h>
#include <Aetherion/Estimation/LciFrame.h>
#include <Aetherion/Estimation/MeasurementModels.h>
#include <Aetherion/Estimation/ProcessModel.h>
#include <Aetherion/Spatial/Skew.h>

namespace Aetherion::Estimation
{

/// What an update did.
template <class S>
struct UpdateResult
{
  bool applied = false;  ///< false: the configuration does not carry this measurement, or it was rejected; nothing changed
  bool rejected = false; ///< true: innovation or covariance not finite, or S not positive definite; nothing changed
  int dim = 0;           ///< measurement dimension
  S nis = S(0);          ///< normalised innovation squared, z^T S^-1 z (chi^2 with `dim` dof if consistent)
};

template <class S, ErrorConvention C>
class InvariantEKF
{
public:
  using Mat3 = Eigen::Matrix<S, 3, 3>;
  using Vec3 = Eigen::Matrix<S, 3, 1>;
  using VecX = Eigen::Matrix<S, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;
  using State = EstimationState<S>;

  static constexpr ErrorConvention kConvention = C;

  InvariantEKF(Configuration cfg, LciFrame<S> frame, ProcessNoise<S> noise)
    : cfg_(cfg)
    , model_(frame)
    , measurements_(frame)
    , Qc_(model_.continuousNoiseCovariance(cfg, noise))
    , P_(MatX::Zero(cfg.dimension(), cfg.dimension()))
  {
  }

  /// Start from x0 with covariance P0 given in the convention's coordinates
  /// (use covarianceFromPhysical() to build it from physical sigmas).
  void initialise(const State& x0, const MatX& P0)
  {
    x_ = x0;
    P_ = P0;
  }

  [[nodiscard]] const State& state() const { return x_; }
  [[nodiscard]] const MatX& covariance() const { return P_; }
  [[nodiscard]] const Configuration& configuration() const { return cfg_; }
  [[nodiscard]] const StrapdownProcessModel<S>& processModel() const { return model_; }
  [[nodiscard]] const MeasurementModels<S>& measurementModels() const { return measurements_; }

  /// Map a covariance in physical coordinates -- world-frame attitude error,
  /// dv, dp (all LCI), then the bias blocks -- into this convention's error
  /// coordinates at state x, P = J P_phys J^T (first order):
  ///   RI: dv = nu + [phi]x v, dp = rho + [phi]x p, so J[nu,phi] = [v]x, J[rho,phi] = [p]x;
  ///   LI: phi, nu, rho are body-frame quantities, J = blockdiag(R^T, R^T, R^T).
  [[nodiscard]] MatX covarianceFromPhysical(const State& x, const MatX& P_phys) const
  {
    const int n = cfg_.dimension();
    MatX J = MatX::Identity(n, n);
    const int nav = cfg_.offset(BlockId::NavCore);
    const int iPhi = nav + NavCoreLayout::kAttitude;
    const int iNu = nav + NavCoreLayout::kVelocity;
    const int iRho = nav + NavCoreLayout::kPosition;
    if constexpr (C == ErrorConvention::RightInvariant)
    {
      J.template block<3, 3>(iNu, iPhi) = Spatial::skew(x.X.v);
      J.template block<3, 3>(iRho, iPhi) = Spatial::skew(x.X.p);
    }
    else
    {
      const Mat3 Rt = x.X.R.transpose();
      J.template block<3, 3>(iPhi, iPhi) = Rt;
      J.template block<3, 3>(iNu, iNu) = Rt;
      J.template block<3, 3>(iRho, iRho) = Rt;
    }
    return J * P_phys * J.transpose();
  }

  void predict(const ImuSample<S>& u, const S& dt)
  {
    MatX Phi, Qd;
    x_ = model_.template step<C>(cfg_, x_, u, dt, Qc_, Phi, Qd);
    P_ = Phi * P_ * Phi.transpose() + Qd;
    symmetrise();
  }

  /// GNSS position in LCI with its covariance in LCI (MeasurementModels::gnssPositionLci / nedToLci).
  UpdateResult<S> updateGnssPosition(const Vec3& y_L, const Mat3& Sigma_L)
  {
    if (!cfg_.has(MeasurementId::GnssPosition))
      return {};
    VecX z;
    MatX H, R;
    measurements_.template gnssPosition<C>(cfg_, x_, y_L, Sigma_L, z, H, R);
    return update(z, H, R);
  }

  UpdateResult<S> updateGnssVelocity(const Vec3& y_L, const Mat3& Sigma_L)
  {
    if (!cfg_.has(MeasurementId::GnssVelocity))
      return {};
    VecX z;
    MatX H, R;
    measurements_.template gnssVelocity<C>(cfg_, x_, y_L, Sigma_L, z, H, R);
    return update(z, H, R);
  }

  /// Pressure altitude [m] (MeasurementModels::pressureAltitude_m of the sample) and its sigma.
  UpdateResult<S> updateBarometer(const S& y_m, const S& sigma_m)
  {
    if (!cfg_.has(MeasurementId::Barometer))
      return {};
    VecX z;
    MatX H, R;
    measurements_.template barometer<C>(cfg_, x_, y_m, sigma_m, z, H, R);
    return update(z, H, R);
  }

  /// Normalised estimation error squared against the true state.
  [[nodiscard]] S nees(const State& truth) const
  {
    const VecX e = x_.template error<C>(cfg_, truth);
    return e.dot(P_.ldlt().solve(e));
  }

private:
  Configuration cfg_;
  StrapdownProcessModel<S> model_;
  MeasurementModels<S> measurements_;
  MatX Qc_;
  State x_;
  MatX P_;

  UpdateResult<S> update(const VecX& z, const MatX& H, const MatX& R)
  {
    UpdateResult<S> r;
    r.dim = static_cast<int>(z.size());

    // A measurement that is not a number, or whose innovation covariance does
    // not factorise, must not reach the state: one NaN in K makes every later
    // estimate NaN with no fault reported. Refuse it and say so instead.
    if (!z.allFinite() || !R.allFinite())
    {
      r.rejected = true;
      return r;
    }

    const int n = cfg_.dimension();
    const MatX PHt = P_ * H.transpose();
    const MatX Sm = H * PHt + R;
    const auto ldlt = Sm.ldlt();
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive())
    {
      r.rejected = true;
      return r;
    }
    const MatX K = ldlt.solve(PHt.transpose()).transpose();  // P H^T S^-1

    r.applied = true;
    r.nis = z.dot(ldlt.solve(z));

    const VecX delta = K * z;
    x_ = x_.template retract<C>(cfg_, delta);

    const MatX IKH = MatX::Identity(n, n) - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    symmetrise();
    return r;
  }

  void symmetrise() { P_ = S(0.5) * (P_ + P_.transpose()).eval(); }
};

}  // namespace Aetherion::Estimation
