// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// SyntheticTruth.h
//
// The analytic truth the estimator tests share: a level circle in the LCI
// frame at speed V and yaw rate psi_dot, with the bank swinging about the
// coordinated value and the pitch oscillating, so that the attitude is
// observable. In a *steady* coordinated turn the specific force sits along
// body -z and every tilt error is indistinguishable from a body-frame
// accelerometer bias -- the filter then drifts along that direction with
// NEES growing while GNSS keeps position and velocity perfect. Any
// synthetic truth must bank and pitch (TODO-sensor-fusion.md, step 6).
//
// imu(t) returns the ideal IMU of this truth; sample it at mid-step when
// driving the zero-order-hold propagation, so the mismatch is third order.
// ------------------------------------------------------------------------------

#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

#include <Eigen/Dense>

#include <Aetherion/Estimation/EstimationState.h>
#include <Aetherion/Estimation/LciFrame.h>
#include <Aetherion/Estimation/ProcessModel.h>
#include <Aetherion/ODE/RKMK/Lie/SO3.h>

namespace Aetherion::Estimation::Testing
{

// Gaussian samples that are the same on every platform. std::normal_distribution
// is implementation-defined -- libstdc++ and MSVC draw different sequences
// from the same engine -- so a "deterministic" seed gave Linux CI a different
// 240 s run than Windows, and a bound set on one failed on the other.
// Box-Muller on the engine's own bits, as Environment/DrydenTurbulence.h does.
class PortableGaussian
{
public:
  explicit PortableGaussian(std::uint64_t seed) : rng_(seed) {}

  double operator()()
  {
    if (hasSpare_)
    {
      hasSpare_ = false;
      return spare_;
    }
    double u1 = 0.0;
    do
      u1 = uniform();
    while (u1 <= 0.0);
    const double u2 = uniform();
    const double r = std::sqrt(-2.0 * std::log(u1));
    spare_ = r * std::sin(2.0 * std::numbers::pi * u2);
    hasSpare_ = true;
    return r * std::cos(2.0 * std::numbers::pi * u2);
  }

private:
  double uniform() { return static_cast<double>(rng_() >> 11) * 0x1.0p-53; }  // [0, 1), 53 bits

  std::mt19937_64 rng_;
  double spare_ = 0.0;
  bool hasSpare_ = false;
};

struct SyntheticTruth
{
  static constexpr double kDeg = std::numbers::pi / 180.0;
  using Vec3 = Eigen::Vector3d;

  double V = 120.0;
  double psiDot = 0.05;                              // rad/s: a 126 s circle, V*psiDot = 6 m/s^2 (0.6 g) lateral
  double h = 500.0;                                  // m above the LCI origin
  double bank0 = std::atan2(120.0 * 0.05, 9.80665);  // coordinated: ~31.5 deg
  double bankAmp = 15.0 * kDeg, bankRate = 0.3;
  double pitchAmp = 8.0 * kDeg, pitchRate = 0.5;

  [[nodiscard]] double bank(double t) const { return bank0 + bankAmp * std::sin(bankRate * t); }
  [[nodiscard]] double theta(double t) const { return pitchAmp * std::sin(pitchRate * t); }

  /// Navigation core at t; biases zero.
  [[nodiscard]] EstimationState<double> at(double t) const
  {
    namespace SO3 = ODE::RKMK::Lie::SO3;
    const double psi = psiDot * t;
    EstimationState<double> x;
    x.X.R =
        SO3::Exp_R(Vec3(0.0, 0.0, psi)) * SO3::Exp_R(Vec3(bank(t), 0.0, 0.0)) * SO3::Exp_R(Vec3(0.0, theta(t), 0.0));
    x.X.v = Vec3(V * std::cos(psi), V * std::sin(psi), 0.0);
    x.X.p = Vec3(V / psiDot * std::sin(psi), V / psiDot * (1.0 - std::cos(psi)), -h);
    return x;
  }

  [[nodiscard]] Vec3 accelerationLci(double t) const
  {
    const double psi = psiDot * t;
    return { -V * psiDot * std::sin(psi), V * psiDot * std::cos(psi), 0.0 };
  }

  /// Body rate (R^T R_dot)^vee for R = Rz(psi) Rx(phi) Ry(theta):
  ///   omega = Ry^T [ Rx^T (0, 0, psi_dot) + (phi_dot, 0, 0) ] + (0, theta_dot, 0)
  ///         = (c_th phi_dot - s_th c_phi psi_dot,  s_phi psi_dot + theta_dot,  s_th phi_dot + c_th c_phi psi_dot).
  [[nodiscard]] Vec3 bodyRate(double t) const
  {
    const double th = theta(t), ph = bank(t);
    const double thDot = pitchAmp * pitchRate * std::cos(pitchRate * t);
    const double phDot = bankAmp * bankRate * std::cos(bankRate * t);
    const double cth = std::cos(th), sth = std::sin(th), cph = std::cos(ph), sph = std::sin(ph);
    return { cth * phDot - sth * cph * psiDot, sph * psiDot + thDot, sth * phDot + cth * cph * psiDot };
  }

  /// The ideal IMU of this truth at time t: inertial rate and specific force, body frame.
  [[nodiscard]] ImuSample<double> imu(double t, const LciFrame<double>& frame) const
  {
    const EstimationState<double> x = at(t);
    return { bodyRate(t), x.X.R.transpose() * (accelerationLci(t) - frame.gravity(x.X.p)) };
  }
};

}  // namespace Aetherion::Estimation::Testing
