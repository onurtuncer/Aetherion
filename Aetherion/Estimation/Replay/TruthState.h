// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TruthState.h
//
// The plant's truth, as the co-simulation host logs it (geodetic position,
// NED velocity, ZYX Euler angles body -> NED, at simulation time t), as an
// EstimationState in the LCI frame (sensor fusion step 6). With W the ECI
// frame, E ECEF at the Earth rotation angle era(t) = w_e t (Aetherion's
// plants start at era = 0, FlightDynamics/Policies/AeroPolicies.h), N the
// local NED at (lat, lon) and B the body:
//
//   r_W = R_WE(t) r_E(lat, lon, h),                      p = R_WL^T (r_W - r_W0)
//   v_W = R_WE(t) (R_EN v_N + w_e x r_E),                v = R_WL^T v_W
//   R_LB = R_WL^T R_WE(t) R_EN(lat, lon) R_NB(yaw, pitch, roll)
//
// The NED velocity the host logs is Earth-relative; the transport term
// makes it inertial, exactly as MeasurementModels::gnssVelocityLci does for
// a fix. Heights are ellipsoidal (`out.alt_m` is the plant's geodetic
// altitude). The Euler angles are singular at 90 deg pitch; that is the
// rocket's problem (TODO-sensor-fusion.md), not the F-16 cases'.
// ------------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include <Aetherion/Estimation/EstimationState.h>
#include <Aetherion/Estimation/LciFrame.h>
#include <Aetherion/Estimation/MeasurementModels.h>

namespace Aetherion::Estimation::Replay
{

/// ZYX Euler angles (yaw about z, then pitch about y, then roll about x),
/// body -> NED: R_NB = Rz(yaw) Ry(pitch) Rx(roll).
[[nodiscard]] inline Eigen::Matrix3d rotationNedFromBody(double yaw, double pitch, double roll)
{
  using Eigen::AngleAxisd;
  using Eigen::Vector3d;
  return (AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) *
          AngleAxisd(roll, Vector3d::UnitX()))
      .toRotationMatrix();
}

/// Rotation NED(lat, lon) -> ECEF, columns N, E, D (Coordinate::NEDToECEF as a matrix).
[[nodiscard]] inline Eigen::Matrix3d rotationEcefFromNed(double lat, double lon)
{
  const double sLat = std::sin(lat), cLat = std::cos(lat), sLon = std::sin(lon), cLon = std::cos(lon);
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(-sLat * cLon, -sLat * sLon, cLat);
  R.col(1) = Eigen::Vector3d(-sLon, cLon, 0.0);
  R.col(2) = Eigen::Vector3d(-cLat * cLon, -cLat * sLon, -sLat);
  return R;
}

/// Rotation ECEF -> ECI at Earth rotation angle era: R_3(-era).
[[nodiscard]] inline Eigen::Matrix3d rotationEciFromEcef(double era)
{
  const double c = std::cos(era), s = std::sin(era);
  Eigen::Matrix3d R;
  R << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return R;
}

struct GeodeticTruth
{
  double t = 0.0;  ///< simulation time [s]
  double lat_rad = 0.0;
  double lon_rad = 0.0;
  double alt_m = 0.0;                              ///< ellipsoidal
  Eigen::Vector3d vNed = Eigen::Vector3d::Zero();  ///< Earth-relative [m/s]
  double yaw_rad = 0.0, pitch_rad = 0.0, roll_rad = 0.0;
};

/// The navigation core of the truth in LCI; the bias members are left at
/// zero (the sensor FMUs do not publish their realised biases yet, Hemerion
/// step 3).
[[nodiscard]] inline EstimationState<double> truthToState(const GeodeticTruth& g, const LciFrame<double>& frame)
{
  const double era = frame.earthRotationAngle(g.t);
  const MeasurementModels<double> m(frame);
  const GnssFix<double> fix{ g.lat_rad, g.lon_rad, g.alt_m };

  EstimationState<double> x;
  x.X.p = m.gnssPositionLci(fix, era);
  x.X.v = m.gnssVelocityLci(fix, g.vNed, era);
  x.X.R = frame.R_WL.transpose() * rotationEciFromEcef(era) * rotationEcefFromNed(g.lat_rad, g.lon_rad) *
          rotationNedFromBody(g.yaw_rad, g.pitch_rad, g.roll_rad);
  return x;
}

/// Attitude of an LCI state as ZYX Euler angles body -> local NED at the
/// state's own position and time: what "heading and yaw covariance are
/// reported by rotating into the current local NED" means in practice.
[[nodiscard]] inline Eigen::Vector3d eulerNedFromState(const EstimationState<double>& x,
                                                       double t,
                                                       const LciFrame<double>& frame)
{
  const double era = frame.earthRotationAngle(t);
  const Eigen::Vector3d r_W = frame.toEci(x.X.p);
  const Eigen::Vector3d r_E = rotationEciFromEcef(era).transpose() * r_W;
  // Geodetic lat/lon of the point: Bowring, one iteration is plenty for the
  // NED axes (they are insensitive to the height).
  const double a = Environment::WGS84::kSemiMajorAxis_m, e2 = Environment::WGS84::kEccentricitySq;
  const double lon = std::atan2(r_E[1], r_E[0]);
  const double pxy = std::hypot(r_E[0], r_E[1]);
  double lat = std::atan2(r_E[2], pxy * (1.0 - e2));
  for (int i = 0; i < 3; ++i)
  {
    const double sl = std::sin(lat);
    const double N = a / std::sqrt(1.0 - e2 * sl * sl);
    lat = std::atan2(r_E[2] + e2 * N * sl, pxy);
  }
  const Eigen::Matrix3d R_NB =
      rotationEcefFromNed(lat, lon).transpose() * rotationEciFromEcef(era).transpose() * frame.R_WL * x.X.R;
  // ZYX from R_NB: yaw = atan2(R21, R11), pitch = -asin(R31), roll = atan2(R32, R33).
  return { std::atan2(R_NB(1, 0), R_NB(0, 0)), -std::asin(std::clamp(R_NB(2, 0), -1.0, 1.0)),
           std::atan2(R_NB(2, 1), R_NB(2, 2)) };
}

}  // namespace Aetherion::Estimation::Replay
