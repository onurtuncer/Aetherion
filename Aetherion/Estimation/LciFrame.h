// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// LciFrame.h
//
// The launch-centred inertial (LCI) navigation frame of the estimator
// (doc/sensor_fusion.rst, "Navigation frame"): origin at the launch point,
// axes equal to local NED at t0, then frozen in inertial space. With W the
// ECI frame,
//
//   r_W0 = R_WE(t0) r_E(lat0, lon0, h0),     R_WL = R_WE(t0) R_EN(lat0, lon0),
//   p    = R_WL^T (r_W - r_W0),               g_L(p) = R_WL^T g_W(r_W0 + R_WL p).
//
// L differs from ECI by a constant rotation and translation, so there are
// no Earth-rate or Coriolis terms in the strapdown kinematics; the only
// state dependence of the model is gravity evaluated at the estimated
// position. The frame is also where the GNSS measurement models map into,
// through the time-dependent R_WE(t) (MeasurementModels.h).
// ------------------------------------------------------------------------------

#pragma once

#include <array>

#include <Eigen/Dense>

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Environment/Gravity.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/ODE/RKMK/Core/Scalar.h>  // Core::sqrt_s

namespace Aetherion::Estimation {

    template <class S>
    struct LciFrame
    {
        using Mat3 = Eigen::Matrix<S, 3, 3>;
        using Vec3 = Eigen::Matrix<S, 3, 1>;

        Vec3 r_W0 = Vec3::Zero();      ///< ECI position of the origin [m]
        Mat3 R_WL = Mat3::Identity();  ///< LCI -> ECI; columns are N, E, D of the origin at t0
        S era0_rad = S(0);  ///< Earth rotation angle at t0; the ECEF->ECI mapping of a later measurement needs era(t)

  /// Earth rotation angle at t0 + t: what the ECEF -> ECI rotation of a
  /// measurement taken t seconds after the frame epoch uses.
        [[nodiscard]] S earthRotationAngle(const S& t_since_t0_s) const
        {
            return era0_rad + S(Environment::WGS84::kRotationRate_rad_s) * t_since_t0_s;
        }

  /// Frame from the geodetic origin (WGS-84 latitude, longitude, ellipsoidal
  /// height) and the Earth rotation angle at t0. These are the `lat0_deg`,
  /// `lon0_deg`, `alt0_ft` of the replayed run's sidecar, converted.
        [[nodiscard]] static LciFrame FromGeodetic(const S& lat0_rad, const S& lon0_rad, const S& h0_m, const S& era0_rad)
        {
            const auto r_E = Coordinate::GeodeticToECEF(lat0_rad, lon0_rad, h0_m);
            const auto r_W = Coordinate::ECEFToECI(r_E, era0_rad);
            const auto R = Coordinate::NEDToInertialRotationMatrix(lat0_rad, lon0_rad, era0_rad);  // row-major

            LciFrame f;
            f.era0_rad = era0_rad;
            f.r_W0 = Vec3(r_W[0], r_W[1], r_W[2]);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    f.R_WL(r, c) = R[3 * r + c];
            return f;
        }

        [[nodiscard]] Vec3 toLci(const Vec3& r_W) const { return R_WL.transpose() * (r_W - r_W0); }
        [[nodiscard]] Vec3 toEci(const Vec3& p) const { return r_W0 + R_WL * p; }
        [[nodiscard]] Vec3 rotateToLci(const Vec3& u_W) const { return R_WL.transpose() * u_W; }
        [[nodiscard]] Vec3 rotateToEci(const Vec3& u_L) const { return R_WL * u_L; }

  /// Gravitational acceleration at LCI position p, resolved in LCI: the
  /// WGS-84 central + J2 model of Environment/Gravity.h.
        [[nodiscard]] Vec3 gravity(const Vec3& p) const
        {
            const Vec3 r_W = toEci(p);
            const std::array<S, 3> g_W = Environment::J2(std::array<S, 3>{ r_W[0], r_W[1], r_W[2] });
            return rotateToLci(Vec3(g_W[0], g_W[1], g_W[2]));
        }

  /// Gradient dg_L/dp. Central term only, -(mu/r^3)(I - 3 u u^T) with
  /// u = r_W/|r_W|: of order 1.5e-6 s^-2 (the squared Schuler frequency).
  /// The J2 contribution to the gradient is another 1e-3 smaller and is
  /// omitted; the gradient enters only the error dynamics, never the state.
        [[nodiscard]] Mat3 gravityGradient(const Vec3& p) const
        {
            const Vec3 r_W = toEci(p);
            const S r2 = r_W.dot(r_W);
            const S inv_r = S(1) / ODE::RKMK::Core::sqrt_s(r2);
            const Vec3 u = r_W * inv_r;
            const S k = -S(Environment::WGS84::kGM_m3_s2) * inv_r * inv_r * inv_r;
            const Mat3 G_W = k * (Mat3::Identity() - S(3) * u * u.transpose());
            return R_WL.transpose() * G_W * R_WL;
        }
    };

}  // namespace Aetherion::Estimation
