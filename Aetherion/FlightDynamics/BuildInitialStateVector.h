// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// BuildInitialStateVector.h
//
// Builds the 1×14 flat initial state vector from a RigidBody::Config,
// using the existing Aetherion coordinate transforms.
//
// State layout (StateLayout.h):
//
//   Index  Segment   Symbol     Description
//   ─────────────────────────────────────────────────────────────────────────
//   0..2   IDX_P     r_ECI      Position in ECI frame                  [m]
//   3..6   IDX_Q     q_EB       Attitude quaternion body→ECI  [w, x, y, z]
//   7..9   IDX_W     ω_B        Angular velocity in body frame      [rad/s]
//   10..12 IDX_V     v_ECI      Linear velocity in ECI frame            [m/s]
//   13     IDX_M     m          Vehicle mass                            [kg]
// ------------------------------------------------------------------------------

#pragma once

#include <numbers>
#include <cmath>

#include <Eigen/Dense>

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Environment/WGS84.h>      // kRotationRate_rad_s — single source
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/StateLayout.h>

namespace Aetherion::FlightDynamics {

    // Degrees → radians conversion factor
    inline constexpr double kDegreesToRadians = std::numbers::pi / 180.0;

    // Earth rotation rate — sourced from WGS84.h; no longer a local literal.
    // Kept as a named alias here so existing call sites that spell out
    // FlightDynamics::kEarthRotationRate_rad_s continue to compile.
    inline constexpr double kEarthRotationRate_rad_s =
        Environment::WGS84::kRotationRate_rad_s;

    // =========================================================================
    // BuildInitialStateVector (primary overload)
    //
    // @param cfg           Fully populated RigidBody::Config
    // @param theta_era_rad Earth Rotation Angle at t0  [rad]
    // @return              Eigen column vector of length 14
    // =========================================================================
    inline Eigen::Matrix<double, RigidBody::StateLayout::N, 1>
        BuildInitialStateVector(
            const RigidBody::Config& cfg,
            const double             theta_era_rad)
    {
        using namespace RigidBody;
        using namespace Coordinate;

        const double lat_rad = cfg.pose.lat_deg * kDegreesToRadians;
        const double lon_rad = cfg.pose.lon_deg * kDegreesToRadians;
        const double h_m = cfg.pose.alt_m;
        const double az_rad = cfg.pose.azimuth_deg * kDegreesToRadians;
        const double zen_rad = cfg.pose.zenith_deg * kDegreesToRadians;
        const double roll_rad = cfg.pose.roll_deg * kDegreesToRadians;

        // 1) ECI position: geodetic → ECEF → ECI
        const Vec3<double> r_ecef = GeodeticToECEF(lat_rad, lon_rad, h_m);
        const Vec3<double> r_eci = ECEFToECI(r_ecef, theta_era_rad);

        // 2) Attitude quaternion: NED launch angles → body→ECI quaternion
        const LaunchStateECI<double> launch =
            MakeLaunchStateECI(lat_rad, lon_rad, h_m,
                az_rad, zen_rad, roll_rad,
                theta_era_rad);
        const Quat<double>& q_EB = launch.q_EB;

        // 3) ECI velocity: NED → ECEF → ECI
        const Vec3<double> v_ned{
            cfg.velocityNED.north_mps,
            cfg.velocityNED.east_mps,
            cfg.velocityNED.down_mps
        };
        const Vec3<double> v_eci =
            ECEFToECI(NEDToECEF(v_ned, lat_rad, lon_rad), theta_era_rad);

        // 4) Body angular velocity
        const double omega_x = cfg.bodyRates.roll_rad_s;
        const double omega_y = cfg.bodyRates.pitch_rad_s;
        const double omega_z = cfg.bodyRates.yaw_rad_s;

        // 5) Initial mass
        const double mass_kg = cfg.inertialParameters.mass_kg;

        // Assemble flat state vector
        Eigen::Matrix<double, StateLayout::N, 1> x0;
        x0.setZero();

        x0[StateLayout::IDX_P + 0] = r_eci[0];
        x0[StateLayout::IDX_P + 1] = r_eci[1];
        x0[StateLayout::IDX_P + 2] = r_eci[2];

        x0[StateLayout::IDX_Q + 0] = q_EB[0]; // w
        x0[StateLayout::IDX_Q + 1] = q_EB[1]; // x
        x0[StateLayout::IDX_Q + 2] = q_EB[2]; // y
        x0[StateLayout::IDX_Q + 3] = q_EB[3]; // z

        x0[StateLayout::IDX_W + 0] = omega_x;
        x0[StateLayout::IDX_W + 1] = omega_y;
        x0[StateLayout::IDX_W + 2] = omega_z;

        x0[StateLayout::IDX_V + 0] = v_eci[0];
        x0[StateLayout::IDX_V + 1] = v_eci[1];
        x0[StateLayout::IDX_V + 2] = v_eci[2];

        x0[StateLayout::IDX_M] = mass_kg;

        return x0;
    }

} // namespace Aetherion::FlightDynamics
