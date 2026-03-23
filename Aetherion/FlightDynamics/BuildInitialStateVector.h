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
// Builds the 1×14 flat initial state vector from a SimulationConfig,
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
//
// Inputs from SimulationConfig:
//   cfg.pose                       → PoseWGS84_NED  (deg, deg, m, deg, deg, deg)
//   cfg.velocityNED                → VelocityNED    (m/s, NED)
//   cfg.bodyRates → roll/pitch/yaw rates [rad/s] in body
//   cfg.inertialParameters.mass_kg → initial mass  [kg]
//   cfg.simulation.startTime       → t0 used to compute Earth rotation angle
//
// Earth Rotation Angle (ERA):
//   A simple linear model is used:
//     θ_ERA(t0) = ω_E * t0
//   where ω_E = 7.2921150e-5 rad/s (WGS-84).
//   Pass a pre-computed ERA directly via the overload that takes theta_era_rad.
//
// Coordinate transform chain (all from LocalToInertial.h):
//
//   Geodetic (lat,lon,h)
//       │  GeodeticToECEF()
//       ▼
//     ECEF r_ecef
//       │  ECEFToECI(theta_era)
//       ▼
//     ECI  r_eci                 ← state[IDX_P .. IDX_P+2]
//
//   PoseWGS84_NED (az, zen, roll) + (lat, lon, theta_era)
//       │  MakeLaunchStateECI()
//       ▼
//     q_EB  [w,x,y,z]            ← state[IDX_Q .. IDX_Q+3]
//
//   VelocityNED (vN, vE, vD)
//       │  NEDToECEF()  then ECEFToECI()
//       ▼
//     v_ECI                      ← state[IDX_V .. IDX_V+2]
//
//   BodyRates
//     (roll_rad_s, pitch_rad_s, yaw_rad_s)  ← state[IDX_W .. IDX_W+2]
//
//   InertialParameters.mass_kg             ← state[IDX_M]
//
// ------------------------------------------------------------------------------

#pragma once

#include <numbers>
#include <cmath>

#include <Eigen/Dense>

// Aetherion coordinate transforms
#include <Aetherion/Coordinate/LocalToInertial.h>   // GeodeticToECEF, NEDToECEF,
                                                      // ECEFToECI, MakeLaunchStateECI

// FlightDynamics types
#include <Aetherion/RigidBody/Config.h>

// State layout
#include <Aetherion/RigidBody/StateLayout.h>

namespace Aetherion::FlightDynamics {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    inline constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
    inline constexpr double kEarthRotationRate_rad_s = 7.2921150e-5; // WGS-84 [rad/s]

    // =========================================================================
    // BuildInitialStateVector
    //
    // Primary overload: caller supplies the Earth Rotation Angle (ERA) at t0.
    //
    // @param cfg           Fully populated SimulationConfig
    // @param theta_era_rad ERA at simulation start time t0  [rad]
    // @return              Eigen column vector of length 14
    // =========================================================================
    inline Eigen::Matrix<double, RigidBody::StateLayout::N, 1>
        BuildInitialStateVector(
            const RigidBody::Config& cfg,
            const double            theta_era_rad)
    {
        using namespace RigidBody;
        using namespace Coordinate;

        // ── Convert pose angles from degrees to radians ───────────────────────
        const double lat_rad = cfg.pose.lat_deg * kDegreesToRadians;
        const double lon_rad = cfg.pose.lon_deg * kDegreesToRadians;
        const double h_m = cfg.pose.alt_m;
        const double az_rad = cfg.pose.azimuth_deg * kDegreesToRadians;
        const double zen_rad = cfg.pose.zenith_deg * kDegreesToRadians;
        const double roll_rad = cfg.pose.roll_deg * kDegreesToRadians;

        // ── 1) ECI position: geodetic → ECEF → ECI ───────────────────────────
        //
        //   r_ecef = GeodeticToECEF(lat, lon, h)   [WGS-84 ellipsoid]
        //   r_eci  = ECEFToECI(r_ecef, theta_era)  [rotation about Z by θ]
        //
        const Vec3<double> r_ecef =
            GeodeticToECEF(lat_rad, lon_rad, h_m);

        const Vec3<double> r_eci =
            ECEFToECI(r_ecef, theta_era_rad);

        // ── 2) Attitude quaternion: NED launch angles → body→ECI quaternion ──
        //
        //   MakeLaunchStateECI() handles the full chain:
        //     azimuth/zenith → NED body axes → ECEF → ECI → quaternion
        //   It returns q_EB = [w, x, y, z] such that v_ECI = q_EB ⊗ v_B ⊗ q_EB⁻¹
        //
        const LaunchStateECI<double> launch =
            MakeLaunchStateECI(
                lat_rad, lon_rad, h_m,
                az_rad, zen_rad, roll_rad,
                theta_era_rad);

        // q_EB is already unit-normalised by MakeLaunchStateECI
        const Quat<double>& q_EB = launch.q_EB; // [w, x, y, z]

        // ── 3) ECI velocity: NED → ECEF → ECI ────────────────────────────────
        //
        //   The vehicle may have a non-zero initial velocity expressed in local NED.
        //   Transform chain:
        //     v_ned  → v_ecef = NEDToECEF(v_ned, lat, lon)
        //     v_ecef → v_eci  = ECEFToECI(v_ecef, theta_era)
        //
        //   Note: This is a *kinematic* transform (no Coriolis correction).
        //   If you need to account for the ECEF frame's rotation w.r.t ECI,
        //   you would add ω_E × r_ecef to v_ecef before the rotation.
        //   For a launch from rest (velocityNED == 0), both terms are zero.
        //
        const Vec3<double> v_ned{
            cfg.velocityNED.north_mps,
            cfg.velocityNED.east_mps,
            cfg.velocityNED.down_mps
        };

        const Vec3<double> v_ecef =
            NEDToECEF(v_ned, lat_rad, lon_rad);

        const Vec3<double> v_eci =
            ECEFToECI(v_ecef, theta_era_rad);

        // ── 4) Body angular velocity ──────────────────────────────────────────
        //
        //   BodyRates store roll/pitch/yaw rates
        //   in body-frame axes [rad/s].  The state stores ω_B = [ωx, ωy, ωz].
        //
        //   Convention (body frame, nose-forward, right-wing, down):
        //     ωx = roll  rate about body +x (forward)
        //     ωy = pitch rate about body +y (right wing)
        //     ωz = yaw   rate about body +z (down)
        //
        const double omega_x = cfg.bodyRates.roll_rad_s;
        const double omega_y = cfg.bodyRates.pitch_rad_s;
        const double omega_z = cfg.bodyRates.yaw_rad_s;

        // ── 5) Initial mass ───────────────────────────────────────────────────
        const double mass_kg = cfg.inertialParameters.mass_kg;

        // ── Assemble the flat state vector ────────────────────────────────────
        Eigen::Matrix<double, StateLayout::N, 1> x0;
        x0.setZero();

        // IDX_P = 0  → ECI position [m]
        x0[StateLayout::IDX_P + 0] = r_eci[0];
        x0[StateLayout::IDX_P + 1] = r_eci[1];
        x0[StateLayout::IDX_P + 2] = r_eci[2];

        // IDX_Q = 3  → quaternion body→ECI  [w, x, y, z]
        x0[StateLayout::IDX_Q + 0] = q_EB[0]; // w
        x0[StateLayout::IDX_Q + 1] = q_EB[1]; // x
        x0[StateLayout::IDX_Q + 2] = q_EB[2]; // y
        x0[StateLayout::IDX_Q + 3] = q_EB[3]; // z

        // IDX_W = 7  → body angular velocity [rad/s]
        x0[StateLayout::IDX_W + 0] = omega_x;
        x0[StateLayout::IDX_W + 1] = omega_y;
        x0[StateLayout::IDX_W + 2] = omega_z;

        // IDX_V = 10 → ECI velocity [m/s]
        x0[StateLayout::IDX_V + 0] = v_eci[0];
        x0[StateLayout::IDX_V + 1] = v_eci[1];
        x0[StateLayout::IDX_V + 2] = v_eci[2];

        // IDX_M = 13 → mass [kg]
        x0[StateLayout::IDX_M] = mass_kg;

        return x0;
    }

    // =========================================================================
    // BuildInitialStateVector  (convenience overload)
    //
    // Computes ERA from cfg.simulation.startTime using the simple linear model:
    //     θ_ERA = ω_E × t0
    //
    // Suitable when t0 is measured from an epoch at which the ECI and ECEF
    // X-axes were aligned (e.g. a J2000-like epoch with zero GMST offset).
    // For higher fidelity, use the primary overload and supply the correct ERA.
    //
    // @param cfg  Fully populated SimulationConfig
    // @return     Eigen column vector of length 14
    // =========================================================================
  /*  inline Eigen::Matrix<double, RigidBody::StateLayout::N, 1>
        BuildInitialStateVector(const SimulationConfig& cfg)
    {
        const double theta_era_rad =
            kEarthRotationRate_rad_s * cfg.simulation.startTime;

        return BuildInitialStateVector(cfg, theta_era_rad);
    }*/

} // namespace Aetherion::FlightDynamics
