// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// MakeSnapshot2.h
//
// Produces a Snapshot2 (31-column NASA-compatible snapshot) from a
// RigidBody::StateD, a GravityPolicy, and an AeroPolicy.
//
// All kinematic, geodetic, atmospheric, and air-data fields are computed
// identically to MakeSnapshot1.  The six aerodynamic fields are populated
// by evaluating the AeroPolicy at the current state.  The seven
// Aetherion-internal fields (v_eci, q_body_to_eci) are omitted since they
// have no NASA counterpart and are not needed for reference comparison.
// ------------------------------------------------------------------------------

#pragma once

#include "Snapshot2.h"
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace Aetherion::Simulation {

/// @brief Build a 31-column NASA-compatible Snapshot2.
///
/// @tparam GravityPol  Gravity policy (satisfies @c GravityPolicy).
/// @tparam AeroPol     Aero policy   (satisfies @c AeroPolicy).
///
/// @param t          Simulation time [s].
/// @param s          Current rigid-body state.
/// @param theta_gst  Earth Rotation Angle [rad].
/// @param gravity    Gravity policy instance.
/// @param aero       Aero policy instance.
/// @return           Fully populated Snapshot2 (31 columns, NASA-compatible).
template <FlightDynamics::GravityPolicy GravityPol,
          FlightDynamics::AeroPolicy    AeroPol>
[[nodiscard]]
Snapshot2 MakeSnapshot2(
    double                       t,
    const RigidBody::StateD&     s,
    double                       theta_gst,
    const GravityPol&            gravity,
    const AeroPol&               aero)
{
    namespace Coord = Aetherion::Coordinate;
    namespace Env   = Aetherion::Environment;

    using Vec3d = Eigen::Vector3d;
    using CArr3 = Coord::Vec3<double>;

    Snapshot2 snap;

    // ── 1. Time ───────────────────────────────────────────────────────────
    snap.time = t;

    // ── 2. ECEF position ─────────────────────────────────────────────────
    const Vec3d r_eci = s.g.p;
    const CArr3 r_eci_arr{ r_eci.x(), r_eci.y(), r_eci.z() };
    const CArr3 r_ecef_arr = Coord::ECIToECEF(r_eci_arr, theta_gst);
    snap.gePosition_m = Vec3d(r_ecef_arr[0], r_ecef_arr[1], r_ecef_arr[2]);

    // ── 3. Geodetic ───────────────────────────────────────────────────────
    double lat_rad{}, lon_rad{}, alt_m{};
    Coord::ECEFToGeodeticWGS84(r_ecef_arr, lat_rad, lon_rad, alt_m);
    snap.latitude_rad  = lat_rad;
    snap.longitude_rad = lon_rad;
    snap.altitudeMsl_m = alt_m;

    // ── 4. Gravity magnitude ──────────────────────────────────────────────
    constexpr double kUnitMass = 1.0;
    const auto grav_wrench = gravity(s.g, kUnitMass);
    const Vec3d a_body = grav_wrench.f.template tail<3>();
    snap.localGravity_m_s2 = (s.g.R * a_body).norm();

    // ── 5. Atmosphere ─────────────────────────────────────────────────────
    const auto atm = Env::US1976Atmosphere(alt_m);
    snap.speedOfSound_m_s    = atm.a;
    snap.airDensity_kg_m3    = atm.rho;
    snap.ambientPressure_Pa  = atm.p;
    snap.ambientTemperature_K = atm.T;

    // ── 6. NED velocity ───────────────────────────────────────────────────
    const Vec3d v_B      = s.nu_B.tail<3>();
    const Vec3d v_eci_v  = s.g.R * v_B;
    const CArr3 v_eci_arr{ v_eci_v.x(), v_eci_v.y(), v_eci_v.z() };
    const CArr3 v_ecef_rot = Coord::ECIToECEF(v_eci_arr, theta_gst);

    const Vec3d r_ecef_v(r_ecef_arr[0], r_ecef_arr[1], r_ecef_arr[2]);
    constexpr double kOmegaE = Env::WGS84::kRotationRate_rad_s;
    const Vec3d omega_E(0.0, 0.0, kOmegaE);
    const Vec3d v_ecef_v = Vec3d(v_ecef_rot[0], v_ecef_rot[1], v_ecef_rot[2])
                           - omega_E.cross(r_ecef_v);

    const CArr3 v_ecef_arr2{ v_ecef_v.x(), v_ecef_v.y(), v_ecef_v.z() };
    const CArr3 v_ned_arr = Coord::ECEFToNED(v_ecef_arr2, lat_rad, lon_rad);
    snap.feVelocity_m_s = Vec3d(v_ned_arr[0], v_ned_arr[1], v_ned_arr[2]);
    snap.altitudeRateWrtMsl_m_s = -v_ned_arr[2];

    // ── 7. Euler angles (ZYX, body → NED) ────────────────────────────────
    const Coord::Mat3<double> R_IN_arr =
        Coord::NEDToInertialRotationMatrix(lat_rad, lon_rad, theta_gst);
    Eigen::Matrix3d R_IN;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R_IN(r, c) = R_IN_arr[3*r + c];

    const Eigen::Matrix3d R_NB = R_IN.transpose() * s.g.R;
    snap.eulerAngle_rad_Pitch = std::asin(std::clamp(-R_NB(2,0), -1.0, 1.0));
    snap.eulerAngle_rad_Yaw   = std::atan2(R_NB(1,0), R_NB(0,0));
    snap.eulerAngle_rad_Roll  = std::atan2(R_NB(2,1), R_NB(2,2));

    // ── 8. Body angular rates ─────────────────────────────────────────────
    snap.bodyAngularRateWrtEi_rad_s_Roll  = s.nu_B(0);
    snap.bodyAngularRateWrtEi_rad_s_Pitch = s.nu_B(1);
    snap.bodyAngularRateWrtEi_rad_s_Yaw   = s.nu_B(2);

    // ── 9. Air data ───────────────────────────────────────────────────────
    const double tas = snap.feVelocity_m_s.norm();
    snap.trueAirspeed_m_s  = tas;
    snap.mach               = tas / atm.a;
    snap.dynamicPressure_Pa = 0.5 * atm.rho * tas * tas;

    // ── 10. Aerodynamic wrench ────────────────────────────────────────────
    // Wrench layout: [moment(0:2); force(3:5)]
    using SE3d = ODE::RKMK::Lie::SE3<double>;
    const SE3d g(s.g.R, s.g.p);
    const auto wrench = aero(g, s.nu_B, s.m, t);

    snap.aero_bodyMoment_Nm_L = wrench.f(0);
    snap.aero_bodyMoment_Nm_M = wrench.f(1);
    snap.aero_bodyMoment_Nm_N = wrench.f(2);
    snap.aero_bodyForce_N_X   = wrench.f(3);
    snap.aero_bodyForce_N_Y   = wrench.f(4);
    snap.aero_bodyForce_N_Z   = wrench.f(5);

    return snap;
}

/// @brief Convenience overload for dragless (zero-aero) scenarios.
///
/// Equivalent to calling the two-policy form with ``ZeroAeroPolicy``.
template <FlightDynamics::GravityPolicy GravityPol>
[[nodiscard]]
Snapshot2 MakeSnapshot2(
    double                       t,
    const RigidBody::StateD&     s,
    double                       theta_gst,
    const GravityPol&            gravity)
{
    return MakeSnapshot2(t, s, theta_gst, gravity,
                         FlightDynamics::ZeroAeroPolicy{});
}

} // namespace Aetherion::Simulation
