// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// MakeSnapshot1.h
//
// Converts a RigidBody::StateD (SE(3) × R^7) together with the simulation
// time, the Greenwich Sidereal Angle, and a gravity policy instance into the
// flat Snapshot1 struct that matches the NASA TM-2015-218675 Atmos_01 CSV.
//
// The function is templated on GravityPolicy so that the gravity magnitude
// reported in localGravity_m_s2 is always consistent with whichever model
// the integrator is using (CentralGravityPolicy, J2GravityPolicy, etc.).
// Passing the policy instance by const-ref also carries any non-default
// constants (custom mu, Re, J2) the integrator was constructed with.
//
// The companion MakeSnapshot1.cpp is no longer needed; delete it.
// ------------------------------------------------------------------------------

#pragma once

#include "Snapshot1.h"
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/Coordinate/InertialToLocal.h>   // ECIToECEF, ECEFToGeodeticWGS84,
                                                     // ECEFToNED, NEDToInertialRotationMatrix
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <algorithm>   // std::clamp
#include <cmath>

namespace Aetherion::Simulation {

    // ─────────────────────────────────────────────────────────────────────────
    // MakeSnapshot1<GravityPolicy>
    //
    // Parameters
    //   t         – simulation time                                      [s]
    //   s         – rigid-body state  (s.g = SE3, s.nu_B = [ω_B; v_B])
    //   theta_gst – Earth rotation angle (ECI → ECEF about +Z)         [rad]
    //   gravity   – gravity policy instance used by the integrator;
    //               its operator() is called here to ensure localGravity_m_s2
    //               is computed with the same model and constants.
    // ─────────────────────────────────────────────────────────────────────────
    template <FlightDynamics::GravityPolicy GravityPol>
    [[nodiscard]]
    Snapshot1 MakeSnapshot1(
        double                       t,
        const RigidBody::StateD& s,
        double                       theta_gst,
        const GravityPol& gravity)
    {
        namespace Coord = Aetherion::Coordinate;
        namespace Env = Aetherion::Environment;

        using Vec3d = Eigen::Vector3d;
        using CArr3 = Coord::Vec3<double>;   // std::array<double,3>

        Snapshot1 snap;

        // ── 1. Time ──────────────────────────────────────────────────────────
        snap.time = t;

        // ── 2. ECI position (retained for internal use) ──────────────────────
        const Vec3d r_eci = s.g.p;

        // ── 3. Geodetic position (WGS-84) ────────────────────────────────────
        const CArr3 r_eci_arr = { r_eci.x(), r_eci.y(), r_eci.z() };
        const CArr3 r_ecef_arr = Coord::ECIToECEF(r_eci_arr, theta_gst);

        // Report the Earth-fixed (ECEF) position so gePosition_m matches the
        // NASA TM-2015-218675 convention (starts near [6387281,0,0] at t=0 and
        // stays Earth-relative as the body falls).  ECI Y would diverge by
        // ~14 km over 30 s purely due to Earth's rotation.
        snap.gePosition_m = Vec3d(r_ecef_arr[0], r_ecef_arr[1], r_ecef_arr[2]);

        double lat_rad{}, lon_rad{}, alt_m{};
        Coord::ECEFToGeodeticWGS84(r_ecef_arr, lat_rad, lon_rad, alt_m);

        snap.latitude_rad = lat_rad;
        snap.longitude_rad = lon_rad;
        snap.altitudeMsl_m = alt_m;

        // ── 4. Gravity magnitude ─────────────────────────────────────────────
        //
        // Delegate to the same policy instance the integrator holds so that
        // the model (Central / J2 / future higher-order) and all constants
        // (mu, Re, J2, ...) are guaranteed identical.
        //
        // GravityPolicy::operator()(SE3, mass) returns a Wrench whose force
        // tail is expressed in the body frame.  We only need the ECI-frame
        // acceleration magnitude, so we extract the wrench force, rotate it
        // back to ECI (R * F_body), and divide by mass=1 (unit-mass trick).
        //
        // Using mass = 1 kg to extract the acceleration vector:
        //   F_body = gravity(g, 1).f.tail<3>()   →   a_body [m/s²]
        //   a_eci  = R * a_body
        //   |g|    = a_eci.norm()
        //
        constexpr double kUnitMass = 1.0;
        const auto wrench = gravity(s.g, kUnitMass);
        const Vec3d a_body = wrench.f.template tail<3>();
        const Vec3d a_eci = s.g.R * a_body;
        snap.localGravity_m_s2 = a_eci.norm();

        // ── 5. Atmosphere (US 1976) ───────────────────────────────────────────
        const auto atm = Env::US1976Atmosphere(alt_m);
        snap.ambientTemperature_K = atm.T;
        snap.ambientPressure_Pa = atm.p;
        snap.airDensity_kg_m3 = atm.rho;
        snap.speedOfSound_m_s = atm.a;

        // ── 6. NED velocity ───────────────────────────────────────────────────
        const Vec3d v_B = s.nu_B.tail<3>();
        const Vec3d v_eci_vec = s.g.R * v_B;
        snap.v_eci = v_eci_vec;

        // Convert ECI velocity to Earth-relative ECEF velocity:
        //   v_ECEF = R_EI * v_ECI - omega_E x r_ECEF
        // The rotation-only step ECIToECEF is correct for positions but omits
        // the omega x r Coriolis term for velocities.
        const CArr3 v_eci_arr = { v_eci_vec.x(), v_eci_vec.y(), v_eci_vec.z() };
        const CArr3 v_ecef_rot_arr = Coord::ECIToECEF(v_eci_arr, theta_gst);

        const Vec3d r_ecef_vec(r_ecef_arr[0], r_ecef_arr[1], r_ecef_arr[2]);
        constexpr double kOmegaE = Aetherion::Environment::WGS84::kRotationRate_rad_s;
        const Vec3d omega_E(0.0, 0.0, kOmegaE);
        const Vec3d v_ecef_vec = Vec3d(v_ecef_rot_arr[0], v_ecef_rot_arr[1], v_ecef_rot_arr[2])
                                 - omega_E.cross(r_ecef_vec);

        const CArr3 v_ecef_arr = { v_ecef_vec.x(), v_ecef_vec.y(), v_ecef_vec.z() };
        const CArr3 v_ned_arr = Coord::ECEFToNED(v_ecef_arr, lat_rad, lon_rad);

        snap.feVelocity_m_s = Vec3d(v_ned_arr[0], v_ned_arr[1], v_ned_arr[2]);
        snap.altitudeRateWrtMsl_m_s = -v_ned_arr[2];   // dh/dt = −v_down

        // ── 7. ZYX Euler angles (body → NED) ─────────────────────────────────
        //
        // R_NB = R_IN^T · R_IB  (body → NED)
        //
        const Coord::Mat3<double> R_IN_arr =
            Coord::NEDToInertialRotationMatrix(lat_rad, lon_rad, theta_gst);

        Eigen::Matrix3d R_IN;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R_IN(r, c) = R_IN_arr[3 * r + c];

        const Eigen::Matrix3d R_NB = R_IN.transpose() * s.g.R;

        snap.eulerAngle_rad_Pitch = std::asin(std::clamp(-R_NB(2, 0), -1.0, 1.0));
        snap.eulerAngle_rad_Yaw = std::atan2(R_NB(1, 0), R_NB(0, 0));
        snap.eulerAngle_rad_Roll = std::atan2(R_NB(2, 1), R_NB(2, 2));

        snap.q_body_to_eci = Eigen::Quaterniond(s.g.R).normalized();

        // ── 8. Body angular rates wrt ECI ────────────────────────────────────
        snap.bodyAngularRateWrtEi_rad_s_Roll = s.nu_B(0);
        snap.bodyAngularRateWrtEi_rad_s_Pitch = s.nu_B(1);
        snap.bodyAngularRateWrtEi_rad_s_Yaw = s.nu_B(2);

        // ── 9. Air-data ───────────────────────────────────────────────────────
        const double tas = snap.feVelocity_m_s.norm();
        snap.trueAirspeed_m_s = tas;
        snap.mach = tas / atm.a;
        snap.dynamicPressure_Pa = 0.5 * atm.rho * tas * tas;

        // ── 10. Aerodynamics: zero — caller uses the two-policy overload
        //        MakeSnapshot1(t, s, theta_gst, gravity, aero) to populate. ──

        return snap;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MakeSnapshot1<GravityPolicy, AeroPolicy>  (two-policy overload)
    //
    // Extends the single-policy form by evaluating the AeroPolicy at the
    // current state and filling in the six aerodynamic fields:
    //   aero_bodyForce_N_X/Y/Z     — wrench.f(3:5)  [N]
    //   aero_bodyMoment_Nm_L/M/N   — wrench.f(0:2)  [N·m]
    //
    // All other fields are identical to the single-policy form.
    // ─────────────────────────────────────────────────────────────────────────
    template <FlightDynamics::GravityPolicy GravityPol,
              FlightDynamics::AeroPolicy    AeroPol>
    [[nodiscard]]
    Snapshot1 MakeSnapshot1(
        double                       t,
        const RigidBody::StateD&     s,
        double                       theta_gst,
        const GravityPol&            gravity,
        const AeroPol&               aero)
    {
        // Delegate all kinematic/atmospheric fields to the single-policy form.
        Snapshot1 snap = MakeSnapshot1(t, s, theta_gst, gravity);

        // Evaluate aero wrench at the current state.
        // Wrench layout (Featherstone): [moment(0:2); force(3:5)]
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

} // namespace Aetherion::Simulation

