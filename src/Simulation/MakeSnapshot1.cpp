// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// MakeSnapshot1.cpp
//
// Refactored step 7 (Euler angles):  the old code manually recomputed sin/cos
// of lat/lon and theta_gst to build R_EN and R_IE from scratch.  Those same
// rotations are already implemented (and tested) in InertialToLocal.h as
// ECIToECEF(), ECEFToNED(), and NEDToInertialRotationMatrix().
//
// The fix: call NEDToInertialRotationMatrix() — which returns the 3×3
// rotation R_IN (NED→ECI, row-major std::array<Scalar,9>) — and convert it
// to Eigen for the R_NB = R_IN^T · R_IB product.  Identical arithmetic,
// zero duplication.
// ------------------------------------------------------------------------------

#include <Aetherion/Simulation/MakeSnapshot1.h>

#include <Aetherion/Coordinate/InertialToLocal.h>   // ECIToECEF, ECEFToGeodeticWGS84,
                                                     // ECEFToNED, NEDToInertialRotationMatrix
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/Gravity.h>

#include <Eigen/Dense>

#include <algorithm>   // std::clamp
#include <cmath>

namespace Aetherion::Simulation {

    Snapshot1 MakeSnapshot1(
        double                   t,
        const RigidBody::StateD& s,
        double                   theta_gst)
    {
        namespace Coord = Aetherion::Coordinate;
        namespace Env = Aetherion::Environment;

        using Vec3d = Eigen::Vector3d;
        using CArr3 = Coord::Vec3<double>;   // std::array<double,3>

        Snapshot1 snap;

        // ── 1. Time ──────────────────────────────────────────────────────────
        snap.time = t;

        // ── 2. ECI position ──────────────────────────────────────────────────
        const Vec3d r_eci = s.g.p;
        snap.gePosition_m = r_eci;

        // ── 3. Geodetic position (WGS-84) ────────────────────────────────────
        const CArr3 r_eci_arr = { r_eci.x(), r_eci.y(), r_eci.z() };
        const CArr3 r_ecef_arr = Coord::ECIToECEF(r_eci_arr, theta_gst);

        double lat_rad{}, lon_rad{}, alt_m{};
        Coord::ECEFToGeodeticWGS84(r_ecef_arr, lat_rad, lon_rad, alt_m);

        snap.latitude_rad = lat_rad;
        snap.longitude_rad = lon_rad;
        snap.altitudeMsl_m = alt_m;

        // ── 4. Gravity magnitude ─────────────────────────────────────────────
        const auto g_eci_arr = Env::CentralGravity<double>(r_eci_arr);
        snap.localGravity_m_s2 = std::sqrt(
            g_eci_arr[0] * g_eci_arr[0] +
            g_eci_arr[1] * g_eci_arr[1] +
            g_eci_arr[2] * g_eci_arr[2]);

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

        const CArr3 v_eci_arr = { v_eci_vec.x(), v_eci_vec.y(), v_eci_vec.z() };
        const CArr3 v_ecef_arr = Coord::ECIToECEF(v_eci_arr, theta_gst);
        const CArr3 v_ned_arr = Coord::ECEFToNED(v_ecef_arr, lat_rad, lon_rad);

        snap.feVelocity_m_s = Vec3d(v_ned_arr[0], v_ned_arr[1], v_ned_arr[2]);
        snap.altitudeRateWrtMsl_m_s = -v_ned_arr[2];   // dh/dt = −v_down

        // ── 7. ZYX Euler angles (body → NED) ─────────────────────────────────
        //
        // R_NB = R_IN^T · R_IB  (body → NED)
        //
        // Previously this block manually recomputed sin/cos of lat/lon and
        // theta_gst to assemble R_EN and R_IE as 3×3 Eigen matrices.
        // That arithmetic already exists (and is tested) in
        // NEDToInertialRotationMatrix(), which returns R_IN as a row-major
        // std::array<double,9>.  We just convert it to Eigen here.
        // ─────────────────────────────────────────────────────────────────────
        const Coord::Mat3<double> R_IN_arr =
            Coord::NEDToInertialRotationMatrix(lat_rad, lon_rad, theta_gst);

        // Map row-major std::array → Eigen 3×3
        // R_IN_arr layout: R_IN_arr[3*row + col]
        Eigen::Matrix3d R_IN;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R_IN(r, c) = R_IN_arr[3 * r + c];

        // R_NB: columns of R_IN^T are the NED basis in ECI;
        // R_IN^T · R_IB maps body axes into NED.
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

        // ── 10. Aerodynamics: populated by caller for non-dragless cases ──────

        return snap;
    }

} // namespace Aetherion::Simulation