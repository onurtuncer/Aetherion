// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Simulation/MakeSnapshot1.h>

#include <Aetherion/Coordinate/InertialToLocal.h>   // ECIToECEF, ECEFToGeodeticWGS84, ECEFToNED
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/Gravity.h>

#include <Eigen/Dense>

#include <cmath>
#include <algorithm>  // std::clamp

namespace Aetherion::Simulation {

    Snapshot1 MakeSnapshot1(
        double                       t,
        const RigidBody::StateD& s,
        double                       theta_gst)
    {
        namespace Coord = Aetherion::Coordinate;
        namespace Env = Aetherion::Environment;

        using Vec3d = Eigen::Vector3d;

        Snapshot1 snap;

        // ── 1. Time ──────────────────────────────────────────────────────────────
        snap.time = t;

        // ── 2. ECI position ──────────────────────────────────────────────────────
        const Vec3d r_eci = s.g.p;
        snap.gePosition_m = r_eci;

        // ── 3. Geodetic position (WGS-84) ────────────────────────────────────────
        const Coord::Vec3<double> r_eci_arr = { r_eci.x(), r_eci.y(), r_eci.z() };
        const Coord::Vec3<double> r_ecef_arr = Coord::ECIToECEF(r_eci_arr, theta_gst);

        double lat_rad{}, lon_rad{}, alt_m{};
        Coord::ECEFToGeodeticWGS84(r_ecef_arr, lat_rad, lon_rad, alt_m);

        snap.latitude_rad = lat_rad;
        snap.longitude_rad = lon_rad;
        snap.altitudeMsl_m = alt_m;

        // ── 4. Gravity magnitude ─────────────────────────────────────────────────
        const auto g_eci_arr = Env::CentralGravity<double>(r_eci_arr);
        snap.localGravity_m_s2 = std::sqrt(
            g_eci_arr[0] * g_eci_arr[0] +
            g_eci_arr[1] * g_eci_arr[1] +
            g_eci_arr[2] * g_eci_arr[2]);

        // ── 5. Atmosphere (US 1976) ───────────────────────────────────────────────
        const auto atm = Env::US1976Atmosphere(alt_m);
        snap.ambientTemperature_K = atm.T;
        snap.ambientPressure_Pa = atm.p;
        snap.airDensity_kg_m3 = atm.rho;
        snap.speedOfSound_m_s = atm.a;

        // ── 6. NED velocity ───────────────────────────────────────────────────────
        // nu_B layout: [ω_B(0:2); v_B(3:5)] – both in body frame
        const Vec3d v_B = s.nu_B.tail<3>();
        const Vec3d v_eci_vec = s.g.R * v_B;
        snap.v_eci = v_eci_vec;

        const Coord::Vec3<double> v_eci_arr = { v_eci_vec.x(), v_eci_vec.y(), v_eci_vec.z() };
        const Coord::Vec3<double> v_ecef_arr = Coord::ECIToECEF(v_eci_arr, theta_gst);
        const Coord::Vec3<double> v_ned_arr = Coord::ECEFToNED(v_ecef_arr, lat_rad, lon_rad);

        snap.feVelocity_m_s = Vec3d(v_ned_arr[0], v_ned_arr[1], v_ned_arr[2]);
        snap.altitudeRateWrtMsl_m_s = -v_ned_arr[2];   // dh/dt = −v_down

        // ── 7. ZYX Euler angles (body → NED) ─────────────────────────────────────
        // R_NB = R_IN^T * R_IB  (body → NED),  then standard 3-2-1 decomposition.
        const double sin_lat = std::sin(lat_rad), cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad), cos_lon = std::cos(lon_rad);

        // R_EN: NED → ECEF (column vectors are N, E, D axes in ECEF)
        Eigen::Matrix3d R_EN;
        R_EN << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,
            -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,
            cos_lat, 0.0, -sin_lat;

        // R_IE: ECEF → ECI = R3(−theta_gst)
        const double cg = std::cos(-theta_gst), sg = std::sin(-theta_gst);
        Eigen::Matrix3d R_IE;
        R_IE << cg, -sg, 0.0,
            sg, cg, 0.0,
            0.0, 0.0, 1.0;

        // R_IN = R_IE * R_EN  (NED → ECI);  R_IN^T = NED ← ECI
        const Eigen::Matrix3d R_NB = (R_IE * R_EN).transpose() * s.g.R;

        snap.eulerAngle_rad_Pitch = std::asin(std::clamp(-R_NB(2, 0), -1.0, 1.0));
        snap.eulerAngle_rad_Yaw = std::atan2(R_NB(1, 0), R_NB(0, 0));
        snap.eulerAngle_rad_Roll = std::atan2(R_NB(2, 1), R_NB(2, 2));

        snap.q_body_to_eci = Eigen::Quaterniond(s.g.R).normalized();

        // ── 8. Body angular rates wrt ECI ────────────────────────────────────────
        snap.bodyAngularRateWrtEi_rad_s_Roll = s.nu_B(0);
        snap.bodyAngularRateWrtEi_rad_s_Pitch = s.nu_B(1);
        snap.bodyAngularRateWrtEi_rad_s_Yaw = s.nu_B(2);

        // ── 9. Air-data ───────────────────────────────────────────────────────────
        const double tas = snap.feVelocity_m_s.norm();
        snap.trueAirspeed_m_s = tas;
        snap.mach = tas / atm.a;
        snap.dynamicPressure_Pa = 0.5 * atm.rho * tas * tas;

        // ── 10. Aerodynamics: default-initialised to 0.0 in Snapshot1 ────────────

        return snap;
    }

} // namespace Aetherion::Simulation