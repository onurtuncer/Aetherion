// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//*******************************************************************************
//
//  - Position:
//      r_eci  -> r_ecef (ECIToECEF)
//      r_ecef -> (lat, lon, h) WGS-84   (ECEFToGeodeticWGS84)
//
//  - Vectors / directions (NED, right-handed):
//      v_eci  -> v_ecef -> v_ned        (ECIToNED)
//
//  - Orientation (NED, right-handed):
//      q_EB (body->ECI) ->
//         body axes in ECI -> ECEF -> NED ->
//         q_DB (body->NED)
//
// All functions are templated on Scalar and use only elementary functions,
// suitable for use with CppAD::AD<Scalar>.
//
// *******************************************************************************

#pragma once

#include "Math.h"
#include <Aetherion/Coordinate/LocalToInertial.h>

namespace Aetherion::Coordinate {

    // -------------------------------------------------------------------------
    // 1. ECI -> ECEF (vectors)
    // -------------------------------------------------------------------------

    /// Rotate a vector from ECI to ECEF using Earth rotation angle theta (radians).
    /// theta is the angle that rotates ECI -> ECEF.
    template <class Scalar>
    inline Vec3<Scalar> ECIToECEF(const Vec3<Scalar>& v_eci, const Scalar& theta_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar c = Cosine(theta_rad);
        const Scalar s = Sine(theta_rad);

        return Vec3<Scalar>{
            c* v_eci[0] + s * v_eci[1],
                -s * v_eci[0] + c * v_eci[1],
                v_eci[2]
        };
    }

    // (ECEFToECI is already provided in LocalToInertial.h)

    // -------------------------------------------------------------------------
    // 2. ECEF -> WGS-84 geodetic (lat, lon, h)
    // -------------------------------------------------------------------------

    template <class Scalar>
    void ECEFToGeodeticWGS84(
        const Vec3<Scalar>& r_ecef,
        Scalar& lat_rad,
        Scalar& lon_rad,
        Scalar& h_m);

    // -------------------------------------------------------------------------
    // 3. ECEF -> NED and ECI -> NED (vectors/directions)
    // -------------------------------------------------------------------------

    /// Transform a vector from ECEF to local NED at (lat, lon).
    /// NED is right-handed: N × E = D.
    template <class Scalar>
    inline Vec3<Scalar> ECEFToNED(
        const Vec3<Scalar>& v_ecef,
        const Scalar& lat_rad,
        const Scalar& lon_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        // N, E, D basis in ECEF coordinates
        const Vec3<Scalar> N{
            -sLat * cLon,
            -sLat * sLon,
             cLat
        };
        const Vec3<Scalar> E{
            -sLon,
             cLon,
             Scalar(0)
        };
        // D = -U (Down)
        const Vec3<Scalar> D{
            -cLat * cLon,
            -cLat * sLon,
            -sLat
        };

        const Scalar vX = v_ecef[0];
        const Scalar vY = v_ecef[1];
        const Scalar vZ = v_ecef[2];

        const Scalar vN = vX * N[0] + vY * N[1] + vZ * N[2];
        const Scalar vE = vX * E[0] + vY * E[1] + vZ * E[2];
        const Scalar vD = vX * D[0] + vY * D[1] + vZ * D[2];

        return Vec3<Scalar>{ vN, vE, vD };
    }

    /// Transform a vector from ECI to NED at (lat, lon), given Earth rotation angle.
    template <class Scalar>
    inline Vec3<Scalar> ECIToNED(
        const Vec3<Scalar>& v_eci,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& earth_rotation_angle_rad)
    {
        const Vec3<Scalar> v_ecef = ECIToECEF(v_eci, earth_rotation_angle_rad);
        return ECEFToNED(v_ecef, lat_rad, lon_rad);
    }

    // -------------------------------------------------------------------------
    // 4. Orientation: q_EB (body->ECI) to local NED
    // -------------------------------------------------------------------------

    /// Convert quaternion q_EB (body->ECI) to body axes expressed in ECI:
    /// columns of rotation matrix R_EB (ECI <- body).
    template <class Scalar>
    inline void QuaternionToAxesECI(
        const Quat<Scalar>& q_EB,
        Vec3<Scalar>& x_b_eci,
        Vec3<Scalar>& y_b_eci,
        Vec3<Scalar>& z_b_eci)
    {
        const Scalar w = q_EB[0];
        const Scalar x = q_EB[1];
        const Scalar y = q_EB[2];
        const Scalar z = q_EB[3];

        const Scalar ww = w * w;
        const Scalar xx = x * x;
        const Scalar yy = y * y;
        const Scalar zz = z * z;

        const Scalar wx = w * x;
        const Scalar wy = w * y;
        const Scalar wz = w * z;
        const Scalar xy = x * y;
        const Scalar xz = x * z;
        const Scalar yz = y * z;

        x_b_eci = Vec3<Scalar>{
            ww + xx - yy - zz,
            Scalar(2) * (xy + wz),
            Scalar(2) * (xz - wy)
        };
        y_b_eci = Vec3<Scalar>{
            Scalar(2) * (xy - wz),
            ww - xx + yy - zz,
            Scalar(2) * (yz + wx)
        };
        z_b_eci = Vec3<Scalar>{
            Scalar(2) * (xz + wy),
            Scalar(2) * (yz - wx),
            ww - xx - yy + zz
        };
    }

    /// Body axes in NED given q_EB (body->ECI), site (lat, lon) and Earth angle.
    template <class Scalar>
    inline void BodyAxesNEDFromECI(
        const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& earth_rotation_angle_rad,
        Vec3<Scalar>& x_b_ned,
        Vec3<Scalar>& y_b_ned,
        Vec3<Scalar>& z_b_ned)
    {
        // 1) Body axes in ECI
        Vec3<Scalar> x_b_eci, y_b_eci, z_b_eci;
        QuaternionToAxesECI(q_EB, x_b_eci, y_b_eci, z_b_eci);

        // 2) ECI -> ECEF
        const Vec3<Scalar> x_b_ecef = ECIToECEF(x_b_eci, earth_rotation_angle_rad);
        const Vec3<Scalar> y_b_ecef = ECIToECEF(y_b_eci, earth_rotation_angle_rad);
        const Vec3<Scalar> z_b_ecef = ECIToECEF(z_b_eci, earth_rotation_angle_rad);

        // 3) ECEF -> NED
        x_b_ned = ECEFToNED(x_b_ecef, lat_rad, lon_rad);
        y_b_ned = ECEFToNED(y_b_ecef, lat_rad, lon_rad);
        z_b_ned = ECEFToNED(z_b_ecef, lat_rad, lon_rad);

        x_b_ned = detail::Normalize(x_b_ned);
        y_b_ned = detail::Normalize(y_b_ned);
        z_b_ned = detail::Normalize(z_b_ned);
    }

    /// Quaternion mapping body->NED, computed from q_EB and site (lat, lon, theta).
    /// Returns q_DB such that v_NED = R(q_DB) v_B.
    template <class Scalar>
    inline Quat<Scalar> BodyQuaternionInNED(
        const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& earth_rotation_angle_rad)
    {
        Vec3<Scalar> x_b_ned, y_b_ned, z_b_ned;
        BodyAxesNEDFromECI(q_EB, lat_rad, lon_rad, earth_rotation_angle_rad,
            x_b_ned, y_b_ned, z_b_ned);

        return detail::RotationMatrixToQuaternion(x_b_ned, y_b_ned, z_b_ned);
    }

    // -------------------------------------------------------------------------
    // 5. Convenience: inverse of LaunchStateECI (position + orientation)
    // -------------------------------------------------------------------------

    template <class Scalar>
    struct LocalStateNED {
        Scalar lat_rad{};
        Scalar lon_rad{};
        Scalar h_m{};
        Vec3<Scalar> dir_ned{}; // body +x in NED
        Quat<Scalar> q_DB{};    // body->NED
    };

    template <class Scalar>
    inline LocalStateNED<Scalar> InvertLaunchStateToLocalNED(
        const LaunchStateECI<Scalar>& state,
        const Scalar& earth_rotation_angle_rad)
    {
        using detail::Sine;
        using detail::Cosine;
        using detail::Dot;
        using detail::Normalize;

        // 1) ECI position -> ECEF -> WGS-84 (lat, lon, h)
        const Vec3<Scalar> r_ecef = ECIToECEF(state.r0, earth_rotation_angle_rad);

        Scalar lat_rad{}, lon_rad{}, h_m{};
        ECEFToGeodeticWGS84(r_ecef, lat_rad, lon_rad, h_m);

        // 2) Build N, E, D basis in ECEF at (lat, lon)
        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        const Vec3<Scalar> N_ecef{
            -sLat * cLon,
            -sLat * sLon,
             cLat
        };
        const Vec3<Scalar> E_ecef{
            -sLon,
             cLon,
             Scalar(0)
        };
        const Vec3<Scalar> D_ecef{
            -cLat * cLon,
            -cLat * sLon,
            -sLat
        };

        // 3) Basis in ECI
        const Vec3<Scalar> N_eci = ECEFToECI(N_ecef, earth_rotation_angle_rad);
        const Vec3<Scalar> E_eci = ECEFToECI(E_ecef, earth_rotation_angle_rad);
        const Vec3<Scalar> D_eci = ECEFToECI(D_ecef, earth_rotation_angle_rad);

        // 4) Direction: body +x in ECI -> components in NED
        Vec3<Scalar> dir_ned{
            Dot(N_eci, state.dir0_eci),
            Dot(E_eci, state.dir0_eci),
            Dot(D_eci, state.dir0_eci)
        };
        dir_ned = Normalize(dir_ned);

        // 5) Orientation: body axes in ECI -> express in NED -> quaternion body->NED
        Vec3<Scalar> x_b_eci, y_b_eci, z_b_eci;
        QuaternionToAxesECI(state.q_EB, x_b_eci, y_b_eci, z_b_eci);

        const Vec3<Scalar> x_b_ned{
            Dot(N_eci, x_b_eci),
            Dot(E_eci, x_b_eci),
            Dot(D_eci, x_b_eci)
        };
        const Vec3<Scalar> y_b_ned{
            Dot(N_eci, y_b_eci),
            Dot(E_eci, y_b_eci),
            Dot(D_eci, y_b_eci)
        };
        const Vec3<Scalar> z_b_ned{
            Dot(N_eci, z_b_eci),
            Dot(E_eci, z_b_eci),
            Dot(D_eci, z_b_eci)
        };

        const Quat<Scalar> q_DB =
            detail::RotationMatrixToQuaternion(x_b_ned, y_b_ned, z_b_ned);

        LocalStateNED<Scalar> out;
        out.lat_rad = lat_rad;
        out.lon_rad = lon_rad;
        out.h_m = h_m;
        out.dir_ned = dir_ned;
        out.q_DB = q_DB;
        return out;
    }

} // namespace Aetherion::Coordinate
