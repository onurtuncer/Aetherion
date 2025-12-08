// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

//*******************************************************************************
//
//  - Position:
//      r_eci  -> r_ecef (ECIToECEF)
//      r_ecef -> (lat, lon, h) WGS-84   (ECEFToGeodeticWGS84)
//
//  - Vectors / directions:
//      v_eci  -> v_ecef -> v_neu        (ECIToNEU)
//
//  - Orientation:
//      q_EB (body->ECI) ->
//         body axes in ECI -> ECEF -> NEU ->
//         q_NB (body->NEU)
//
// All functions are templated on Scalar and use only elementary functions,
// suitable for use with CppAD::AD<Scalar>.
//
// *******************************************************************************

#pragma once

#include <array>
#include <cmath>

#include "Aetherion/Coordinate/FrameConversions.h"

namespace Aetherion::Coordinate {

    // Reuse Vec3 / Quat from FrameConversions.h
    // template <class Scalar> using Vec3 = std::array<Scalar, 3>;
    // template <class Scalar> using Quat = std::array<Scalar, 4>;

    namespace detail {

        // Bring wrappers from FrameConversions.h into this header's scope
        using Aetherion::Coordinate::detail::Sine;
        using Aetherion::Coordinate::detail::Cosine;
        using Aetherion::Coordinate::detail::SquareRoot;
        using Aetherion::Coordinate::detail::Cross;
        using Aetherion::Coordinate::detail::Normalize;
        using Aetherion::Coordinate::detail::RotationMatrixToQuaternion;

    } // namespace detail

    // -------------------------------------------------------------------------
    // 1. ECI <-> ECEF for vectors
    // -------------------------------------------------------------------------

    /// Rotate a vector from ECI to ECEF using Earth rotation angle theta (radians).
    /// theta is the angle that rotates ECI -> ECEF.
    template <class Scalar>
    inline Vec3<Scalar> ECIToECEF(
        const Vec3<Scalar>& v_eci,
        const Scalar& theta_rad)
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

    // (ECEFToECI is already in LaunchStateECI.hpp as ECEFToECI.)

    // -------------------------------------------------------------------------
    // 2. ECEF -> WGS-84 geodetic (lat, lon, h)
    // -------------------------------------------------------------------------

    /// Convert ECEF position [m] into WGS-84 geodetic coordinates:
    ///   lat_rad [rad], lon_rad [rad], h_m [m].
    ///
    /// This uses a small fixed-point iteration (typically 4–5 steps) and is
    /// differentiable for CppAD (loop count is fixed).
    template <class Scalar>
    inline void ECEFToGeodeticWGS84(
        const Vec3<Scalar>& r_ecef,
        Scalar& lat_rad,
        Scalar& lon_rad,
        Scalar& h_m)
    {
        using detail::Sine;
        using detail::Cosine;
        using detail::SquareRoot;

        const Scalar x = r_ecef[0];
        const Scalar y = r_ecef[1];
        const Scalar z = r_ecef[2];

        // WGS-84 constants
        const Scalar a = Scalar(6378137.0);                 // semi-major axis [m]
        const Scalar f = Scalar(1.0 / 298.257223563);       // flattening
        const Scalar e2 = f * (Scalar(2) - f);               // eccentricity^2

        const Scalar one = Scalar(1);

        // Longitude is straightforward
        lon_rad = std::atan2(static_cast<double>(y), static_cast<double>(x));

        const Scalar p = SquareRoot(x * x + y * y);

        // Initial guess for latitude (geocentric-like)
        Scalar lat = std::atan2(static_cast<double>(z),
            static_cast<double>(p * (one - e2)));

        Scalar h = Scalar(0);

        // Fixed number of iterations for refinement (AD-friendly)
        for (int i = 0; i < 5; ++i) {
            const Scalar sLat = Sine(lat);
            const Scalar cLat = Cosine(lat);

            const Scalar N = a / SquareRoot(one - e2 * sLat * sLat);
            h = p / cLat - N;
            lat = std::atan2(static_cast<double>(z),
                static_cast<double>(p * (one - e2 * N / (N + h))));
        }

        lat_rad = lat;
        h_m = h;
    }

    // -------------------------------------------------------------------------
    // 3. ECEF -> NEU and ECI -> NEU (for vectors/directions)
    // -------------------------------------------------------------------------

    /// Transform a vector from ECEF to local NEU at (lat, lon).
    /// Inverse of NEUToECEF: components are dot products with N,E,U basis.
    template <class Scalar>
    inline Vec3<Scalar> ECEFToNEU(
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

        // Same basis vectors used in NEUToECEF (N,E,U in ECEF coords)
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
        const Vec3<Scalar> U{
             cLat * cLon,
             cLat * sLon,
             sLat
        };

        const Scalar vX = v_ecef[0];
        const Scalar vY = v_ecef[1];
        const Scalar vZ = v_ecef[2];

        // Components in NEU = dot(v_ecef, basis)
        const Scalar vN = vX * N[0] + vY * N[1] + vZ * N[2];
        const Scalar vE = vX * E[0] + vY * E[1] + vZ * E[2];
        const Scalar vU = vX * U[0] + vY * U[1] + vZ * U[2];

        return Vec3<Scalar>{ vN, vE, vU };
    }

    /// Transform a vector from ECI to NEU at (lat, lon), given Earth rotation
    /// angle theta (radians).
    template <class Scalar>
    inline Vec3<Scalar> ECIToNEU(
        const Vec3<Scalar>& v_eci,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& earth_rotation_angle_rad)
    {
        const Vec3<Scalar> v_ecef =
            ECIToECEF(v_eci, earth_rotation_angle_rad);

        return ECEFToNEU(v_ecef, lat_rad, lon_rad);
    }

    // -------------------------------------------------------------------------
    // 4. Orientation: from q_EB (body->ECI) to local NEU
    // -------------------------------------------------------------------------

    /// Convert quaternion q_EB (body->ECI) to body axes expressed in ECI:
    /// columns of rotation matrix R_WB (W=ECI, B=body).
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

        // Standard quaternion->matrix (columns are body axes in ECI)
        // R_WB = [x_b_eci | y_b_eci | z_b_eci]
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

    /// Body axes in NEU given q_EB (body->ECI), site (lat, lon) and Earth angle.
    template <class Scalar>
    inline void BodyAxesNEUFromECI(
        const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& earth_rotation_angle_rad,
        Vec3<Scalar>& x_b_neu,
        Vec3<Scalar>& y_b_neu,
        Vec3<Scalar>& z_b_neu)
    {
        // 1) Body axes in ECI
        Vec3<Scalar> x_b_eci, y_b_eci, z_b_eci;
        QuaternionToAxesECI(q_EB, x_b_eci, y_b_eci, z_b_eci);

        // 2) Transform each axis: ECI -> ECEF -> NEU
        const Vec3<Scalar> x_b_ecef = ECIToECEF(x_b_eci, earth_rotation_angle_rad);
        const Vec3<Scalar> y_b_ecef = ECIToECEF(y_b_eci, earth_rotation_angle_rad);
        const Vec3<Scalar> z_b_ecef = ECIToECEF(z_b_eci, earth_rotation_angle_rad);

        x_b_neu = ECEFToNEU(x_b_ecef, lat_rad, lon_rad);
        y_b_neu = ECEFToNEU(y_b_ecef, lat_rad, lon_rad);
        z_b_neu = ECEFToNEU(z_b_ecef, lat_rad, lon_rad);

        // Optionally normalize to guard against numerical drift
        x_b_neu = detail::Normalize(x_b_neu);
        y_b_neu = detail::Normalize(y_b_neu);
        z_b_neu = detail::Normalize(z_b_neu);
    }

    /// Quaternion mapping body->NEU, computed from q_EB and site (lat, lon, theta).
    /// Returns q_NB such that v_NEU = q_NB * v_B * q_NB^-1.
    template <class Scalar>
    inline Quat<Scalar> BodyQuaternionInNEU(
        const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& earth_rotation_angle_rad)
    {
        Vec3<Scalar> x_b_neu, y_b_neu, z_b_neu;
        BodyAxesNEUFromECI(q_EB, lat_rad, lon_rad,
            earth_rotation_angle_rad,
            x_b_neu, y_b_neu, z_b_neu);

        // Reuse the same RotationMatrixToQuaternion used for ECI frame,
        // now feeding body axes expressed in NEU.
        return detail::RotationMatrixToQuaternion(
            x_b_neu, y_b_neu, z_b_neu);
    }

    // -------------------------------------------------------------------------
    // 5. Convenience: full inverse of LaunchStateECI (position + orientation)
    // -------------------------------------------------------------------------

    /// Full inverse mapping of LaunchStateECI:
    /// Given state in ECI and Earth angle, recover WGS-84 site and body attitude
    /// in NEU coordinates.
    template <class Scalar>
    struct LocalStateNEU {
        Scalar lat_rad;      // geodetic latitude
        Scalar lon_rad;      // geodetic longitude
        Scalar h_m;          // height above ellipsoid
        Vec3<Scalar> r_ecef; // position in ECEF [m]
        Vec3<Scalar> r_eci;  // original position in ECI [m]
        Vec3<Scalar> dir_neu;// launch direction in NEU (body +x in NEU)
        Quat<Scalar> q_NB;   // body->NEU quaternion
    };

    template <class Scalar>
    inline LocalStateNEU<Scalar> InvertLaunchStateToLocalNEU(
        const LaunchStateECI<Scalar>& state_eci,
        const Scalar& earth_rotation_angle_rad)
    {
        LocalStateNEU<Scalar> out;

        // Position: ECI -> ECEF -> WGS-84
        out.r_eci = state_eci.r0;
        out.r_ecef = ECIToECEF(state_eci.r0, earth_rotation_angle_rad);

        ECEFToGeodeticWGS84(out.r_ecef,
            out.lat_rad,
            out.lon_rad,
            out.h_m);

        // Direction: dir0_eci -> NEU
        out.dir_neu = ECIToNEU(state_eci.dir0_eci,
            out.lat_rad,
            out.lon_rad,
            earth_rotation_angle_rad);

        // Orientation: q_EB -> q_NB
        out.q_NB = BodyQuaternionInNEU(
            state_eci.q_EB,
            out.lat_rad,
            out.lon_rad,
            earth_rotation_angle_rad);

        return out;
    }

} // namespace Aetherion::Coordinate
