// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Coordinate/Math.h>
#include <Aetherion/Environment/WGS84.h>   // single source of WGS-84 constants

namespace Aetherion::Coordinate {

    /// @brief Convert WGS-84 geodetic coordinates to ECEF Cartesian position.
    ///
    /// Uses the standard prime-vertical radius formula:
    /// @f[ N = a / \sqrt{1 - e^2 \sin^2\phi} @f]
    ///
    /// @param lat_rad  Geodetic latitude [rad].
    /// @param lon_rad  Geodetic longitude [rad].
    /// @param h_m      Ellipsoidal height above WGS-84 ellipsoid [m].
    /// @return         ECEF position vector @f$[X,\,Y,\,Z]@f$ [m].
    template <class Scalar>
    inline Vec3<Scalar> GeodeticToECEF(
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& h_m)
    {
        using detail::Sine;
        using detail::Cosine;
        using detail::SquareRoot;

        // WGS-84 constants — from the single authoritative header
        const Scalar a = Scalar(Environment::WGS84::kSemiMajorAxis_m);
        const Scalar f = Scalar(Environment::WGS84::kFlattening);
        const Scalar e2 = f * (Scalar(2) - f);     // = kEccentricitySq

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        const Scalar one = Scalar(1);
        const Scalar N = a / SquareRoot(one - e2 * sLat * sLat); // prime vertical radius

        const Scalar cosLatN_plus_h = (N + h_m) * cLat;
        const Scalar Z_term = (N * (one - e2) + h_m) * sLat;

        return Vec3<Scalar>{
            cosLatN_plus_h* cLon,  // X
                cosLatN_plus_h* sLon,  // Y
                Z_term                   // Z
        };
    }

    // ── 2. Unit direction in NED from azimuth + zenith ───────────────────────

    template <class Scalar>
    inline Vec3<Scalar> DirectionNEDFromAzimuthZenith(
        const Scalar& azimuth_rad,
        const Scalar& zenith_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sZ = Sine(zenith_rad);
        const Scalar cZ = Cosine(zenith_rad);
        const Scalar sA = Sine(azimuth_rad);
        const Scalar cA = Cosine(azimuth_rad);

        return Vec3<Scalar>{
            sZ* cA,   // N
                sZ* sA,   // E
                -cZ         // D (Down)
        };
    }

    // ── 3. NED → ECEF vector rotation ────────────────────────────────────────

    template <class Scalar>
    inline Vec3<Scalar> NEDToECEF(
        const Vec3<Scalar>& v_ned,
        const Scalar& lat_rad,
        const Scalar& lon_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        const Vec3<Scalar> N{ -sLat * cLon, -sLat * sLon,  cLat };
        const Vec3<Scalar> E{ -sLon,         cLon,         Scalar(0) };
        const Vec3<Scalar> D{ -cLat * cLon, -cLat * sLon, -sLat };

        const Scalar vN = v_ned[0];
        const Scalar vE = v_ned[1];
        const Scalar vD = v_ned[2];

        return Vec3<Scalar>{
            vN* N[0] + vE * E[0] + vD * D[0],
                vN* N[1] + vE * E[1] + vD * D[1],
                vN* N[2] + vE * E[2] + vD * D[2]
        };
    }

    /// ECEF → ECI: inverse of R₃(θ), i.e. R₃(−θ) rotation about Z.
    template <class Scalar>
    inline Vec3<Scalar> ECEFToECI(
        const Vec3<Scalar>& v_ecef,
        const Scalar& theta_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar c = Cosine(theta_rad);
        const Scalar s = Sine(theta_rad);

        return Vec3<Scalar>{
            c* v_ecef[0] - s * v_ecef[1],
                s* v_ecef[0] + c * v_ecef[1],
                v_ecef[2]
        };
    }

    // ── 4. Launch state in ECI ────────────────────────────────────────────────

    template <class Scalar>
    struct LaunchStateECI {
        Vec3<Scalar> r0;
        Vec3<Scalar> dir0_eci;
        Quat<Scalar> q_EB;
    };

    template <class Scalar>
    inline LaunchStateECI<Scalar> MakeLaunchStateECI(
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& h_m,
        const Scalar& azimuth_rad,
        const Scalar& zenith_rad,
        const Scalar& roll_rad,
        const Scalar& earth_rotation_angle_rad)
    {
        using detail::Sine;
        using detail::Cosine;
        using detail::Cross;
        using detail::Normalize;

        const Vec3<Scalar> r_ecef =
            GeodeticToECEF(lat_rad, lon_rad, h_m);
        const Vec3<Scalar> r_eci =
            ECEFToECI(r_ecef, earth_rotation_angle_rad);

        const Vec3<Scalar> dir_ned =
            DirectionNEDFromAzimuthZenith(azimuth_rad, zenith_rad);
        const Vec3<Scalar> forward_ned = Normalize(dir_ned);

        const Vec3<Scalar> up_ref_ned{ Scalar(0), Scalar(0), Scalar(-1) };

        auto make_side_ned = [&](const Vec3<Scalar>& ref_ned) -> Vec3<Scalar> {
            const Vec3<Scalar> candidate = Cross(ref_ned, forward_ned);
            const Scalar n2 = candidate[0] * candidate[0]
                + candidate[1] * candidate[1]
                + candidate[2] * candidate[2];
            if (n2 > Scalar(1e-12)) return Normalize(candidate);
            return Vec3<Scalar>{ Scalar(0), Scalar(0), Scalar(0) };
            };

        Vec3<Scalar> side_ned = make_side_ned(up_ref_ned);
        if (side_ned[0] == Scalar(0) && side_ned[1] == Scalar(0) && side_ned[2] == Scalar(0)) {
            const Vec3<Scalar> north_ref{ Scalar(1), Scalar(0), Scalar(0) };
            side_ned = Normalize(Cross(north_ref, forward_ned));
        }

        const Vec3<Scalar> up_body0_ned = Normalize(Cross(forward_ned, side_ned));
        const Vec3<Scalar> x_b0_ned = forward_ned;
        const Vec3<Scalar> y_b0_ned = side_ned;
        const Vec3<Scalar> z_b0_ned = up_body0_ned;

        const Scalar cR = Cosine(roll_rad);
        const Scalar sR = Sine(roll_rad);

        const Vec3<Scalar> x_b_ned = x_b0_ned;
        const Vec3<Scalar> y_b_ned{
            cR * y_b0_ned[0] + sR * z_b0_ned[0],
            cR * y_b0_ned[1] + sR * z_b0_ned[1],
            cR * y_b0_ned[2] + sR * z_b0_ned[2]
        };
        const Vec3<Scalar> z_b_ned{
            -sR * y_b0_ned[0] + cR * z_b0_ned[0],
            -sR * y_b0_ned[1] + cR * z_b0_ned[1],
            -sR * y_b0_ned[2] + cR * z_b0_ned[2]
        };

        const Vec3<Scalar> x_b_eci = ECEFToECI(NEDToECEF(x_b_ned, lat_rad, lon_rad), earth_rotation_angle_rad);
        const Vec3<Scalar> y_b_eci = ECEFToECI(NEDToECEF(y_b_ned, lat_rad, lon_rad), earth_rotation_angle_rad);
        const Vec3<Scalar> z_b_eci = ECEFToECI(NEDToECEF(z_b_ned, lat_rad, lon_rad), earth_rotation_angle_rad);

        const Quat<Scalar> q_EB =
            detail::RotationMatrixToQuaternion(x_b_eci, y_b_eci, z_b_eci);

        return LaunchStateECI<Scalar>{ r_eci, x_b_eci, q_EB };
    }

    // NED → ECI rotation matrix (frame NED to frame ECI).
    // R_IN[3r+c] is the element at row r, column c (row-major).
    // v^ECI = R_IN * v^NED
    template <class Scalar>
    inline Mat3<Scalar> NEDToInertialRotationMatrix(
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_era_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        const Vec3<Scalar> N_ecef{ -sLat * cLon, -sLat * sLon,  cLat };
        const Vec3<Scalar> E_ecef{ -sLon,         cLon,         Scalar(0) };
        const Vec3<Scalar> D_ecef{ -cLat * cLon, -cLat * sLon, -sLat };

        const Vec3<Scalar> N_eci = ECEFToECI(N_ecef, theta_era_rad);
        const Vec3<Scalar> E_eci = ECEFToECI(E_ecef, theta_era_rad);
        const Vec3<Scalar> D_eci = ECEFToECI(D_ecef, theta_era_rad);

        // Columns of R_IN are N_eci, E_eci, D_eci (row-major storage)
        Mat3<Scalar> R_IN{};
        R_IN[0] = N_eci[0];  R_IN[1] = E_eci[0];  R_IN[2] = D_eci[0];
        R_IN[3] = N_eci[1];  R_IN[4] = E_eci[1];  R_IN[5] = D_eci[1];
        R_IN[6] = N_eci[2];  R_IN[7] = E_eci[2];  R_IN[8] = D_eci[2];

        return R_IN;
    }

} // namespace Aetherion::Coordinate
