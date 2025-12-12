// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Coordinate/Math.h>

// ------------------------------------------------------------------------------
// Launch geometry: WGS-84 + NED -> ECI position and attitude (quaternion).
// Local frame is right-handed North–East–Down.
// ------------------------------------------------------------------------------

namespace Aetherion::Coordinate{

    // ---------------------- 1. WGS-84 geodetic -> ECEF ------------------------

    /// Geodetic (lat, lon, h) -> ECEF [m], WGS-84 ellipsoid.
    /// lat, lon in radians, h in meters (height above ellipsoid).
    template <class Scalar>
    inline Vec3<Scalar> GeodeticToECEF(
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& h_m)
    {
        using detail::Sine;
        using detail::Cosine;
        using detail::SquareRoot;

        // WGS-84 constants
        const Scalar a = Scalar(6378137.0);              // semi-major axis [m]
        const Scalar f = Scalar(1.0 / 298.257223563);    // flattening
        const Scalar e2 = f * (Scalar(2) - f);           // eccentricity^2

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        const Scalar one = Scalar(1);
        const Scalar tmp = one - e2 * sLat * sLat;
        const Scalar N = a / SquareRoot(tmp);           // prime vertical radius

        const Scalar cosLatN_plus_h = (N + h_m) * cLat;
        const Scalar Z_term = (N * (one - e2) + h_m) * sLat;

        return Vec3<Scalar>{
            cosLatN_plus_h* cLon,   // X
                cosLatN_plus_h* sLon,   // Y
                Z_term                   // Z
        };
    }

    // ---------------------- 2. Direction in NED ------------------------------

    /// Unit direction in local NED from azimuth (north->east) and zenith (from Up).
    /// NED is [N, E, D] with +D pointing Down, so:
    ///  - for zenith = 0 (straight Up), D = -1,
    ///  - for zenith = pi (straight Down), D = +1.
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

        // [N, E, D], with D = -U (Up)
        return Vec3<Scalar>{
            sZ* cA,   // N
                sZ* sA,   // E
                -cZ        // D (Down)
        };
    }

    // ---------------------- 3. NED -> ECEF and ECEF -> ECI -------------------

    /// Transform a vector from local NED at (lat, lon) to ECEF.
    /// lat, lon in radians (geodetic).
    /// NED is right-handed with N × E = D.
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

        // Basis vectors of NED expressed in ECEF
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
        // D = -U, with U the usual Up vector in ECEF
        const Vec3<Scalar> D{
            -cLat * cLon,
            -cLat * sLon,
            -sLat
        };

        const Scalar vN = v_ned[0];
        const Scalar vE = v_ned[1];
        const Scalar vD = v_ned[2];

        return Vec3<Scalar>{
            vN* N[0] + vE * E[0] + vD * D[0],
                vN* N[1] + vE * E[1] + vD * D[1],
                vN* N[2] + vE * E[2] + vD * D[2]
        };
    }

    /// Rotate a vector from ECEF to ECI using Earth rotation angle theta (radians).
    /// theta is the angle that rotates ECI -> ECEF; we apply the inverse here.
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

    // ---------------------- 4. Launch state in ECI ---------------------------

    /// Launch state in ECI:
    /// - r0: position in ECI [m]
    /// - dir0_eci: unit launch direction in ECI (body +x axis)
    /// - q_EB: unit quaternion [w,x,y,z] that maps body->ECI
    ///         (v_ECI = q_EB * v_B * q_EB^-1)
    template <class Scalar>
    struct LaunchStateECI {
        Vec3<Scalar> r0;
        Vec3<Scalar> dir0_eci;
        Quat<Scalar> q_EB;
    };

    /// Compute initial launch state in ECI (position + direction + quaternion),
    /// given:
    ///   lat, lon, h: geodetic WGS-84 site (radians, radians, meters)
    ///   azimuth, zenith: launch direction in local NED
    ///                    (azimuth north->east, zenith from Up)
    ///   roll: rotation about body +x axis (aligned with launch direction)
    ///   earth_rotation_angle: Earth rotation angle at t0 (radians)
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

        // 1) Position: geodetic -> ECEF -> ECI
        const Vec3<Scalar> r_ecef =
            GeodeticToECEF(lat_rad, lon_rad, h_m);

        const Vec3<Scalar> r_eci =
            ECEFToECI(r_ecef, earth_rotation_angle_rad);

        // 2) Body axes in NED before roll
        const Vec3<Scalar> dir_ned =
            DirectionNEDFromAzimuthZenith(azimuth_rad, zenith_rad); // unit by construction

        const Vec3<Scalar> forward_ned = Normalize(dir_ned); // body +x

        // Reference "Up" in NED coordinates for zero roll: Up = -D.
        const Vec3<Scalar> up_ref_ned{ Scalar(0), Scalar(0), Scalar(-1) };

        // Side axis (body y) before roll: s = normalize(ref × forward)
        // Guard against degeneracy when forward is parallel to ref.
        auto make_side_ned = [&](const Vec3<Scalar>& ref_ned) -> Vec3<Scalar> {
            const Vec3<Scalar> candidate = Cross(ref_ned, forward_ned);
            const Scalar n2 = candidate[0] * candidate[0]
                + candidate[1] * candidate[1]
                + candidate[2] * candidate[2];
            if (n2 > Scalar(1e-12)) {
                return Normalize(candidate);
            }
            return Vec3<Scalar>{ Scalar(0), Scalar(0), Scalar(0) };
            };

        Vec3<Scalar> side_ned = make_side_ned(up_ref_ned);

        // If degenerate, use North as fallback reference: [N,E,D] = [1,0,0] in NED.
        if (side_ned[0] == Scalar(0) &&
            side_ned[1] == Scalar(0) &&
            side_ned[2] == Scalar(0))
        {
            const Vec3<Scalar> north_ref_ned{ Scalar(1), Scalar(0), Scalar(0) };
            side_ned = Normalize(Cross(north_ref_ned, forward_ned));
        }

        // Third axis (body z) before roll: z = x × y
        const Vec3<Scalar> up_body0_ned = Normalize(
            Cross(forward_ned, side_ned));

        const Vec3<Scalar> x_b0_ned = forward_ned;   // body +x
        const Vec3<Scalar> y_b0_ned = side_ned;      // body +y
        const Vec3<Scalar> z_b0_ned = up_body0_ned;  // body +z

        // 3) Apply roll about body +x axis (in body basis)
        const Scalar cR = Cosine(roll_rad);
        const Scalar sR = Sine(roll_rad);

        Vec3<Scalar> x_b_ned = x_b0_ned; // unchanged

        Vec3<Scalar> y_b_ned{
            cR * y_b0_ned[0] + sR * z_b0_ned[0],
            cR * y_b0_ned[1] + sR * z_b0_ned[1],
            cR * y_b0_ned[2] + sR * z_b0_ned[2]
        };

        Vec3<Scalar> z_b_ned{
            -sR * y_b0_ned[0] + cR * z_b0_ned[0],
            -sR * y_b0_ned[1] + cR * z_b0_ned[1],
            -sR * y_b0_ned[2] + cR * z_b0_ned[2]
        };

        // 4) Transform body axes NED -> ECEF -> ECI
        const Vec3<Scalar> x_b_ecef = NEDToECEF(x_b_ned, lat_rad, lon_rad);
        const Vec3<Scalar> y_b_ecef = NEDToECEF(y_b_ned, lat_rad, lon_rad);
        const Vec3<Scalar> z_b_ecef = NEDToECEF(z_b_ned, lat_rad, lon_rad);

        const Vec3<Scalar> x_b_eci = ECEFToECI(x_b_ecef, earth_rotation_angle_rad);
        const Vec3<Scalar> y_b_eci = ECEFToECI(y_b_ecef, earth_rotation_angle_rad);
        const Vec3<Scalar> z_b_eci = ECEFToECI(z_b_ecef, earth_rotation_angle_rad);

        // 5) Quaternion body->ECI
        const Quat<Scalar> q_EB =
            detail::RotationMatrixToQuaternion(x_b_eci, y_b_eci, z_b_eci);

        LaunchStateECI<Scalar> out{
            r_eci,
            x_b_eci, // launch direction in ECI
            q_EB
        };
        return out;
    }

    // NED -> ECI rotation matrix (frame NED to frame ECI).
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

    // NED basis expressed in ECEF (matches your NEDToECEF() basis)
    const Vec3<Scalar> N_ecef{ -sLat * cLon, -sLat * sLon,  cLat };
    const Vec3<Scalar> E_ecef{ -sLon,         cLon,         Scalar(0) };
    const Vec3<Scalar> D_ecef{ -cLat * cLon, -cLat * sLon, -sLat }; // Down

    // Transform each basis vector ECEF -> ECI using the SAME convention as ECEFToECI()
    const Vec3<Scalar> N_eci = ECEFToECI(N_ecef, theta_era_rad);
    const Vec3<Scalar> E_eci = ECEFToECI(E_ecef, theta_era_rad);
    const Vec3<Scalar> D_eci = ECEFToECI(D_ecef, theta_era_rad);

    // Assemble R_IN with columns [N_eci | E_eci | D_eci] in row-major storage.
    Mat3<Scalar> R_IN{};
    R_IN[0] = N_eci[0];  R_IN[1] = E_eci[0];  R_IN[2] = D_eci[0];
    R_IN[3] = N_eci[1];  R_IN[4] = E_eci[1];  R_IN[5] = D_eci[1];
    R_IN[6] = N_eci[2];  R_IN[7] = E_eci[2];  R_IN[8] = D_eci[2];

    return R_IN;
}


    /*
    // NED -> ECI rotation matrix (frame NED to frame ECI).
    // v^ECI = R_IN * v^NED
    template <class Scalar>
    Mat3<Scalar>
        NEDToInertialRotationMatrix(const Scalar& lat_rad,
            const Scalar& lon_rad,
            const Scalar& theta_era_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sphi = Sine(lat_rad);
        const Scalar cphi = Cosine(lat_rad);
        const Scalar slon = Sine(lon_rad);
        const Scalar clon = Cosine(lon_rad);

        // ---------------------------------------------------------------------
        // R_NED^ECEF: ECEF -> NED (standard NED DCM, rows are N,E,D in ECEF)
        //
        // v^NED = R_NED^ECEF * v^ECEF
        // ---------------------------------------------------------------------
        Mat3<Scalar> R_NED_ECEF{};

        R_NED_ECEF[0] = -sphi * clon;  // row 0, col 0
        R_NED_ECEF[1] = -sphi * slon;  // row 0, col 1
        R_NED_ECEF[2] = cphi;         // row 0, col 2

        R_NED_ECEF[3] = -slon;         // row 1, col 0
        R_NED_ECEF[4] = clon;         // row 1, col 1
        R_NED_ECEF[5] = Scalar(0);    // row 1, col 2

        R_NED_ECEF[6] = -cphi * clon;  // row 2, col 0
        R_NED_ECEF[7] = -cphi * slon;  // row 2, col 1
        R_NED_ECEF[8] = -sphi;         // row 2, col 2

        // ECEF <- NED is the transpose
        Mat3<Scalar> R_ECEF_NED{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_ECEF_NED[3 * r + c] = R_NED_ECEF[3 * c + r];
            }
        }

        // ---------------------------------------------------------------------
        // ECI <- ECEF: R3(-theta_era)
        //
        // If r^ECEF = R3(theta_era) * r^ECI,
        // then r^ECI = R3(-theta_era) * r^ECEF.
        // ---------------------------------------------------------------------
        const Scalar ctheta = Cosine(theta_era_rad);
        const Scalar stheta = Sine(theta_era_rad);

        Mat3<Scalar> R_ECI_ECEF{};
        // row 0
        R_ECI_ECEF[0] = ctheta;
        R_ECI_ECEF[1] = stheta;
        R_ECI_ECEF[2] = Scalar(0);
        // row 1
        R_ECI_ECEF[3] = -stheta;
        R_ECI_ECEF[4] = ctheta;
        R_ECI_ECEF[5] = Scalar(0);
        // row 2
        R_ECI_ECEF[6] = Scalar(0);
        R_ECI_ECEF[7] = Scalar(0);
        R_ECI_ECEF[8] = Scalar(1);

        // ---------------------------------------------------------------------
        // Compose: v^ECI = R_ECI_ECEF * ( R_ECEF_NED * v^NED )
        // => R_IN = R_ECI_ECEF * R_ECEF_NED
        // ---------------------------------------------------------------------
        Mat3<Scalar> R_IN{};

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                Scalar sum = Scalar(0);
                for (int k = 0; k < 3; ++k) {
                    sum += R_ECI_ECEF[3 * r + k] * R_ECEF_NED[3 * k + c];
                }
                R_IN[3 * r + c] = sum;
            }
        }

        return R_IN;
    } */

    // NED -> ECI orientation quaternion.
    // Returns q_ned_to_inertial such that v^ECI = R(q) * v^NED.
    template <class Scalar>
    Quat<Scalar>
        NEDToInertialQuaternion(const Scalar& lat_rad,
            const Scalar& lon_rad,
            const Scalar& theta_era_rad)
    {
        const Mat3<Scalar> R_IN =
            NEDToInertialRotationMatrix(lat_rad, lon_rad, theta_era_rad);

        // Extract columns of R_IN (N,E,D axes in ECI) and reuse the Vec3-based helper.
        const Vec3<Scalar> x_b{ R_IN[0], R_IN[3], R_IN[6] };
        const Vec3<Scalar> y_b{ R_IN[1], R_IN[4], R_IN[7] };
        const Vec3<Scalar> z_b{ R_IN[2], R_IN[5], R_IN[8] };

        return detail::RotationMatrixToQuaternion(x_b, y_b, z_b);
    }

} // namespace Aetherion::Coordinate
