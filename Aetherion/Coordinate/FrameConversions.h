// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

// ------------------------------------------------------------------------------
// Launch geometry: WGS-84 + NEU -> ECI position and attitude (quaternion).
// ------------------------------------------------------------------------------

#include <array>
#include <cmath>

namespace Aetherion::Coordinate {

    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

    template <class Scalar>
    using Quat = std::array<Scalar, 4>; // [w, x, y, z], body->ECI

    namespace detail {

        template <class S>
        inline S Sine(const S& x) {
            using std::sin;
            return sin(x);
        }

        template <class S>
        inline S Cosine(const S& x) {
            using std::cos;
            return cos(x);
        }

        template <class S>
        inline S SquareRoot(const S& x) {
            using std::sqrt;
            return sqrt(x);
        }

        template <class S>
        inline Vec3<S> Cross(const Vec3<S>& a, const Vec3<S>& b) {
            return Vec3<S>{
                a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]
            };
        }

        template <class S>
        inline Vec3<S> Normalize(const Vec3<S>& v) {
            const S n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            const S inv_n = S(1) / SquareRoot(n2);
            return Vec3<S>{ v[0] * inv_n, v[1] * inv_n, v[2] * inv_n };
        }

        /// Convert rotation matrix (columns = body axes in some world frame)
        /// to quaternion q_WB (body->world), robust for all rotations.
        template <class S>
        inline Quat<S> RotationMatrixToQuaternion(
            const Vec3<S>& x_b, // column 0
            const Vec3<S>& y_b, // column 1
            const Vec3<S>& z_b  // column 2
        )
        {
            using detail::SquareRoot;

            const S m00 = x_b[0], m01 = y_b[0], m02 = z_b[0];
            const S m10 = x_b[1], m11 = y_b[1], m12 = z_b[1];
            const S m20 = x_b[2], m21 = y_b[2], m22 = z_b[2];

            const S one = S(1);

            const S trace = m00 + m11 + m22;

            S w, x, y, z;

            if (trace > S(0)) {
                // trace is positive: use w as the primary
                const S S4 = SquareRoot(trace + one) * S(2); // 4*w
                w = S4 * S(0.25);
                x = (m21 - m12) / S4;
                y = (m02 - m20) / S4;
                z = (m10 - m01) / S4;
            }
            else if (m00 > m11 && m00 > m22) {
                // m00 is largest on diagonal: use x as the primary
                const S S4 = SquareRoot(one + m00 - m11 - m22) * S(2); // 4*x
                w = (m21 - m12) / S4;
                x = S4 * S(0.25);
                y = (m01 + m10) / S4;
                z = (m02 + m20) / S4;
            }
            else if (m11 > m22) {
                // m11 is largest: use y as the primary
                const S S4 = SquareRoot(one + m11 - m00 - m22) * S(2); // 4*y
                w = (m02 - m20) / S4;
                x = (m01 + m10) / S4;
                y = S4 * S(0.25);
                z = (m12 + m21) / S4;
            }
            else {
                // m22 is largest: use z as the primary
                const S S4 = SquareRoot(one + m22 - m00 - m11) * S(2); // 4*z
                w = (m10 - m01) / S4;
                x = (m02 + m20) / S4;
                y = (m12 + m21) / S4;
                z = S4 * S(0.25);
            }

            // Final normalize to clean up numerical drift
            const S norm2 = w * w + x * x + y * y + z * z;
            const S inv_n = one / SquareRoot(norm2);

            return Quat<S>{ w* inv_n, x* inv_n, y* inv_n, z* inv_n };
        }


    } // namespace detail

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
        const Scalar a = Scalar(6378137.0);                 // semi-major axis [m]
        const Scalar f = Scalar(1.0 / 298.257223563);       // flattening
        const Scalar e2 = f * (Scalar(2) - f);               // eccentricity^2

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        const Scalar one = Scalar(1);
        const Scalar tmp = one - e2 * sLat * sLat;
        const Scalar N = a / SquareRoot(tmp);              // prime vertical radius

        const Scalar cosLatN_plus_h = (N + h_m) * cLat;
        const Scalar Z_term = (N * (one - e2) + h_m) * sLat;

        return Vec3<Scalar>{
            cosLatN_plus_h* cLon,   // X
                cosLatN_plus_h* sLon,   // Y
                Z_term                   // Z
        };
    }

    // ---------------------- 2. Direction in NEU ------------------------------

    /// Unit direction in local NEU from azimuth (north->east) and zenith (from Up).
    template <class Scalar>
    inline Vec3<Scalar> DirectionNEUFromAzimuthZenith(
        const Scalar& azimuth_rad,
        const Scalar& zenith_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sZ = Sine(zenith_rad);
        const Scalar cZ = Cosine(zenith_rad);
        const Scalar sA = Sine(azimuth_rad);
        const Scalar cA = Cosine(azimuth_rad);

        // [N, E, U]
        return Vec3<Scalar>{
            sZ* cA,  // N
                sZ* sA,  // E
                cZ        // U
        };
    }

    // ---------------------- 3. NEU -> ECEF and ECEF -> ECI -------------------

    /// Transform a vector from local NEU at (lat, lon) to ECEF.
    /// lat, lon in radians (geodetic).
    template <class Scalar>
    inline Vec3<Scalar> NEUToECEF(
        const Vec3<Scalar>& v_neu,
        const Scalar& lat_rad,
        const Scalar& lon_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar sLat = Sine(lat_rad);
        const Scalar cLat = Cosine(lat_rad);
        const Scalar sLon = Sine(lon_rad);
        const Scalar cLon = Cosine(lon_rad);

        // Basis vectors of NEU expressed in ECEF
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

        const Scalar vN = v_neu[0];
        const Scalar vE = v_neu[1];
        const Scalar vU = v_neu[2];

        return Vec3<Scalar>{
            vN* N[0] + vE * E[0] + vU * U[0],
                vN* N[1] + vE * E[1] + vU * U[1],
                vN* N[2] + vE * E[2] + vU * U[2]
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
    ///   azimuth, zenith: launch direction in NEU
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

        // 2) Body axes in NEU before roll
        const Vec3<Scalar> dir_neu =
            DirectionNEUFromAzimuthZenith(azimuth_rad, zenith_rad); // unit by construction

        const Vec3<Scalar> forward_neu = Normalize(dir_neu); // body +x

        // Reference "up" in NEU for zero roll
        const Vec3<Scalar> up_ref_neu{ Scalar(0), Scalar(0), Scalar(1) };

        // Side axis (body y) before roll: s = normalize(ref × forward)
        // Guard against degeneracy when forward is parallel to ref.
        auto make_side_neu = [&](const Vec3<Scalar>& ref_neu) -> Vec3<Scalar> {
            const Vec3<Scalar> candidate = Cross(ref_neu, forward_neu);
            const Scalar n2 = candidate[0] * candidate[0]
                + candidate[1] * candidate[1]
                + candidate[2] * candidate[2];
            if (n2 > Scalar(1e-12)) {
                return Normalize(candidate);
            }
            return Vec3<Scalar>{ Scalar(0), Scalar(0), Scalar(0) };
            };

        Vec3<Scalar> side_neu = make_side_neu(up_ref_neu);

        // If degenerate, use North as fallback reference: [N,E,U] = [1,0,0] in NEU.
        if (side_neu[0] == Scalar(0) &&
            side_neu[1] == Scalar(0) &&
            side_neu[2] == Scalar(0))
        {
            const Vec3<Scalar> north_ref_neu{ Scalar(1), Scalar(0), Scalar(0) };
            side_neu = Normalize(Cross(north_ref_neu, forward_neu));
        }

        // Third axis (body z) before roll: z = x × y
        const Vec3<Scalar> up_body0_neu = Normalize(
            Cross(forward_neu, side_neu));

        const Vec3<Scalar> x_b0_neu = forward_neu;   // body +x
        const Vec3<Scalar> y_b0_neu = side_neu;      // body +y
        const Vec3<Scalar> z_b0_neu = up_body0_neu;  // body +z

        // 3) Apply roll about body +x axis (in body basis)
        const Scalar cR = Cosine(roll_rad);
        const Scalar sR = Sine(roll_rad);

        Vec3<Scalar> x_b_neu = x_b0_neu; // unchanged

        Vec3<Scalar> y_b_neu{
            cR * y_b0_neu[0] + sR * z_b0_neu[0],
            cR * y_b0_neu[1] + sR * z_b0_neu[1],
            cR * y_b0_neu[2] + sR * z_b0_neu[2]
        };

        Vec3<Scalar> z_b_neu{
            -sR * y_b0_neu[0] + cR * z_b0_neu[0],
            -sR * y_b0_neu[1] + cR * z_b0_neu[1],
            -sR * y_b0_neu[2] + cR * z_b0_neu[2]
        };

        // 4) Transform body axes NEU -> ECEF -> ECI
        const Vec3<Scalar> x_b_ecef = NEUToECEF(x_b_neu, lat_rad, lon_rad);
        const Vec3<Scalar> y_b_ecef = NEUToECEF(y_b_neu, lat_rad, lon_rad);
        const Vec3<Scalar> z_b_ecef = NEUToECEF(z_b_neu, lat_rad, lon_rad);

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

} // namespace Aetherion::Coordinate


