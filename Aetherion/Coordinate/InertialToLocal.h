// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Coordinate/Math.h>
#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Environment/WGS84.h>   // single source of WGS-84 constants

namespace Aetherion::Coordinate {

    /// @brief Rotate a vector from ECI to ECEF frame.
    ///
    /// Applies @f$R_3(\theta)@f$ (rotation about Z by @f$+\theta@f$):
    /// @f[ v^{ECEF} = R_3(\theta)\,v^{ECI} @f]
    /// where @f$\theta@f$ is the Earth Rotation Angle (ERA).
    /// This is the inverse of @c ECEFToECI.
    ///
    /// @param v_eci      3-vector in ECI frame.
    /// @param theta_rad  Earth Rotation Angle @f$\theta_{ERA}@f$ [rad].
    /// @return           Same vector in ECEF frame.
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

    /// @brief Convert an ECEF Cartesian position to WGS-84 geodetic coordinates.
    ///
    /// Iterative Bowring algorithm (converges in ≤ 3 iterations for terrestrial altitudes).
    /// Implemented in @c ECEFtoGeodetic.cpp using @c WGS84::kSemiMajorAxis_m and
    /// @c WGS84::kFlattening.
    ///
    /// @param r_ecef   ECEF position vector @f$[X,\,Y,\,Z]@f$ [m].
    /// @param lat_rad  Output: geodetic latitude [rad].
    /// @param lon_rad  Output: geodetic longitude [rad].
    /// @param h_m      Output: ellipsoidal height above WGS-84 ellipsoid [m].
    template <class Scalar>
    void ECEFToGeodeticWGS84(
        const Vec3<Scalar>& r_ecef,
        Scalar& lat_rad,
        Scalar& lon_rad,
        Scalar& h_m);

    /// @brief Rotate a vector from ECEF to local NED frame.
    ///
    /// Applies the direction-cosine matrix @f$R_{NED}^{ECEF}(\phi,\lambda)@f$.
    /// Only rotation; no origin translation.
    ///
    /// @param v_ecef    3-vector in ECEF frame.
    /// @param lat_rad   Geodetic latitude of the NED origin [rad].
    /// @param lon_rad   Geodetic longitude of the NED origin [rad].
    /// @return          Same vector in NED frame @f$[N,\,E,\,D]@f$.
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

        const Vec3<Scalar> N{ -sLat * cLon, -sLat * sLon,  cLat };
        const Vec3<Scalar> E{ -sLon,         cLon,         Scalar(0) };
        const Vec3<Scalar> D{ -cLat * cLon, -cLat * sLon, -sLat };

        return Vec3<Scalar>{
            detail::Dot(v_ecef, N),
                detail::Dot(v_ecef, E),
                detail::Dot(v_ecef, D)
        };
    }

    /// @brief Rotate a vector from ECI to local NED frame.
    ///
    /// Convenience composition: ECI → ECEF (@c ECIToECEF) then ECEF → NED (@c ECEFToNED).
    ///
    /// @param v_eci      3-vector in ECI frame.
    /// @param lat_rad    Geodetic latitude of the NED origin [rad].
    /// @param lon_rad    Geodetic longitude of the NED origin [rad].
    /// @param theta_rad  Earth Rotation Angle @f$\theta_{ERA}@f$ [rad].
    /// @return           Same vector in NED frame.
    template <class Scalar>
    inline Vec3<Scalar> ECIToNED(
        const Vec3<Scalar>& v_eci,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_rad)
    {
        return ECEFToNED(ECIToECEF(v_eci, theta_rad), lat_rad, lon_rad);
    }

    /// @brief Extract body-frame axis unit vectors in ECI from an attitude quaternion.
    ///
    /// Given @f$q_{EB}@f$ (body → ECI), returns the three body-frame axes expressed in ECI.
    ///
    /// @param q_EB       Quaternion @f$[w,x,y,z]@f$ rotating body frame into ECI.
    /// @param x_b_eci    Output: body x-axis (forward) expressed in ECI.
    /// @param y_b_eci    Output: body y-axis (right) expressed in ECI.
    /// @param z_b_eci    Output: body z-axis (down) expressed in ECI.
    template <class Scalar>
    inline void QuaternionToAxesECI(
        const Quat<Scalar>& q_EB,
        Vec3<Scalar>& x_b_eci,
        Vec3<Scalar>& y_b_eci,
        Vec3<Scalar>& z_b_eci)
    {
        const Mat3<Scalar> R = detail::QuaternionToRotationMatrix(q_EB);
        x_b_eci = Vec3<Scalar>{ R[0], R[3], R[6] };
        y_b_eci = Vec3<Scalar>{ R[1], R[4], R[7] };
        z_b_eci = Vec3<Scalar>{ R[2], R[5], R[8] };
    }

    /// @brief Express the three body-frame axes in the local NED frame.
    ///
    /// Transforms each body axis from ECI to NED via ECI → ECEF → NED, then normalises.
    ///
    /// @param q_EB       Quaternion rotating body frame into ECI.
    /// @param lat_rad    Geodetic latitude of the NED origin [rad].
    /// @param lon_rad    Geodetic longitude of the NED origin [rad].
    /// @param theta_rad  Earth Rotation Angle [rad].
    /// @param x_b_ned    Output: body x-axis (forward) in NED (unit vector).
    /// @param y_b_ned    Output: body y-axis (right) in NED (unit vector).
    /// @param z_b_ned    Output: body z-axis (down) in NED (unit vector).
    template <class Scalar>
    inline void BodyAxesNEDFromECI(
        const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_rad,
        Vec3<Scalar>& x_b_ned,
        Vec3<Scalar>& y_b_ned,
        Vec3<Scalar>& z_b_ned)
    {
        Vec3<Scalar> x_b_eci, y_b_eci, z_b_eci;
        QuaternionToAxesECI(q_EB, x_b_eci, y_b_eci, z_b_eci);

        x_b_ned = ECEFToNED(ECIToECEF(x_b_eci, theta_rad), lat_rad, lon_rad);
        y_b_ned = ECEFToNED(ECIToECEF(y_b_eci, theta_rad), lat_rad, lon_rad);
        z_b_ned = ECEFToNED(ECIToECEF(z_b_eci, theta_rad), lat_rad, lon_rad);

        x_b_ned = detail::Normalize(x_b_ned);
        y_b_ned = detail::Normalize(y_b_ned);
        z_b_ned = detail::Normalize(z_b_ned);
    }

    /// @brief Compute the body→NED attitude quaternion from the body→ECI quaternion.
    ///
    /// Rotates the body axes into NED via @c BodyAxesNEDFromECI, then converts
    /// the resulting DCM back to a quaternion.
    ///
    /// @param q_EB       Quaternion @f$[w,x,y,z]@f$ rotating body into ECI.
    /// @param lat_rad    Geodetic latitude of the NED origin [rad].
    /// @param lon_rad    Geodetic longitude of the NED origin [rad].
    /// @param theta_rad  Earth Rotation Angle [rad].
    /// @return           Quaternion rotating body frame into local NED frame.
    template <class Scalar>
    inline Quat<Scalar> BodyQuaternionInNED(
        const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_rad)
    {
        Vec3<Scalar> x_b_ned, y_b_ned, z_b_ned;
        BodyAxesNEDFromECI(q_EB, lat_rad, lon_rad, theta_rad,
            x_b_ned, y_b_ned, z_b_ned);
        return detail::RotationMatrixToQuaternion(x_b_ned, y_b_ned, z_b_ned);
    }

    /// @brief Current vehicle state expressed in local NED coordinates.
    template <class Scalar>
    struct LocalStateNED {
        Scalar       lat_rad{}; ///< Geodetic latitude [rad].
        Scalar       lon_rad{}; ///< Geodetic longitude [rad].
        Scalar       h_m{};     ///< Ellipsoidal height [m].
        Vec3<Scalar> dir_ned{}; ///< Body forward direction (unit vector) in NED.
        Quat<Scalar> q_LB{};    ///< Quaternion rotating body frame into local NED frame.
    };

    /// @brief Convert an ECI launch state back to local NED coordinates.
    ///
    /// Inverts the ECI → local-NED chain:
    ///   -# ECI position → ECEF → geodetic (@c ECEFToGeodeticWGS84).
    ///   -# ECI forward direction → NED (@c ECIToNED + normalise).
    ///   -# Body attitude quaternion in NED (@c BodyQuaternionInNED).
    ///
    /// @param state      ECI launch state (position, forward direction, quaternion).
    /// @param theta_rad  Earth Rotation Angle @f$\theta_{ERA}@f$ at the epoch [rad].
    /// @return           @c LocalStateNED with geodetic coordinates, NED direction, and NED quaternion.
    template <class Scalar>
    inline LocalStateNED<Scalar> InvertLaunchStateToLocalNED(
        const LaunchStateECI<Scalar>& state,
        const Scalar& theta_rad)
    {
        const Vec3<Scalar> r_ecef = ECIToECEF(state.r0, theta_rad);

        Scalar lat{}, lon{}, h{};
        ECEFToGeodeticWGS84(r_ecef, lat, lon, h);

        // Direction: project dir0_eci into NED using the standard functions
        // — no repeated sin/cos computations.
        const Vec3<Scalar> dir_ned_raw = ECIToNED(state.dir0_eci, lat, lon, theta_rad);
        const Vec3<Scalar> dir_ned = detail::Normalize(dir_ned_raw);

        LocalStateNED<Scalar> out;
        out.lat_rad = lat;
        out.lon_rad = lon;
        out.h_m = h;
        out.dir_ned = dir_ned;
        out.q_LB = BodyQuaternionInNED(state.q_EB, lat, lon, theta_rad);
        return out;
    }

} // namespace Aetherion::Coordinate