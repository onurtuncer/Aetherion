#pragma once

#include <Aetherion/Coordinate/Math.h>
#include <Aetherion/Coordinate/LocalToInertial.h>

namespace Aetherion::Coordinate {

    // -------------------------------------------------------------------------
    // 1) ECI -> ECEF (vector)
    // -------------------------------------------------------------------------
    template <class Scalar>
    inline Vec3<Scalar> ECIToECEF(const Vec3<Scalar>& v_eci, const Scalar& theta_rad)
    {
        using detail::Sine;
        using detail::Cosine;

        const Scalar c = Cosine(theta_rad);
        const Scalar s = Sine(theta_rad);

        // ECEF = R3(theta) * ECI
        return Vec3<Scalar>{
            c* v_eci[0] + s * v_eci[1],
                -s * v_eci[0] + c * v_eci[1],
                v_eci[2]
        };
    }

    // -------------------------------------------------------------------------
    // 2) ECEF -> WGS84 geodetic (moved to .cpp)
    // -------------------------------------------------------------------------
    template <class Scalar>
    void ECEFToGeodeticWGS84(
        const Vec3<Scalar>& r_ecef,
        Scalar& lat_rad,
        Scalar& lon_rad,
        Scalar& h_m);

    // -------------------------------------------------------------------------
    // 3) ECEF -> NED and ECI -> NED (vectors)
    // -------------------------------------------------------------------------
    template <class Scalar>
    inline Vec3<Scalar> ECEFToNED(const Vec3<Scalar>& v_ecef,
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
        const Vec3<Scalar> D{ -cLat * cLon, -cLat * sLon, -sLat }; // Down = -Up

        return Vec3<Scalar>{
            detail::Dot(v_ecef, N),
                detail::Dot(v_ecef, E),
                detail::Dot(v_ecef, D)
        };
    }

    template <class Scalar>
    inline Vec3<Scalar> ECIToNED(const Vec3<Scalar>& v_eci,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_rad)
    {
        return ECEFToNED(ECIToECEF(v_eci, theta_rad), lat_rad, lon_rad);
    }

    // -------------------------------------------------------------------------
    // 4) Orientation helpers
    // -------------------------------------------------------------------------
    template <class Scalar>
    inline void QuaternionToAxesECI(const Quat<Scalar>& q_EB,
        Vec3<Scalar>& x_b_eci,
        Vec3<Scalar>& y_b_eci,
        Vec3<Scalar>& z_b_eci)
    {
        // Use the canonical helper to avoid sign/normalization drift
        const Mat3<Scalar> R = detail::QuaternionToRotationMatrix(q_EB);
        x_b_eci = Vec3<Scalar>{ R[0], R[3], R[6] }; // col0
        y_b_eci = Vec3<Scalar>{ R[1], R[4], R[7] }; // col1
        z_b_eci = Vec3<Scalar>{ R[2], R[5], R[8] }; // col2
    }

    template <class Scalar>
    inline void BodyAxesNEDFromECI(const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_rad,
        Vec3<Scalar>& x_b_ned,
        Vec3<Scalar>& y_b_ned,
        Vec3<Scalar>& z_b_ned)
    {
        // 1) Body axes in ECI
        Vec3<Scalar> x_b_eci, y_b_eci, z_b_eci;
        QuaternionToAxesECI(q_EB, x_b_eci, y_b_eci, z_b_eci);

        // 2) ECI -> ECEF -> NED
        x_b_ned = ECEFToNED(ECIToECEF(x_b_eci, theta_rad), lat_rad, lon_rad);
        y_b_ned = ECEFToNED(ECIToECEF(y_b_eci, theta_rad), lat_rad, lon_rad);
        z_b_ned = ECEFToNED(ECIToECEF(z_b_eci, theta_rad), lat_rad, lon_rad);

        x_b_ned = detail::Normalize(x_b_ned);
        y_b_ned = detail::Normalize(y_b_ned);
        z_b_ned = detail::Normalize(z_b_ned);
    }

    template <class Scalar>
    inline Quat<Scalar> BodyQuaternionInNED(const Quat<Scalar>& q_EB,
        const Scalar& lat_rad,
        const Scalar& lon_rad,
        const Scalar& theta_rad)
    {
        Vec3<Scalar> x_b_ned, y_b_ned, z_b_ned;
        BodyAxesNEDFromECI(q_EB, lat_rad, lon_rad, theta_rad, x_b_ned, y_b_ned, z_b_ned);
        return detail::RotationMatrixToQuaternion(x_b_ned, y_b_ned, z_b_ned); // body->NED
    }

    // -------------------------------------------------------------------------
    // 5) Invert launch state (ECI -> local NED)
    // -------------------------------------------------------------------------
    template <class Scalar>
    struct LocalStateNED {
        Scalar lat_rad{};
        Scalar lon_rad{};
        Scalar h_m{};
        Vec3<Scalar> dir_ned{};
        Quat<Scalar> q_LB{}; // body->Local(NED)
    };

    template <class Scalar>
    inline LocalStateNED<Scalar> InvertLaunchStateToLocalNED(
        const LaunchStateECI<Scalar>& state,
        const Scalar& theta_rad)
    {
        // Position: ECI -> ECEF -> geodetic
        const Vec3<Scalar> r_ecef = ECIToECEF(state.r0, theta_rad);

        Scalar lat{}, lon{}, h{};
        ECEFToGeodeticWGS84(r_ecef, lat, lon, h);

        // Direction: project dir0_eci into NED at recovered lat/lon
        // Build N/E/D basis in ECEF, rotate them to ECI, then dot in ECI.
        using detail::Sine;
        using detail::Cosine;

        const Scalar sLat = Sine(lat);
        const Scalar cLat = Cosine(lat);
        const Scalar sLon = Sine(lon);
        const Scalar cLon = Cosine(lon);

        const Vec3<Scalar> N_ecef{ -sLat * cLon, -sLat * sLon,  cLat };
        const Vec3<Scalar> E_ecef{ -sLon,         cLon,         Scalar(0) };
        const Vec3<Scalar> D_ecef{ -cLat * cLon, -cLat * sLon, -sLat };

        const Vec3<Scalar> N_eci = ECEFToECI(N_ecef, theta_rad);
        const Vec3<Scalar> E_eci = ECEFToECI(E_ecef, theta_rad);
        const Vec3<Scalar> D_eci = ECEFToECI(D_ecef, theta_rad);

        Vec3<Scalar> dir_ned{
            detail::Dot(N_eci, state.dir0_eci),
            detail::Dot(E_eci, state.dir0_eci),
            detail::Dot(D_eci, state.dir0_eci)
        };
        dir_ned = detail::Normalize(dir_ned);

        LocalStateNED<Scalar> out;
        out.lat_rad = lat;
        out.lon_rad = lon;
        out.h_m = h;
        out.dir_ned = dir_ned;
        out.q_LB = BodyQuaternionInNED(state.q_EB, lat, lon, theta_rad);
        return out;
    }

} // namespace Aetherion::Coordinate
