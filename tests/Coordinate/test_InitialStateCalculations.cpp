// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cppad/cppad.hpp>
#include <cmath>
#include <numbers>
#include <vector>

#include <Aetherion/Coordinate/LocalToInertial.h>

using Catch::Approx;

using Aetherion::Coordinate::Vec3;
using Aetherion::Coordinate::Quat;
namespace detail = Aetherion::Coordinate::detail;

using Aetherion::Coordinate::GeodeticToECEF;
using Aetherion::Coordinate::DirectionNEDFromAzimuthZenith;
using Aetherion::Coordinate::NEDToECEF;
using Aetherion::Coordinate::ECEFToECI;
using Aetherion::Coordinate::LaunchStateECI;
using Aetherion::Coordinate::MakeLaunchStateECI;

// Norm of Vec3<double>
inline static double Norm(const Vec3<double>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Dot product of Vec3<double>
inline static double Dot(const Vec3<double>& a, const Vec3<double>& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Norm of quaternion [w,x,y,z]
inline static double Norm(const Quat<double>& q)
{
    return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

// Quaternion * vector (rotate v_B by q_EB: body -> ECI).
// q = [w,x,y,z]
inline static Vec3<double> RotateVectorByQuaternion(const Quat<double>& q,
    const Vec3<double>& v)
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    // q * v (as pure quaternion [0,v])
    const double vw = -x * v[0] - y * v[1] - z * v[2];
    const double vx = w * v[0] + y * v[2] - z * v[1];
    const double vy = w * v[1] + z * v[0] - x * v[2];
    const double vz = w * v[2] + x * v[1] - y * v[0];

    // (q * v) * q_conj
    const double qw = w, qx = -x, qy = -y, qz = -z;

    Vec3<double> out{
        vw * qx + vx * qw + vy * qz - vz * qy,
        vw * qy - vx * qz + vy * qw + vz * qx,
        vw * qz + vx * qy - vy * qx + vz * qw
    };
    return out;
}

// ============================================================================
// 3. MakeLaunchStateECI - basic sanity (position, direction, quaternion)
// ============================================================================

TEST_CASE("MakeLaunchStateECI - equator straight up, zero roll", "[coordinate][launch]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double h = 0.0;
    const double theta = 0.0;  // Earth angle

    const double az = 0.0;
    const double zen = 0.0;
    const double roll = 0.0;

    auto state = MakeLaunchStateECI(
        lat, lon, h,
        az, zen, roll,
        theta);

    // Position should match Geodetic->ECEF->ECI
    auto r_ref_ecef = GeodeticToECEF(lat, lon, h);
    auto r_ref_eci = ECEFToECI(r_ref_ecef, theta);

    REQUIRE(state.r0[0] == Approx(r_ref_eci[0]).margin(1e-6));
    REQUIRE(state.r0[1] == Approx(r_ref_eci[1]).margin(1e-6));
    REQUIRE(state.r0[2] == Approx(r_ref_eci[2]).margin(1e-6));

    // Direction: with zen=0 at equator, Up -> +X_ECI
    REQUIRE(state.dir0_eci[0] == Approx(1.0).margin(1e-12));
    REQUIRE(state.dir0_eci[1] == Approx(0.0).margin(1e-12));
    REQUIRE(state.dir0_eci[2] == Approx(0.0).margin(1e-12));

    // Quaternion must be unit
    Quat<double> q = state.q_EB;
    REQUIRE(Norm(q) == Approx(1.0).epsilon(1e-12));

    // Rotate body basis vectors with q_EB
    Vec3<double> ex_body{ 1.0, 0.0, 0.0 };
    Vec3<double> ey_body{ 0.0, 1.0, 0.0 };
    Vec3<double> ez_body{ 0.0, 0.0, 1.0 };

    Vec3<double> ex_eci = RotateVectorByQuaternion(q, ex_body);
    Vec3<double> ey_eci = RotateVectorByQuaternion(q, ey_body);
    Vec3<double> ez_eci = RotateVectorByQuaternion(q, ez_body);

    // Orthonormality checks
    REQUIRE(Norm(ex_eci) == Approx(1.0).epsilon(1e-12));
    REQUIRE(Norm(ey_eci) == Approx(1.0).epsilon(1e-12));
    REQUIRE(Norm(ez_eci) == Approx(1.0).epsilon(1e-12));

    REQUIRE(Dot(ex_eci, ey_eci) == Approx(0.0).margin(1e-12));
    REQUIRE(Dot(ex_eci, ez_eci) == Approx(0.0).margin(1e-12));
    REQUIRE(Dot(ey_eci, ez_eci) == Approx(0.0).margin(1e-12));

    // ex_eci must coincide with dir0_eci
    REQUIRE(ex_eci[0] == Approx(state.dir0_eci[0]).epsilon(1e-12));
    REQUIRE(ex_eci[1] == Approx(state.dir0_eci[1]).epsilon(1e-12));
    REQUIRE(ex_eci[2] == Approx(state.dir0_eci[2]).epsilon(1e-12));
}

// ============================================================================
// 4. CppAD: r0 sensitivity w.r.t altitude (h)
// ============================================================================

TEST_CASE("MakeLaunchStateECI - CppAD derivatives w.r.t altitude", "[coordinate][launch][CppAD][altitude]")
{
    using AD = CppAD::AD<double>;

    const double lat0 = 0.3;      // some latitude
    const double lon0 = 1.0;      // some longitude
    const double h0 = 1000.0;   // 1 km
    const double az0 = 0.5;
    const double zen0 = 0.4;
    const double roll0 = 0.1;
    const double theta0 = 0.7;

    // Independent variable: altitude only
    std::vector<AD> ax(1);
    ax[0] = AD(h0);
    CppAD::Independent(ax);

    AD lat = AD(lat0);
    AD lon = AD(lon0);
    AD az = AD(az0);
    AD zen = AD(zen0);
    AD roll = AD(roll0);
    AD theta = AD(theta0);
    AD h = ax[0];

    auto state_ad = MakeLaunchStateECI(
        lat, lon, h, az, zen, roll, theta);

    std::vector<AD> ay(3);
    ay[0] = state_ad.r0[0];
    ay[1] = state_ad.r0[1];
    ay[2] = state_ad.r0[2];

    CppAD::ADFun<double> f(ax, ay);

    // AD derivative dr0/dh
    std::vector<double> x0(1);
    x0[0] = h0;
    std::vector<double> J_ad = f.Jacobian(x0);
    REQUIRE(J_ad.size() == 3);

    // Finite-difference reference
    const double dh = 1.0; // 1 m
    auto s_plus = MakeLaunchStateECI(lat0, lon0, h0 + dh, az0, zen0, roll0, theta0);
    auto s_minus = MakeLaunchStateECI(lat0, lon0, h0 - dh, az0, zen0, roll0, theta0);

    Vec3<double> dr_fd{
        (s_plus.r0[0] - s_minus.r0[0]) / (2.0 * dh),
        (s_plus.r0[1] - s_minus.r0[1]) / (2.0 * dh),
        (s_plus.r0[2] - s_minus.r0[2]) / (2.0 * dh)
    };

    REQUIRE(J_ad[0] == Approx(dr_fd[0]).epsilon(1e-6));
    REQUIRE(J_ad[1] == Approx(dr_fd[1]).epsilon(1e-6));
    REQUIRE(J_ad[2] == Approx(dr_fd[2]).epsilon(1e-6));
}

// ============================================================================
// 5. CppAD: direction sensitivity w.r.t azimuth and zenith
// ============================================================================

TEST_CASE("MakeLaunchStateECI - CppAD derivatives of dir0_eci w.r.t azimuth, zenith",
    "[coordinate][launch][CppAD][direction]")
{
    using AD = CppAD::AD<double>;

    const double lat0 = 0.4;
    const double lon0 = -1.1;
    const double h0 = 1500.0;
    const double az0 = 0.7;
    const double zen0 = 0.3;   // not 0 or pi/2, avoid degeneracy
    const double roll0 = 0.2;
    const double theta0 = 0.5;

    // Independent variables: azimuth, zenith
    std::vector<AD> ax(2);
    ax[0] = AD(az0);
    ax[1] = AD(zen0);
    CppAD::Independent(ax);

    AD lat = AD(lat0);
    AD lon = AD(lon0);
    AD h = AD(h0);
    AD roll = AD(roll0);
    AD theta = AD(theta0);
    AD az = ax[0];
    AD zen = ax[1];

    auto state_ad = MakeLaunchStateECI(
        lat, lon, h, az, zen, roll, theta);

    std::vector<AD> ay(3);
    ay[0] = state_ad.dir0_eci[0];
    ay[1] = state_ad.dir0_eci[1];
    ay[2] = state_ad.dir0_eci[2];

    CppAD::ADFun<double> f(ax, ay);

    std::vector<double> x0(2);
    x0[0] = az0;
    x0[1] = zen0;
    std::vector<double> J_ad = f.Jacobian(x0);
    REQUIRE(J_ad.size() == 3 * 2);

    // Finite-difference reference
    const double dh = 1e-4; // small angle step in radians

    auto state_az_p = MakeLaunchStateECI(lat0, lon0, h0, az0 + dh, zen0, roll0, theta0);
    auto state_az_m = MakeLaunchStateECI(lat0, lon0, h0, az0 - dh, zen0, roll0, theta0);
    auto state_zen_p = MakeLaunchStateECI(lat0, lon0, h0, az0, zen0 + dh, roll0, theta0);
    auto state_zen_m = MakeLaunchStateECI(lat0, lon0, h0, az0, zen0 - dh, roll0, theta0);

    Vec3<double> ddir_daz_fd{
        (state_az_p.dir0_eci[0] - state_az_m.dir0_eci[0]) / (2.0 * dh),
        (state_az_p.dir0_eci[1] - state_az_m.dir0_eci[1]) / (2.0 * dh),
        (state_az_p.dir0_eci[2] - state_az_m.dir0_eci[2]) / (2.0 * dh)
    };
    Vec3<double> ddir_dzen_fd{
        (state_zen_p.dir0_eci[0] - state_zen_m.dir0_eci[0]) / (2.0 * dh),
        (state_zen_p.dir0_eci[1] - state_zen_m.dir0_eci[1]) / (2.0 * dh),
        (state_zen_p.dir0_eci[2] - state_zen_m.dir0_eci[2]) / (2.0 * dh)
    };

    // J_ad is 3x2 flattened row-major: [d0/daz, d0/dzen, d1/daz, d1/dzen, ...]
    REQUIRE(J_ad[0] == Approx(ddir_daz_fd[0]).epsilon(1e-5));  // d(dir_x)/daz
    REQUIRE(J_ad[1] == Approx(ddir_dzen_fd[0]).epsilon(1e-5)); // d(dir_x)/dzen
    REQUIRE(J_ad[2] == Approx(ddir_daz_fd[1]).epsilon(1e-5));  // d(dir_y)/daz
    REQUIRE(J_ad[3] == Approx(ddir_dzen_fd[1]).epsilon(1e-5)); // d(dir_y)/dzen
    REQUIRE(J_ad[4] == Approx(ddir_daz_fd[2]).epsilon(1e-5));  // d(dir_z)/daz
    REQUIRE(J_ad[5] == Approx(ddir_dzen_fd[2]).epsilon(1e-5)); // d(dir_z)/dzen
}
