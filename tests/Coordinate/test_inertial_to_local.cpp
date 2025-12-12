// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <numbers>

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Coordinate/InertialToLocal.h>

using Catch::Approx;

using Aetherion::Coordinate::Vec3;
using Aetherion::Coordinate::Quat;
using Aetherion::Coordinate::Mat3;

namespace detail = Aetherion::Coordinate::detail;

using Aetherion::Coordinate::MakeLaunchStateECI;
using Aetherion::Coordinate::LaunchStateECI;

using Aetherion::Coordinate::InvertLaunchStateToLocalNED;   // <-- NED version
using Aetherion::Coordinate::LocalStateNED;                 // <-- NED version

using Aetherion::Coordinate::DirectionNEDFromAzimuthZenith; // already NED

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static double Norm(const Vec3<double>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Wrap longitude difference into [-pi, pi] for comparison
static double LonDiff(double lon_rec, double lon_ref) {
    const double two_pi = 2.0 * std::numbers::pi;
    double d = lon_rec - lon_ref;
    return std::remainder(d, two_pi);
}

// Extract body axes in "A" from q_AB (body->A): columns of R(q_AB)
static void AxesFromQuat(
    const Quat<double>& q_AB,
    Vec3<double>& x_b_A,
    Vec3<double>& y_b_A,
    Vec3<double>& z_b_A)
{
    const Mat3<double> R = detail::QuaternionToRotationMatrix(q_AB);
    // columns (row-major)
    x_b_A = Vec3<double>{ R[0], R[3], R[6] };
    y_b_A = Vec3<double>{ R[1], R[4], R[7] };
    z_b_A = Vec3<double>{ R[2], R[5], R[8] };
}

static void CheckOrthonormalFrame(
    const Vec3<double>& x,
    const Vec3<double>& y,
    const Vec3<double>& z,
    double eps = 1e-12)
{
    const auto xn = detail::Normalize(x);
    const auto yn = detail::Normalize(y);
    const auto zn = detail::Normalize(z);

    REQUIRE(Norm(xn) == Approx(1.0).margin(eps));
    REQUIRE(Norm(yn) == Approx(1.0).margin(eps));
    REQUIRE(Norm(zn) == Approx(1.0).margin(eps));

    REQUIRE(detail::Dot(xn, yn) == Approx(0.0).margin(eps));
    REQUIRE(detail::Dot(xn, zn) == Approx(0.0).margin(eps));
    REQUIRE(detail::Dot(yn, zn) == Approx(0.0).margin(eps));
}

// -----------------------------------------------------------------------------
// NED Round-trip tests
// -----------------------------------------------------------------------------

TEST_CASE("InvertLaunchStateToLocalNED - equator straight up, zero roll",
    "[coordinate][inverse][launch][ned]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double h = 0.0;
    const double theta = 0.0;

    const double az = 0.0;
    const double zen = 0.0; // zenith from Up => 0 = Up
    const double roll = 0.0;

    const LaunchStateECI<double> state =
        MakeLaunchStateECI(lat, lon, h, az, zen, roll, theta);

    const LocalStateNED<double> local =
        InvertLaunchStateToLocalNED(state, theta);

    // Site must come back
    REQUIRE(local.lat_rad == Approx(lat).margin(1e-12));
    REQUIRE(LonDiff(local.lon_rad, lon) == Approx(0.0).margin(1e-12));
    REQUIRE(local.h_m == Approx(h).margin(1e-6));

    // Direction in NED: Up means D = -1 (since +D is Down)
    REQUIRE(local.dir_ned[0] == Approx(0.0).margin(1e-12));  // N
    REQUIRE(local.dir_ned[1] == Approx(0.0).margin(1e-12));  // E
    REQUIRE(local.dir_ned[2] == Approx(-1.0).margin(1e-12)); // D

    // q_DB (body->NED) must be unit
    const Quat<double>& q_LB = local.q_LB;
    const double qnorm = std::sqrt(q_LB[0] * q_LB[0] + q_LB[1] * q_LB[1] + q_LB[2] * q_LB[2] + q_LB[3] * q_LB[3]);
    REQUIRE(qnorm == Approx(1.0).margin(1e-12));

    // Axes from q_DB are in NED
    Vec3<double> x_b_ned, y_b_ned, z_b_ned;
    AxesFromQuat(q_LB, x_b_ned, y_b_ned, z_b_ned);

    CheckOrthonormalFrame(x_b_ned, y_b_ned, z_b_ned, 1e-12);

    // Body +x in NED should align with launch direction in NED
    const auto ex_ned = detail::Normalize(x_b_ned);
    const auto dir_ned = detail::Normalize(local.dir_ned);

    REQUIRE(ex_ned[0] == Approx(dir_ned[0]).margin(1e-12));
    REQUIRE(ex_ned[1] == Approx(dir_ned[1]).margin(1e-12));
    REQUIRE(ex_ned[2] == Approx(dir_ned[2]).margin(1e-12));
}

TEST_CASE("InvertLaunchStateToLocalNED - general configuration",
    "[coordinate][inverse][launch][ned]")
{
    const double lat = 0.7;     // ~40 deg N
    const double lon = -1.0;    // ~-57 deg
    const double h = 2500.0;  // 2.5 km
    const double theta = 0.8;

    const double az = 1.2;
    const double zen = 0.5;
    const double roll = 0.3;

    const LaunchStateECI<double> state =
        MakeLaunchStateECI(lat, lon, h, az, zen, roll, theta);

    const LocalStateNED<double> local =
        InvertLaunchStateToLocalNED(state, theta);

    REQUIRE(local.lat_rad == Approx(lat).margin(1e-9));
    REQUIRE(LonDiff(local.lon_rad, lon) == Approx(0.0).margin(1e-9));
    REQUIRE(local.h_m == Approx(h).margin(1e-3));

    // Direction reference directly in NED (matches forward definition)
    Vec3<double> dir_ned_ref = detail::Normalize(DirectionNEDFromAzimuthZenith(az, zen));
    Vec3<double> dir_ned_rec = detail::Normalize(local.dir_ned);

    REQUIRE(dir_ned_rec[0] == Approx(dir_ned_ref[0]).margin(1e-9));
    REQUIRE(dir_ned_rec[1] == Approx(dir_ned_ref[1]).margin(1e-9));
    REQUIRE(dir_ned_rec[2] == Approx(dir_ned_ref[2]).margin(1e-9));

    const Quat<double>& q_DB = local.q_LB;
    const double qnorm = std::sqrt(q_DB[0] * q_DB[0] + q_DB[1] * q_DB[1] + q_DB[2] * q_DB[2] + q_DB[3] * q_DB[3]);
    REQUIRE(qnorm == Approx(1.0).margin(1e-12));

    Vec3<double> x_b_ned, y_b_ned, z_b_ned;
    AxesFromQuat(q_DB, x_b_ned, y_b_ned, z_b_ned);

    CheckOrthonormalFrame(x_b_ned, y_b_ned, z_b_ned, 1e-10);

    const auto ex_ned = detail::Normalize(x_b_ned);
    REQUIRE(ex_ned[0] == Approx(dir_ned_rec[0]).margin(1e-9));
    REQUIRE(ex_ned[1] == Approx(dir_ned_rec[1]).margin(1e-9));
    REQUIRE(ex_ned[2] == Approx(dir_ned_rec[2]).margin(1e-9));
}
