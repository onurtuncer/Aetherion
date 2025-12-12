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
#include <random>

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Coordinate/InertialToLocal.h>

using Catch::Approx;

using Aetherion::Coordinate::Vec3;
using Aetherion::Coordinate::Quat;
using Aetherion::Coordinate::Mat3;

namespace detail = Aetherion::Coordinate::detail;

using Aetherion::Coordinate::MakeLaunchStateECI;
using Aetherion::Coordinate::LaunchStateECI;

using Aetherion::Coordinate::DirectionNEDFromAzimuthZenith;

// NED versions (must exist in your updated InertialToLocal.h)
using Aetherion::Coordinate::ECIToNED;
using Aetherion::Coordinate::LocalStateNED;
using Aetherion::Coordinate::InvertLaunchStateToLocalNED;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static double Norm(const Vec3<double>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Wrap longitude difference into [-pi, pi] for comparison
static double LonDiff(double lon_rec, double lon_ref) {
    const double two_pi = 2.0 * std::numbers::pi;
    return std::remainder(lon_rec - lon_ref, two_pi);
}

// Extract body axes in frame "A" from q_AB (body->A): columns of R(q_AB)
static void AxesFromQuat(
    const Quat<double>& q_AB,
    Vec3<double>& x_b_A,
    Vec3<double>& y_b_A,
    Vec3<double>& z_b_A)
{
    const Mat3<double> R = detail::QuaternionToRotationMatrix(q_AB);
    // columns (row-major 3x3)
    x_b_A = Vec3<double>{ R[0], R[3], R[6] };
    y_b_A = Vec3<double>{ R[1], R[4], R[7] };
    z_b_A = Vec3<double>{ R[2], R[5], R[8] };
}

// Orthonormality check of a rotation frame (columns)
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
// Random round-trip tests (NED) (position + direction + orientation consistency)
// -----------------------------------------------------------------------------

TEST_CASE("LaunchStateECI <-> LocalStateNED random round-trip",
    "[coordinate][inverse][launch][random][ned]")
{
    std::mt19937 rng(1019407378u); // fixed seed

    // Avoid poles and the exact zenith singularities
    std::uniform_real_distribution<double> lat_dist(-1.2, 1.2); // ~[-69°, 69°]
    std::uniform_real_distribution<double> lon_dist(-std::numbers::pi, std::numbers::pi);
    std::uniform_real_distribution<double> h_dist(0.0, 100000.0); // 0..100 km
    std::uniform_real_distribution<double> az_dist(0.0, 2.0 * std::numbers::pi);
    std::uniform_real_distribution<double> zen_dist(0.05, 1.4);
    std::uniform_real_distribution<double> roll_dist(-std::numbers::pi, std::numbers::pi);
    std::uniform_real_distribution<double> theta_dist(0.0, 2.0 * std::numbers::pi);

    const int N = 50;

    for (int i = 0; i < N; ++i) {
        const double lat = lat_dist(rng);
        const double lon = lon_dist(rng);
        const double h = h_dist(rng);
        const double az = az_dist(rng);
        const double zen = zen_dist(rng);
        const double roll = roll_dist(rng);
        const double theta = theta_dist(rng);

        const LaunchStateECI<double> state =
            MakeLaunchStateECI(lat, lon, h, az, zen, roll, theta);

        const LocalStateNED<double> local =
            InvertLaunchStateToLocalNED(state, theta);

        // Geodetic round-trip
        CHECK(local.lat_rad - lat == Approx(0.0).margin(1e-7));
        CHECK(LonDiff(local.lon_rad, lon) == Approx(0.0).margin(1e-7));
        CHECK(local.h_m - h == Approx(0.0).margin(1e-2)); // ~cm

        // Direction round-trip in NED
        const Vec3<double> dir_ned_ref = detail::Normalize(DirectionNEDFromAzimuthZenith(az, zen));
        const Vec3<double> dir_ned_rec = detail::Normalize(local.dir_ned);

        const double delta_dir = Norm(Vec3<double>{
            dir_ned_rec[0] - dir_ned_ref[0],
                dir_ned_rec[1] - dir_ned_ref[1],
                dir_ned_rec[2] - dir_ned_ref[2]
        });
        CHECK(delta_dir == Approx(0.0).margin(1e-7));

        // Orientation consistency: body +x from q_LB matches dir_ned
        const Quat<double>& q_LB = local.q_LB;
        const double qnorm = std::sqrt(
            q_LB[0] * q_LB[0] + q_LB[1] * q_LB[1] + q_LB[2] * q_LB[2] + q_LB[3] * q_LB[3]);
        CHECK(qnorm == Approx(1.0).margin(1e-10));

        Vec3<double> x_b_ned, y_b_ned, z_b_ned;
        AxesFromQuat(q_LB, x_b_ned, y_b_ned, z_b_ned);

        CheckOrthonormalFrame(x_b_ned, y_b_ned, z_b_ned, 1e-9);

        const auto ex_ned = detail::Normalize(x_b_ned);
        CHECK(ex_ned[0] == Approx(dir_ned_rec[0]).margin(1e-7));
        CHECK(ex_ned[1] == Approx(dir_ned_rec[1]).margin(1e-7));
        CHECK(ex_ned[2] == Approx(dir_ned_rec[2]).margin(1e-7));
    }
}

