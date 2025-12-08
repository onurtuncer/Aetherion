// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------


#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <numbers>
#include <random>

#include "Aetherion/Coordinate/FrameConversions.h"
#include "Aetherion/Coordinate/InertialToLocal.h"

using Catch::Approx;

using Aetherion::Coordinate::Vec3;
using Aetherion::Coordinate::Quat;
using Aetherion::Coordinate::GeodeticToECEF;
using Aetherion::Coordinate::ECEFToGeodeticWGS84;
using Aetherion::Coordinate::ECEFToNEU;
using Aetherion::Coordinate::ECIToNEU;
using Aetherion::Coordinate::MakeLaunchStateECI;
using Aetherion::Coordinate::LaunchStateECI;
using Aetherion::Coordinate::LocalStateNEU;
using Aetherion::Coordinate::InvertLaunchStateToLocalNEU;
using Aetherion::Coordinate::DirectionNEUFromAzimuthZenith;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

inline static double Norm(const Vec3<double>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline static double Dot(const Vec3<double>& a, const Vec3<double>& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline static Vec3<double> Normalize(const Vec3<double>& v)
{
    const double n = Norm(v);
    return Vec3<double>{ v[0] / n, v[1] / n, v[2] / n };
}

// Wrap longitude difference into [-pi, pi] for comparison
inline static double LonDiff(double lon_rec, double lon_ref)
{
    const double two_pi = 2.0 * std::numbers::pi;
    double d = lon_rec - lon_ref;
    d = std::remainder(d, two_pi); // symmetric remainder
    return d;
}

// -----------------------------------------------------------------------------
// 1. Known conditions
// -----------------------------------------------------------------------------

TEST_CASE("ECEFToGeodeticWGS84 - round-trip mid-latitude",
    "[coordinate][inverse][geodetic]")
{
    const double lat0 = 0.6;        // ~34.4 deg
    const double lon0 = 1.0;        // ~57.3 deg
    const double h0 = 1000.0;     // 1 km

    // Forward: Geodetic -> ECEF
    Vec3<double> r_ecef = GeodeticToECEF(lat0, lon0, h0);

    // Inverse: ECEF -> Geodetic
    double lat, lon, h;
    ECEFToGeodeticWGS84(r_ecef, lat, lon, h);

    REQUIRE(lat == Approx(lat0).epsilon(1e-10));
    REQUIRE(LonDiff(lon, lon0) == Approx(0.0).margin(1e-10));
    REQUIRE(h == Approx(h0).margin(1e-3));
}

TEST_CASE("ECIToNEU - equator, up direction",
    "[coordinate][inverse][NEU]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double theta = 0.0; // Earth rotation angle

    // At lat=0, lon=0, Up corresponds to ECEF/ECI [1,0,0] in our setup.
    Vec3<double> v_eci{ 1.0, 0.0, 0.0 };

    Vec3<double> v_neu = ECIToNEU(v_eci, lat, lon, theta);

    REQUIRE(v_neu[0] == Approx(0.0).margin(1e-14)); // N
    REQUIRE(v_neu[1] == Approx(0.0).margin(1e-14)); // E
    REQUIRE(v_neu[2] == Approx(1.0).margin(1e-14)); // U
}

// -----------------------------------------------------------------------------
// 2. Forward + inverse path for simple / general configurations
// -----------------------------------------------------------------------------

TEST_CASE("InvertLaunchStateToLocalNEU - equator straight up, zero roll",
    "[coordinate][inverse][launch]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double h = 0.0;
    const double theta = 0.0;

    const double az = 0.0;
    const double zen = 0.0; // Up
    const double roll = 0.0;

    // Forward: build launch state in ECI
    LaunchStateECI<double> state =
        MakeLaunchStateECI(lat, lon, h, az, zen, roll, theta);

    // Inverse: recover local NEU / WGS-84
    LocalStateNEU<double> local =
        InvertLaunchStateToLocalNEU(state, theta);

    // Position must come back
    REQUIRE(local.lat_rad == Approx(lat).epsilon(1e-12));
    REQUIRE(LonDiff(local.lon_rad, lon) == Approx(0.0).margin(1e-12));
    REQUIRE(local.h_m == Approx(h).margin(1e-6));

    // Direction in NEU must be Up
    // (We know forward dir0_eci is along local Up for this configuration.)
    REQUIRE(local.dir_neu[0] == Approx(0.0).margin(1e-12)); // N
    REQUIRE(local.dir_neu[1] == Approx(0.0).margin(1e-12)); // E
    REQUIRE(local.dir_neu[2] == Approx(1.0).margin(1e-12)); // U

    // Sanity: quaternion is unit (we don't enforce a specific gauge in this degenerate case)
    const Quat<double>& q_NB = local.q_NB;
    const double qnorm = std::sqrt(q_NB[0] * q_NB[0] + q_NB[1] * q_NB[1] +
        q_NB[2] * q_NB[2] + q_NB[3] * q_NB[3]);
    REQUIRE(qnorm == Approx(1.0).epsilon(1e-12));
}

TEST_CASE("InvertLaunchStateToLocalNEU - general configuration",
    "[coordinate][inverse][launch]")
{
    const double lat = 0.7;          // ~40 deg N
    const double lon = -1.0;         // ~-57 deg
    const double h = 2500.0;       // 2.5 km
    const double theta = 0.8;

    const double az = 1.2;
    const double zen = 0.5;
    const double roll = 0.3;

    // Forward
    LaunchStateECI<double> state =
        MakeLaunchStateECI(lat, lon, h, az, zen, roll, theta);

    // Inverse
    LocalStateNEU<double> local =
        InvertLaunchStateToLocalNEU(state, theta);

    // Geodetic parameters should come back
    REQUIRE(local.lat_rad == Approx(lat).epsilon(1e-9));
    REQUIRE(LonDiff(local.lon_rad, lon) == Approx(0.0).margin(1e-9));
    REQUIRE(local.h_m == Approx(h).margin(1e-3));

    // NEU direction should match original NEU direction from azimuth/zenith
    Vec3<double> dir_neu_ref =
        DirectionNEUFromAzimuthZenith(az, zen);
    dir_neu_ref = Normalize(dir_neu_ref);

    Vec3<double> dir_neu_rec = Normalize(local.dir_neu);

    REQUIRE(dir_neu_rec[0] == Approx(dir_neu_ref[0]).epsilon(1e-9));
    REQUIRE(dir_neu_rec[1] == Approx(dir_neu_ref[1]).epsilon(1e-9));
    REQUIRE(dir_neu_rec[2] == Approx(dir_neu_ref[2]).epsilon(1e-9));

    // Sanity: q_NB is unit
    const Quat<double>& q_NB = local.q_NB;
    const double qnorm = std::sqrt(q_NB[0] * q_NB[0] + q_NB[1] * q_NB[1] +
        q_NB[2] * q_NB[2] + q_NB[3] * q_NB[3]);
    REQUIRE(qnorm == Approx(1.0).epsilon(1e-12));
}

// -----------------------------------------------------------------------------
// 3. Random round-trip tests to build confidence
// -----------------------------------------------------------------------------

TEST_CASE("LaunchStateECI <-> LocalStateNEU random round-trip",
    "[coordinate][inverse][launch][random]")
{
    std::mt19937 rng(120819378u); // fixed seed for reproducibility

    // Avoid extreme singularities at poles & zenith = 0 / pi/2 exactly
    std::uniform_real_distribution<double> lat_dist(-1.2, 1.2);          // ~[-69°, 69°]
    std::uniform_real_distribution<double> lon_dist(-std::numbers::pi,
        std::numbers::pi);
    std::uniform_real_distribution<double> h_dist(0.0, 100000.0);        // 0..100 km
    std::uniform_real_distribution<double> az_dist(0.0, 2.0 * std::numbers::pi);
    std::uniform_real_distribution<double> zen_dist(0.05, 1.4);          // avoid exactly 0 and horizon
    std::uniform_real_distribution<double> roll_dist(-std::numbers::pi,
        std::numbers::pi);
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

        LaunchStateECI<double> state =
            MakeLaunchStateECI(lat, lon, h, az, zen, roll, theta);

        LocalStateNEU<double> local =
            InvertLaunchStateToLocalNEU(state, theta);

        // Geodetic round-trip
        const double dlat = local.lat_rad - lat;
        const double dlon = LonDiff(local.lon_rad, lon);
        const double dh = local.h_m - h;

        CHECK(dlat == Approx(0.0).margin(1e-7));      // ~1e-7 rad ~ 6e-6 deg
        CHECK(dlon == Approx(0.0).margin(1e-7));
        CHECK(dh == Approx(0.0).margin(1e-2));      // centimeter-level

        // Direction round-trip: NEU direction
        Vec3<double> dir_neu_ref =
            Normalize(DirectionNEUFromAzimuthZenith(az, zen));
        Vec3<double> dir_neu_rec =
            Normalize(local.dir_neu);

        const double delta_dir = Norm(Vec3<double>{
            dir_neu_rec[0] - dir_neu_ref[0],
                dir_neu_rec[1] - dir_neu_ref[1],
                dir_neu_rec[2] - dir_neu_ref[2]
        });

        CHECK(delta_dir == Approx(0.0).margin(1e-7));

        // Also check q_NB is unit in the random cases
        const Quat<double>& q_NB = local.q_NB;
        const double qnorm = std::sqrt(q_NB[0] * q_NB[0] + q_NB[1] * q_NB[1] +
            q_NB[2] * q_NB[2] + q_NB[3] * q_NB[3]);
        CHECK(qnorm == Approx(1.0).epsilon(1e-10));
    }
}
