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

namespace detail = Aetherion::Coordinate::detail;

// We are NED now:
using Aetherion::Coordinate::ECIToNED;

// -----------------------------------------------------------------------------
// 1) Known conditions (NED)
// -----------------------------------------------------------------------------

TEST_CASE("ECIToNED - equator, up direction", "[coordinate][inverse][NED]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double theta = 0.0;

    // At lat=0, lon=0:
    // Up in ECEF is +X. With theta=0, ECI == ECEF.
    const Vec3<double> v_eci{ 1.0, 0.0, 0.0 };

    const Vec3<double> v_ned = ECIToNED(v_eci, lat, lon, theta);

    // In NED: "Up" corresponds to D = -1 (since +D is Down).
    REQUIRE(v_ned[0] == Approx(0.0).margin(1e-14));  // N
    REQUIRE(v_ned[1] == Approx(0.0).margin(1e-14));  // E
    REQUIRE(v_ned[2] == Approx(-1.0).margin(1e-14)); // D
}

// -----------------------------------------------------------------------------
// Known conditions (NED) at equator, lon=0, theta=0
// -----------------------------------------------------------------------------

TEST_CASE("ECIToNED - equator, north direction", "[coordinate][inverse][NED]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double theta = 0.0;

    // At lat=0, lon=0:
    // N basis in ECEF is +Z, and theta=0 => ECI==ECEF.
    const Vec3<double> v_eci{ 0.0, 0.0, 1.0 };

    const Vec3<double> v_ned = ECIToNED(v_eci, lat, lon, theta);

    REQUIRE(v_ned[0] == Approx(1.0).margin(1e-14)); // N
    REQUIRE(v_ned[1] == Approx(0.0).margin(1e-14)); // E
    REQUIRE(v_ned[2] == Approx(0.0).margin(1e-14)); // D
}

TEST_CASE("ECIToNED - equator, east direction", "[coordinate][inverse][NED]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double theta = 0.0;

    // At lat=0, lon=0:
    // E basis in ECEF is +Y, and theta=0 => ECI==ECEF.
    const Vec3<double> v_eci{ 0.0, 1.0, 0.0 };

    const Vec3<double> v_ned = ECIToNED(v_eci, lat, lon, theta);

    REQUIRE(v_ned[0] == Approx(0.0).margin(1e-14)); // N
    REQUIRE(v_ned[1] == Approx(1.0).margin(1e-14)); // E
    REQUIRE(v_ned[2] == Approx(0.0).margin(1e-14)); // D
}


