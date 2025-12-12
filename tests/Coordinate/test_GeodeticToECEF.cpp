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
//namespace detail = Aetherion::Coordinate::detail;

using Aetherion::Coordinate::GeodeticToECEF;
using Aetherion::Coordinate::DirectionNEDFromAzimuthZenith;
using Aetherion::Coordinate::NEDToECEF;

// ============================================================================
// 1. GeodeticToECEF basic WGS-84 sanity
// ============================================================================

TEST_CASE("GeodeticToECEF - equator prime meridian", "[coordinate][geodetic]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double h = 0.0;

    auto r = GeodeticToECEF(lat, lon, h);

    const double a = 6378137.0;  // WGS-84 semi-major axis

    REQUIRE(r[0] == Approx(a).margin(1e-6));
    REQUIRE(r[1] == Approx(0.0).margin(1e-6));
    REQUIRE(r[2] == Approx(0.0).margin(1e-6));
}

TEST_CASE("GeodeticToECEF - north pole", "[coordinate][geodetic]")
{
    const double lat = std::numbers::pi / 2.0;
    const double lon = 0.0;
    const double h = 0.0;

    auto r = GeodeticToECEF(lat, lon, h);

    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);
    const double b = a * std::sqrt(1.0 - e2);  // semi-minor axis

    REQUIRE(r[0] == Approx(0.0).margin(1e-6));
    REQUIRE(r[1] == Approx(0.0).margin(1e-6));
    REQUIRE(r[2] == Approx(b).margin(1e-3));
}


