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

// ============================================================================
// Direction NED -> ECEF -> ECI sanity
// ============================================================================

TEST_CASE("DirectionNEDFromAzimuthZenith - straight up at equator", "[coordinate][direction]")
{
    const double lat = 0.0;
    const double lon = 0.0;
    const double theta = 0.0;   // Earth rotation angle
    const double az = 0.0;   // azimuth doesn't matter for zenith=0
    const double zen = 0.0;   // zenith = 0 => along Up, which is D = -1 in NED

    Vec3<double> dir_ned = DirectionNEDFromAzimuthZenith(az, zen);
    REQUIRE(dir_ned[0] == Approx(0.0).margin(1e-14));  // N
    REQUIRE(dir_ned[1] == Approx(0.0).margin(1e-14));  // E
    REQUIRE(dir_ned[2] == Approx(-1.0).margin(1e-14)); // D (Down; Up corresponds to D = -1)

    Vec3<double> dir_ecef = NEDToECEF(dir_ned, lat, lon);
    // At lat=0, lon=0: Up = [1,0,0] in ECEF, so NED [0,0,-1] must map to [1,0,0].
    REQUIRE(dir_ecef[0] == Approx(1.0).margin(1e-14));
    REQUIRE(dir_ecef[1] == Approx(0.0).margin(1e-14));
    REQUIRE(dir_ecef[2] == Approx(0.0).margin(1e-14));

    Vec3<double> dir_eci = ECEFToECI(dir_ecef, theta); // theta=0 => identity
    REQUIRE(dir_eci[0] == Approx(1.0).margin(1e-14));
    REQUIRE(dir_eci[1] == Approx(0.0).margin(1e-14));
    REQUIRE(dir_eci[2] == Approx(0.0).margin(1e-14));
}