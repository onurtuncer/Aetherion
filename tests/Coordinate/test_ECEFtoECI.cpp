// -----------------------------------------------------------------------------
// Project: Aetherion
// File:    test_eci_ecef.cpp
// Purpose: Tests for ECI <-> ECEF conversions
// -----------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <numbers>

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Coordinate/InertialToLocal.h>

using Catch::Approx;

using Aetherion::Coordinate::Vec3;
namespace detail = Aetherion::Coordinate::detail;

using Aetherion::Coordinate::ECIToECEF;
using Aetherion::Coordinate::ECEFToECI;

namespace {
    constexpr auto pi = std::numbers::pi_v<double>;

    constexpr Vec3<double> make_vec(double x, double y, double z) {
        return Vec3<double>{x, y, z};
    }
}

// -----------------------------------------------------------------------------
// 1. θ = 0 -> identity mappings
// -----------------------------------------------------------------------------
TEST_CASE("ECI/ECEF - theta = 0 gives identity", "[eci-ecef]")
{
    constexpr auto theta = 0.0;

    constexpr auto v_eci = make_vec(1.0, 2.0, -3.0);
    const auto     v_ecef = ECIToECEF(v_eci, theta);
    const auto     v_eci_back = ECEFToECI(v_ecef, theta);

    REQUIRE(v_ecef[0] == Catch::Approx(v_eci[0]).margin(1e-14));
    REQUIRE(v_ecef[1] == Catch::Approx(v_eci[1]).margin(1e-14));
    REQUIRE(v_ecef[2] == Catch::Approx(v_eci[2]).margin(1e-14));

    REQUIRE(v_eci_back[0] == Catch::Approx(v_eci[0]).margin(1e-14));
    REQUIRE(v_eci_back[1] == Catch::Approx(v_eci[1]).margin(1e-14));
    REQUIRE(v_eci_back[2] == Catch::Approx(v_eci[2]).margin(1e-14));
}

// -----------------------------------------------------------------------------
// 2. θ = π/2: check known rotation of unit axes
//
// Convention assumed:
//
//   v_ecef = R3(theta) * v_eci
//
// where R3(theta) is a rotation about +Z:
//
//   R3(theta) = [  cosθ   sinθ   0
//                 -sinθ   cosθ   0
//                   0      0    1 ]
//
// and ECEFToECI applies the inverse rotation R3(-theta).
// -----------------------------------------------------------------------------
TEST_CASE("ECI/ECEF - axes rotation at 90 degrees", "[eci-ecef]")
{
    constexpr auto theta = pi / 2.0;
    const auto c = std::cos(theta);
    const auto s = std::sin(theta);

    // x-ECI axis -> ECEF
    SECTION("x axis")
    {
        constexpr auto x_eci = make_vec(1.0, 0.0, 0.0);
        const auto     x_ecef = ECIToECEF(x_eci, theta);

        // Expected: [cosθ, -sinθ, 0] = [0, -1, 0] at 90°
        REQUIRE(x_ecef[0] == Catch::Approx(c).margin(1e-14));
        REQUIRE(x_ecef[1] == Catch::Approx(-s).margin(1e-14));
        REQUIRE(x_ecef[2] == Catch::Approx(0.0).margin(1e-14));

        const auto x_eci_back = ECEFToECI(x_ecef, theta);
        REQUIRE(x_eci_back[0] == Catch::Approx(1.0).margin(1e-14));
        REQUIRE(x_eci_back[1] == Catch::Approx(0.0).margin(1e-14));
        REQUIRE(x_eci_back[2] == Catch::Approx(0.0).margin(1e-14));
    }

    // y-ECI axis -> ECEF
    SECTION("y axis")
    {
        constexpr auto y_eci = make_vec(0.0, 1.0, 0.0);
        const auto     y_ecef = ECIToECEF(y_eci, theta);

        // Expected: [sinθ, cosθ, 0] = [1, 0, 0] at 90°
        REQUIRE(y_ecef[0] == Catch::Approx(s).margin(1e-14));
        REQUIRE(y_ecef[1] == Catch::Approx(c).margin(1e-14));
        REQUIRE(y_ecef[2] == Catch::Approx(0.0).margin(1e-14));

        const auto y_eci_back = ECEFToECI(y_ecef, theta);
        REQUIRE(y_eci_back[0] == Catch::Approx(0.0).margin(1e-14));
        REQUIRE(y_eci_back[1] == Catch::Approx(1.0).margin(1e-14));
        REQUIRE(y_eci_back[2] == Catch::Approx(0.0).margin(1e-14));
    }

    // z-ECI axis -> ECEF (unchanged for rotation about z)
    SECTION("z axis")
    {
        constexpr auto z_eci = make_vec(0.0, 0.0, 1.0);
        const auto     z_ecef = ECIToECEF(z_eci, theta);

        REQUIRE(z_ecef[0] == Catch::Approx(0.0).margin(1e-14));
        REQUIRE(z_ecef[1] == Catch::Approx(0.0).margin(1e-14));
        REQUIRE(z_ecef[2] == Catch::Approx(1.0).margin(1e-14));

        const auto z_eci_back = ECEFToECI(z_ecef, theta);
        REQUIRE(z_eci_back[0] == Catch::Approx(0.0).margin(1e-14));
        REQUIRE(z_eci_back[1] == Catch::Approx(0.0).margin(1e-14));
        REQUIRE(z_eci_back[2] == Catch::Approx(1.0).margin(1e-14));
    }
}

// -----------------------------------------------------------------------------
// 3. Round-trip consistency for arbitrary vectors and angles
// -----------------------------------------------------------------------------
TEST_CASE("ECI/ECEF - round trip consistency", "[eci-ecef]")
{
    constexpr auto v_eci = make_vec(1234.5, -6789.0, 42.0);

    constexpr std::array<double, 5> angles{
        0.0,
        pi / 6.0,
        pi / 3.0,
        pi / 2.0,
        pi
    };

    for (auto theta : angles) {
        CAPTURE(theta);

        const auto v_ecef = ECIToECEF(v_eci, theta);
        const auto v_eci_back = ECEFToECI(v_ecef, theta);

        REQUIRE(v_eci_back[0] == Catch::Approx(v_eci[0]).margin(1e-10));
        REQUIRE(v_eci_back[1] == Catch::Approx(v_eci[1]).margin(1e-10));
        REQUIRE(v_eci_back[2] == Catch::Approx(v_eci[2]).margin(1e-10));
    }
}
