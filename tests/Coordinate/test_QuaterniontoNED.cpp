// tests/test_local_orientation_ned.cpp

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <array>
#include <numbers>

#include "Aetherion/Coordinate/QuaternionToLocalNED.h"  

using Catch::Approx;

using Aetherion::Coordinate::Quat;
using Aetherion::Coordinate::LocalOrientationNED;
using Aetherion::Coordinate::QuaternionToAzZenRollNED;

namespace {

    constexpr double pi = std::numbers::pi;

    // Simple axis-angle -> quaternion helper for tests.
    // Axis is in inertial/NED frame, quaternion is [w,x,y,z] rotating body->inertial.
    Quat<double> AxisAngleToQuat(const std::array<double, 3>& axis_in, double angle)
    {
        double ax = axis_in[0];
        double ay = axis_in[1];
        double az = axis_in[2];

        const double n2 = ax * ax + ay * ay + az * az;
        REQUIRE(n2 > 0.0);

        const double inv_n = 1.0 / std::sqrt(n2);
        ax *= inv_n;
        ay *= inv_n;
        az *= inv_n;

        const double half = 0.5 * angle;
        const double c = std::cos(half);
        const double s = std::sin(half);

        return Quat<double>{ c, s* ax, s* ay, s* az };
    }

} // namespace

// -----------------------------------------------------------------------------
// Test 1: Identity attitude in inertial == NED
// -----------------------------------------------------------------------------
TEST_CASE("LocalOrientationNED - identity attitude", "[orientation][ned]")
{
    // Inertial frame is identical to NED for this test.
    const Quat<double> q_body_to_inertial{ 1.0, 0.0, 0.0, 0.0 }; // body aligned with NED
    const Quat<double> q_ned_to_inertial{ 1.0, 0.0, 0.0, 0.0 };  // NED aligned with inertial

    auto local = QuaternionToAzZenRollNED(q_body_to_inertial, q_ned_to_inertial);

    // Forward is N: azimuth = 0, zenith = pi/2, roll = 0.
    REQUIRE(local.azimuth == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(local.zenith == Catch::Approx(pi / 2.0).margin(1e-12));
    REQUIRE(local.roll == Catch::Approx(0.0).margin(1e-12));
}

// -----------------------------------------------------------------------------
// Test 2: Yaw 90 degrees to the right (forward points East), level, no roll
// -----------------------------------------------------------------------------
TEST_CASE("LocalOrientationNED - yaw 90 deg East, level", "[orientation][ned]")
{
    // Inertial frame is NED.
    const Quat<double> q_ned_to_inertial{ 1.0, 0.0, 0.0, 0.0 };

    // Rotate body about Down axis (z = D) by +90 deg.
    // After rotation: forward = East.
    const double yaw = pi / 2.0;
    const std::array<double, 3> axis_down{ 0.0, 0.0, 1.0 }; // NED z-axis (Down)
    const Quat<double> q_body_to_inertial = AxisAngleToQuat(axis_down, yaw);

    auto local = QuaternionToAzZenRollNED(q_body_to_inertial, q_ned_to_inertial);

    // Forward pointing East:
    //  azimuth = +pi/2, zenith = pi/2 (still in horizontal plane), roll = 0.
    REQUIRE(local.azimuth == Catch::Approx(pi / 2.0).margin(1e-12));
    REQUIRE(local.zenith == Catch::Approx(pi / 2.0).margin(1e-12));
    REQUIRE(local.roll == Catch::Approx(0.0).margin(1e-12));
}

// -----------------------------------------------------------------------------
// Test 3: Pure roll about forward axis (x_B) with no yaw/pitch
// -----------------------------------------------------------------------------
TEST_CASE("LocalOrientationNED - pure roll", "[orientation][ned]")
{
    // Inertial frame is NED.
    const Quat<double> q_ned_to_inertial{ 1.0, 0.0, 0.0, 0.0 };

    // Base attitude: body aligned with NED.
    // Apply roll about body x_B which is aligned with N (North).
    const double roll_cmd = 30.0 * pi / 180.0; // 30 degrees
    const std::array<double, 3> axis_forward{ 1.0, 0.0, 0.0 }; // NED x-axis (North)
    const Quat<double> q_body_to_inertial = AxisAngleToQuat(axis_forward, roll_cmd);

    auto local = QuaternionToAzZenRollNED(q_body_to_inertial, q_ned_to_inertial);

    // Forward axis is still N: azimuth = 0, zenith = pi/2.
    REQUIRE(local.azimuth == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(local.zenith == Catch::Approx(pi / 2.0).margin(1e-12));

    // Roll should be approximately 30 deg (within sign convention).
    REQUIRE(local.roll == Catch::Approx(roll_cmd).margin(1e-12));
}

// -----------------------------------------------------------------------------
// Test 4: Pitch up (forward tilts toward Up) without roll
// -----------------------------------------------------------------------------
TEST_CASE("LocalOrientationNED - pitch up", "[orientation][ned]")
{
    // Inertial frame is NED.
    const Quat<double> q_ned_to_inertial{ 1.0, 0.0, 0.0, 0.0 };

    // Pitch about E (y = East) axis by +30 degrees.
    // Forward tilts toward Up (negative D).
    const double pitch = 30.0 * pi / 180.0;
    const std::array<double, 3> axis_east{ 0.0, 1.0, 0.0 }; // NED y-axis (East)
    const Quat<double> q_body_to_inertial = AxisAngleToQuat(axis_east, pitch);

    auto local = QuaternionToAzZenRollNED(q_body_to_inertial, q_ned_to_inertial);

    // Azimuth remains 0 (still in N–Up plane).
    REQUIRE(local.azimuth == Catch::Approx(0.0).margin(1e-12));

    // Forward is tilted upward: zenith < pi/2.
    // The exact relation for pitch θ about East: zenith = pi/2 - θ.
    const double expected_zenith = pi / 2.0 - pitch;
    REQUIRE(local.zenith == Catch::Approx(expected_zenith).margin(1e-12));

    // No roll commanded.
    REQUIRE(local.roll == Catch::Approx(0.0).margin(1e-12));
}
