// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <cppad/cppad.hpp>

#include <Aetherion/Aerodynamics/MachNumber.h>

using Catch::Approx;

namespace Aero = Aetherion::Aerodynamics;

TEST_CASE("SmoothAbs approximates abs away from zero", "[Aerodynamics][Mach]")
{
    const double eps = 1e-9;

    const double x = 3.0;
    const double y = Aero::SmoothAbs(x, eps);

    REQUIRE(y == Approx(3.0).margin(1e-12));
}

TEST_CASE("SmoothMax0 behaves like max(x,0) (smoothly)", "[Aerodynamics][Mach]")
{
    const double eps = 1e-9;

    // Positive input -> ~x
    {
        const double x = 2.0;
        const double y = Aero::SmoothMax0(x, eps);
        REQUIRE(y == Approx(2.0).margin(1e-12));
    }

    // Negative input -> ~0 (not exactly 0 due to smoothing)
    {
        const double x = -2.0;
        const double y = Aero::SmoothMax0(x, eps);
        REQUIRE(y >= 0.0);
        REQUIRE(y < 1e-6);
    }
}

TEST_CASE("SpeedFromVelocity - 3-4-0 triangle", "[Aerodynamics][Mach]")
{
    const Aero::Vec3<double> v{ 3.0, 4.0, 0.0 };
    const double speed = Aero::SpeedFromVelocity(v);

    REQUIRE(speed == Approx(5.0).margin(1e-12));
}

TEST_CASE("SpeedOfSoundFromTemperature matches sqrt(gamma*R*T) for positive T", "[Aerodynamics][Mach]")
{
    const double gamma = 1.4;
    const double R = 287.05;
    const double T = 288.15;

    const double a = Aero::SpeedOfSoundFromTemperature(gamma, R, T);

    const double expected = std::sqrt(gamma * R * T);
    REQUIRE(a == Approx(expected).margin(1e-12));
}

TEST_CASE("SpeedOfSoundFromTemperature stays finite for negative T (smooth clamp)", "[Aerodynamics][Mach]")
{
    const double gamma = 1.4;
    const double R = 287.05;
    const double T = -10.0;

    const double a = Aero::SpeedOfSoundFromTemperature(gamma, R, T);

    REQUIRE(std::isfinite(a));
    REQUIRE(a >= 0.0);
}

TEST_CASE("SpeedOfSoundFromPressureDensity matches sqrt(gamma*p/rho) for nominal sea-level", "[Aerodynamics][Mach]")
{
    const double gamma = 1.4;
    const double p = 101325.0;
    const double rho = 1.225;

    const double a = Aero::SpeedOfSoundFromPressureDensity(gamma, p, rho);

    const double expected = std::sqrt(gamma * p / rho);
    REQUIRE(a == Approx(expected).margin(1e-12));
}


TEST_CASE("CppAD derivative: SpeedFromVelocity gradients match v_i/speed", "[Aerodynamics][Mach][CppAD]")
{
    using AD = CppAD::AD<double>;

    const double eps = 1e-12;

    std::vector<AD> x(3);
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;

    CppAD::Independent(x);

    Aero::Vec3<AD> v{ x[0], x[1], x[2] };
    AD y = Aero::SpeedFromVelocity(v, AD(eps));

    std::vector<AD> Y(1);
    Y[0] = y;

    CppAD::ADFun<double> f(x, Y);

    const std::vector<double> xin{ 3.0, 4.0, 0.0 };
    const std::vector<double> jac = f.Jacobian(xin);

    const double speed = std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + eps * eps);

    REQUIRE(jac.size() == 3);
    REQUIRE(jac[0] == Approx(3.0 / speed).margin(1e-12));
    REQUIRE(jac[1] == Approx(4.0 / speed).margin(1e-12));
    REQUIRE(jac[2] == Approx(0.0).margin(1e-12));
}


