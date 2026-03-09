// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_GravitationalWrench.cpp
//
// Catch2 tests for Aetherion/RigidBody/GravitationalWrench.h
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <array>
#include <cmath>

#include "Aetherion/RigidBody/GravitationalWrench.h" 

namespace {

    template<class S>
    using Arr3 = std::array<S, 3>;

    template<class S>
    using Vec3E = Eigen::Matrix<S, 3, 1>;

    constexpr double kMu_WGS84 = 3.986004418e14; // [m^3/s^2]
    constexpr double kRe_WGS84 = 6378137.0;      // [m]
    constexpr double kJ2_WGS84 = 1.08262668e-3;  // [-]

    inline double g_surface_from_mu_r()
    {
        // CentralGravity magnitude at r = Re: mu / r^2
        return kMu_WGS84 / (kRe_WGS84 * kRe_WGS84);
    }

} // namespace

TEST_CASE("GravitationalWrenchAtCG: central gravity at equator gives expected force and zero moment", "[gravity][wrench][equator]")
{
    using Scalar = double;

    const Scalar m = Scalar(2.0); // [kg]
    const Scalar g = Scalar(g_surface_from_mu_r());

    // Position at equator on +x axis (frame W)
    const Arr3<Scalar> r_W{ Scalar(kRe_WGS84), Scalar(0), Scalar(0) };

    const auto w = Aetherion::RigidBody::GravitationalWrenchAtCG(
        r_W, m, Scalar(kMu_WGS84));

    const Vec3E<Scalar> M = w.f.template segment<3>(0);
    const Vec3E<Scalar> F = w.f.template segment<3>(3);

    CHECK(M(0) == Scalar(0));
    CHECK(M(1) == Scalar(0));
    CHECK(M(2) == Scalar(0));

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(F(0), WithinAbs(-m * g, 1e-9)); // along -x
    CHECK_THAT(F(1), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(F(2), WithinAbs(Scalar(0), 1e-12));
}

TEST_CASE("GravitationalWrenchJ2AtCG: J2 perturbs equatorial gravity and keeps z=0", "[gravity][wrench][J2]")
{
    using Scalar = double;

    const Scalar m = Scalar(1.0); // [kg]
    const Arr3<Scalar> r_W{ Scalar(kRe_WGS84), Scalar(0), Scalar(0) }; // equator => z=0

    const auto w_c = Aetherion::RigidBody::GravitationalWrenchAtCG(
        r_W, m, Scalar(kMu_WGS84));

    const auto w_j2 = Aetherion::RigidBody::GravitationalWrenchJ2AtCG(
        r_W, m, Scalar(kMu_WGS84), Scalar(kRe_WGS84), Scalar(kJ2_WGS84));

    const Vec3E<Scalar> F_c = w_c.f.template segment<3>(3);
    const Vec3E<Scalar> F_j2 = w_j2.f.template segment<3>(3);

    using Catch::Matchers::WithinAbs;

    // z component should remain ~0 at equator for both models
    CHECK_THAT(F_c(2), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(F_j2(2), WithinAbs(Scalar(0), 1e-12));

    // J2 should change x-component from the central value at equator (cd J2 model does)
    REQUIRE(F_c(0) < Scalar(0));
    REQUIRE(F_j2(0) < Scalar(0));
    CHECK(std::abs(F_j2(0)) > std::abs(F_c(0)));
}

TEST_CASE("GravitationalWrenchWithOffset: moment equals r x F", "[gravity][wrench][offset]")
{
    using Scalar = double;

    const Scalar m = Scalar(3.0); // [kg]
    const Scalar g = Scalar(g_surface_from_mu_r());

    const Arr3<Scalar> r_W{ Scalar(kRe_WGS84), Scalar(0), Scalar(0) };

    // Apply gravitational force at +1m along +z (frame W)
    // r = [0,0,1], F ≈ [-m*g,0,0] => M = r x F = [0, +m*g, 0]
    const Vec3E<Scalar> r_app_minus_cg_W_m(Scalar(0), Scalar(0), Scalar(1));

    const auto w = Aetherion::RigidBody::GravitationalWrenchWithOffset(
        r_W, m, r_app_minus_cg_W_m, Scalar(kMu_WGS84));

    const Vec3E<Scalar> M = w.f.template segment<3>(0);
    const Vec3E<Scalar> F = w.f.template segment<3>(3);

    using Catch::Matchers::WithinAbs;

    CHECK_THAT(F(0), WithinAbs(-m * g, 1e-9));
    CHECK_THAT(F(1), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(F(2), WithinAbs(Scalar(0), 1e-12));

    CHECK_THAT(M(0), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(M(1), WithinAbs(m * g, 1e-9));
    CHECK_THAT(M(2), WithinAbs(Scalar(0), 1e-12));
}