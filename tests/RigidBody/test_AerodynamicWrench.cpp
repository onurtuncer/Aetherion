// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cmath>

#include "Aetherion/RigidBody/GravitationalWrench.h" // adjust include path

namespace {

    template<class S>
    using Vec3E = Eigen::Matrix<S, 3, 1>;

    constexpr double kMu_WGS84 = 3.986004418e14; // [m^3/s^2]
    constexpr double kRe_WGS84 = 6378137.0;      // [m]
    constexpr double kJ2_WGS84 = 1.08262668e-3;  // [-]

    inline double g_surface_wgs84()
    {
        // For CentralGravity at r = Re, magnitude is mu / r^2
        return kMu_WGS84 / (kRe_WGS84 * kRe_WGS84); // ~ 9.798285479...
    }

} // namespace

TEST_CASE("GravitationalWrenchAtCG: central gravity at equator gives expected force and zero moment", "[gravity][wrench]")
{
    using Scalar = double;

    const Scalar m = 2.0; // [kg]
    const Scalar g = g_surface_wgs84();

    // Position at equator on +x axis in frame W
    const Aetherion::Environment::Vec3<Scalar> r_W{ Scalar(kRe_WGS84), Scalar(0), Scalar(0) };

    const auto w = Aetherion::RigidBody::GravitationalWrenchAtCG(r_W, m, Scalar(kMu_WGS84));

    const Vec3E<Scalar> M = w.f.template segment<3>(0);
    const Vec3E<Scalar> F = w.f.template segment<3>(3);

    // Moment should be exactly zero (we set it to zero explicitly)
    CHECK(M(0) == Scalar(0));
    CHECK(M(1) == Scalar(0));
    CHECK(M(2) == Scalar(0));

    // Force should be m * g_W, and g_W should be [-g, 0, 0] at r = [Re,0,0]
    using Catch::Matchers::WithinAbs;
    CHECK_THAT(F(0), WithinAbs(-m * g, 1e-9));
    CHECK_THAT(F(1), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(F(2), WithinAbs(Scalar(0), 1e-12));
}

TEST_CASE("GravitationalWrenchJ2AtCG: J2 changes equatorial gravity (more negative x) and keeps z=0", "[gravity][wrench][J2]")
{
    using Scalar = double;

    const Scalar m = 1.0; // [kg]
    const Aetherion::Environment::Vec3<Scalar> r_W{ Scalar(kRe_WGS84), Scalar(0), Scalar(0) };

    const auto w_c = Aetherion::RigidBody::GravitationalWrenchAtCG(r_W, m, Scalar(kMu_WGS84));
    const auto w_j2 = Aetherion::RigidBody::GravitationalWrenchJ2AtCG(
        r_W, m, Scalar(kMu_WGS84), Scalar(kRe_WGS84), Scalar(kJ2_WGS84));

    const Vec3E<Scalar> F_c = w_c.f.template segment<3>(3);
    const Vec3E<Scalar> F_j2 = w_j2.f.template segment<3>(3);

    // At equator (z=0), both should have ~0 z component
    using Catch::Matchers::WithinAbs;
    CHECK_THAT(F_c(2), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(F_j2(2), WithinAbs(Scalar(0), 1e-12));

    // J2 should perturb the x-force at equator (your model adds a non-zero J2 term)
    // and for z=0 the perturbation makes gx "more negative" (larger magnitude).
    REQUIRE(F_c(0) < Scalar(0));
    REQUIRE(F_j2(0) < Scalar(0));
    CHECK(std::abs(F_j2(0)) > std::abs(F_c(0)));
}

TEST_CASE("GravitationalWrenchWithOffset: moment equals r x F", "[gravity][wrench][moment]")
{
    using Scalar = double;

    const Scalar m = 3.0; // [kg]
    const Scalar g = g_surface_wgs84();
    const Aetherion::Environment::Vec3<Scalar> r_W{ Scalar(kRe_WGS84), Scalar(0), Scalar(0) };

    // Apply gravitational force at an offset of +1m along +z in frame W
    // r = [0,0,1], F ≈ [-m*g,0,0] => M = r x F = [0, -m*g, 0]
    const Vec3E<Scalar> r_app_minus_cg_W(Scalar(0), Scalar(0), Scalar(1));

    const auto w = Aetherion::RigidBody::GravitationalWrenchWithOffset(
        r_W, m, r_app_minus_cg_W, Scalar(kMu_WGS84));

    const Vec3E<Scalar> M = w.f.template segment<3>(0);
    const Vec3E<Scalar> F = w.f.template segment<3>(3);

    using Catch::Matchers::WithinAbs;

    CHECK_THAT(F(0), WithinAbs(-m * g, 1e-9));
    CHECK_THAT(F(1), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(F(2), WithinAbs(Scalar(0), 1e-12));

    CHECK_THAT(M(0), WithinAbs(Scalar(0), 1e-12));
    CHECK_THAT(M(1), WithinAbs(-m * g, 1e-9));
    CHECK_THAT(M(2), WithinAbs(Scalar(0), 1e-12));
}