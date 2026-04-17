// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_GravityPolicies.cpp
//
// Catch2 tests for Aetherion/FlightDynamics/Policies/GravityPolicies.h
//
// Policies under test
//   ZeroGravityPolicy      -- always returns a zero wrench
//   CentralGravityPolicy   -- Newtonian point-mass gravity in body frame
//   J2GravityPolicy        -- Central + J2 oblateness perturbation
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cmath>

#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>

namespace {

    using SE3d  = Aetherion::ODE::RKMK::Lie::SE3<double>;
    using Mat3d = Eigen::Matrix3d;
    using Vec3d = Eigen::Vector3d;

    constexpr double kRe = Aetherion::Environment::WGS84::kSemiMajorAxis_m;
    constexpr double kMu = Aetherion::Environment::WGS84::kGM_m3_s2;

    // Surface gravitational acceleration from point-mass model [m/s^2]
    inline constexpr double kGSurf = kMu / (kRe * kRe);

} // namespace

// =============================================================================
// ZeroGravityPolicy
// =============================================================================

TEST_CASE("ZeroGravityPolicy: returns zero wrench for any pose or mass",
    "[gravity][policy][zero]")
{
    using namespace Aetherion::FlightDynamics;
    ZeroGravityPolicy p;

    SECTION("identity pose, unit mass") {
        auto w = p(SE3d::Identity(), 1.0);
        CHECK(w.f.isZero());
    }

    SECTION("non-trivial position, large mass") {
        SE3d g(Mat3d::Identity(), Vec3d(kRe, 0.0, 0.0));
        auto w = p(g, 5000.0);
        CHECK(w.f.isZero());
    }

    SECTION("AD<double> evaluation gives all zeros") {
        using AD    = CppAD::AD<double>;
        using SE3ad = Aetherion::ODE::RKMK::Lie::SE3<AD>;

        SE3ad g_ad(Mat3d::Identity().cast<AD>(), Vec3d(kRe, 0.0, 0.0).cast<AD>());
        auto w_ad = p(g_ad, AD(1.0));

        for (int i = 0; i < 6; ++i)
            CHECK(CppAD::Value(w_ad.f(i)) == 0.0);
    }
}

// =============================================================================
// CentralGravityPolicy
// =============================================================================

TEST_CASE("CentralGravityPolicy: force and moment at equator with identity rotation",
    "[gravity][policy][central]")
{
    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;

    CentralGravityPolicy p;
    const double mass = 1.0;

    SE3d g(Mat3d::Identity(), Vec3d(kRe, 0.0, 0.0));
    auto w = p(g, mass);

    // Moment components must be zero (gravity acts at CG)
    CHECK_THAT(w.f(0), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(1), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(2), WithinAbs(0.0, 1e-12));

    // With identity rotation body frame == ECI; force points along -x
    CHECK_THAT(w.f(3), WithinAbs(-kGSurf * mass, 1e-6));
    CHECK_THAT(w.f(4), WithinAbs(0.0,            1e-12));
    CHECK_THAT(w.f(5), WithinAbs(0.0,            1e-12));
}

TEST_CASE("CentralGravityPolicy: force is proportional to mass",
    "[gravity][policy][central]")
{
    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;

    CentralGravityPolicy p;
    SE3d g(Mat3d::Identity(), Vec3d(kRe, 0.0, 0.0));

    for (double mass : {1.0, 10.0, 500.0}) {
        auto w = p(g, mass);
        CHECK_THAT(w.f(3), WithinAbs(-kGSurf * mass, 1e-3));
        CHECK_THAT(w.f(4), WithinAbs(0.0, 1e-6));
        CHECK_THAT(w.f(5), WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("CentralGravityPolicy: force rotates correctly into body frame",
    "[gravity][policy][central]")
{
    // R_z(+90 deg): body +x -> ECI +y, body +y -> ECI -x
    //
    //     R = [ 0 -1  0 ]      R^T = [ 0  1  0 ]
    //         [ 1  0  0 ]             [-1  0  0 ]
    //         [ 0  0  1 ]             [ 0  0  1 ]
    //
    // Position: (Re, 0, 0) in ECI => F_ECI = (-g_surf*m, 0, 0)
    // F_body = R^T * F_ECI = (0, +g_surf*m, 0)

    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;

    Mat3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;

    const double mass = 1.0;
    SE3d g(R, Vec3d(kRe, 0.0, 0.0));
    auto w = CentralGravityPolicy{}(g, mass);

    CHECK_THAT(w.f(3), WithinAbs(0.0,           1e-6));
    CHECK_THAT(w.f(4), WithinAbs(kGSurf * mass, 1e-6));
    CHECK_THAT(w.f(5), WithinAbs(0.0,           1e-12));
}

TEST_CASE("CentralGravityPolicy: custom mu is applied",
    "[gravity][policy][central]")
{
    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;

    const double mu_custom = 2.0e14;
    CentralGravityPolicy p{ mu_custom };
    const double mass    = 1.0;
    const double g_custom = mu_custom / (kRe * kRe);

    SE3d g(Mat3d::Identity(), Vec3d(kRe, 0.0, 0.0));
    auto w = p(g, mass);

    CHECK_THAT(w.f(3), WithinAbs(-g_custom * mass, 1e-6));
}

TEST_CASE("CentralGravityPolicy: AD<double> evaluation matches double result",
    "[gravity][policy][central][AD]")
{
    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;
    using AD    = CppAD::AD<double>;
    using SE3ad = Aetherion::ODE::RKMK::Lie::SE3<AD>;

    const double mass = 2.0;
    SE3ad g_ad(Mat3d::Identity().cast<AD>(), Vec3d(kRe, 0.0, 0.0).cast<AD>());
    auto w_ad = CentralGravityPolicy{}(g_ad, AD(mass));

    CHECK_THAT(CppAD::Value(w_ad.f(3)), WithinAbs(-kGSurf * mass, 1e-6));
    CHECK_THAT(CppAD::Value(w_ad.f(4)), WithinAbs(0.0, 1e-12));
    CHECK_THAT(CppAD::Value(w_ad.f(5)), WithinAbs(0.0, 1e-12));
}

// =============================================================================
// J2GravityPolicy
// =============================================================================

TEST_CASE("J2GravityPolicy: equatorial J2 perturbation increases radial force magnitude",
    "[gravity][policy][J2]")
{
    // At the equator the J2 term adds a negative (inward) contribution along x,
    // making |F_x| larger than the central-only value.

    using namespace Aetherion::FlightDynamics;

    CentralGravityPolicy central;
    J2GravityPolicy      j2pol;
    const double mass = 1.0;

    SE3d g(Mat3d::Identity(), Vec3d(kRe, 0.0, 0.0));
    auto w_c  = central(g, mass);
    auto w_j2 = j2pol(g, mass);

    REQUIRE(w_c.f(3)  < 0.0);
    REQUIRE(w_j2.f(3) < 0.0);
    CHECK(std::abs(w_j2.f(3)) > std::abs(w_c.f(3)));

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(w_j2.f(4), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w_j2.f(5), WithinAbs(0.0, 1e-12));
}

TEST_CASE("J2GravityPolicy: polar J2 perturbation reduces radial force magnitude",
    "[gravity][policy][J2]")
{
    // At the north pole the J2 term adds a positive (outward) contribution along z,
    // reducing the inward central-gravity magnitude slightly.

    using namespace Aetherion::FlightDynamics;

    CentralGravityPolicy central;
    J2GravityPolicy      j2pol;
    const double mass = 1.0;

    SE3d g(Mat3d::Identity(), Vec3d(0.0, 0.0, kRe));
    auto w_c  = central(g, mass);
    auto w_j2 = j2pol(g, mass);

    REQUIRE(w_c.f(5)  < 0.0);
    REQUIRE(w_j2.f(5) < 0.0);
    CHECK(std::abs(w_j2.f(5)) < std::abs(w_c.f(5)));
}

TEST_CASE("J2GravityPolicy: zero moment regardless of position",
    "[gravity][policy][J2]")
{
    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;

    J2GravityPolicy p;

    for (auto pos : {Vec3d(kRe, 0, 0), Vec3d(0, kRe, 0), Vec3d(0, 0, kRe)}) {
        SE3d g(Mat3d::Identity(), pos);
        auto w = p(g, 1.0);
        CHECK_THAT(w.f(0), WithinAbs(0.0, 1e-12));
        CHECK_THAT(w.f(1), WithinAbs(0.0, 1e-12));
        CHECK_THAT(w.f(2), WithinAbs(0.0, 1e-12));
    }
}

TEST_CASE("J2GravityPolicy: AD<double> evaluation matches double result",
    "[gravity][policy][J2][AD]")
{
    using namespace Aetherion::FlightDynamics;
    using Catch::Matchers::WithinAbs;
    using AD    = CppAD::AD<double>;
    using SE3ad = Aetherion::ODE::RKMK::Lie::SE3<AD>;

    const double mass = 1.0;
    SE3ad g_ad(Mat3d::Identity().cast<AD>(), Vec3d(kRe, 0.0, 0.0).cast<AD>());
    auto w_ad = J2GravityPolicy{}(g_ad, AD(mass));

    // The J2 magnitude at the equator must be larger than the central value
    const double f3_ad = CppAD::Value(w_ad.f(3));
    REQUIRE(f3_ad < 0.0);
    CHECK(std::abs(f3_ad) > kGSurf * mass);
}
