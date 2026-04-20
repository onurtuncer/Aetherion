// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_MassPropulsionAeroPolicies.cpp
//
// Catch2 tests for:
//   FlightDynamics/Policies/MassPolicies.h     (ConstantMassPolicy, LinearBurnPolicy)
//   FlightDynamics/Policies/PropulsionPolicies.h (ZeroPropulsionPolicy, ConstantThrustPolicy)
//   FlightDynamics/Policies/AeroPolicies.h      (ZeroAeroPolicy)
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>

namespace {
    using SE3d  = Aetherion::ODE::RKMK::Lie::SE3<double>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    using Mat3d = Eigen::Matrix3d;
    using Vec3d = Eigen::Vector3d;
    using Catch::Matchers::WithinAbs;
}

// =============================================================================
// MassPolicies
// =============================================================================

TEST_CASE("ConstantMassPolicy: mdot is always zero", "[FlightDynamics][mass][constant]")
{
    Aetherion::FlightDynamics::ConstantMassPolicy p;

    for (double t    : {0.0, 1.0, 100.0})
    for (double mass : {1.0, 500.0, 5000.0}) {
        CHECK_THAT(p.mdot(t, mass), WithinAbs(0.0, 1e-15));
    }
}

TEST_CASE("ConstantMassPolicy: mdot is zero with AD<double>", "[FlightDynamics][mass][constant][AD]")
{
    using AD = CppAD::AD<double>;
    Aetherion::FlightDynamics::ConstantMassPolicy p;
    CHECK(CppAD::Value(p.mdot(AD(5.0), AD(1000.0))) == 0.0);
}

TEST_CASE("LinearBurnPolicy: mdot returns configured rate", "[FlightDynamics][mass][burn]")
{
    const double rate = -2.5;  // kg/s
    Aetherion::FlightDynamics::LinearBurnPolicy p{ rate };

    for (double t    : {0.0, 1.0, 50.0})
    for (double mass : {100.0, 500.0}) {
        CHECK_THAT(p.mdot(t, mass), WithinAbs(rate, 1e-15));
    }
}

TEST_CASE("LinearBurnPolicy: zero rate behaves like constant mass", "[FlightDynamics][mass][burn]")
{
    Aetherion::FlightDynamics::LinearBurnPolicy p{ 0.0 };
    CHECK_THAT(p.mdot(10.0, 200.0), WithinAbs(0.0, 1e-15));
}

// =============================================================================
// PropulsionPolicies
// =============================================================================

TEST_CASE("ZeroPropulsionPolicy: returns zero wrench for any state", "[FlightDynamics][propulsion][zero]")
{
    Aetherion::FlightDynamics::ZeroPropulsionPolicy p;
    SE3d g(Mat3d::Identity(), Vec3d(0.0, 0.0, 0.0));
    Vec6d nu = Vec6d::Zero();

    auto w = p(g, nu, 1.0, 0.0);
    CHECK(w.f.isZero());
}

TEST_CASE("ConstantThrustPolicy: force along body z-axis equals configured thrust", "[FlightDynamics][propulsion][constant]")
{
    const double F = 5000.0;  // N
    Aetherion::FlightDynamics::ConstantThrustPolicy p{ F };
    SE3d g(Mat3d::Identity(), Vec3d(0.0, 0.0, 0.0));
    Vec6d nu = Vec6d::Zero();

    auto w = p(g, nu, 1.0, 0.0);

    // Moment components must be zero
    CHECK_THAT(w.f(0), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(1), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(2), WithinAbs(0.0, 1e-12));
    // Force along +z (index 5)
    CHECK_THAT(w.f(3), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(4), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(5), WithinAbs(F,   1e-12));
}

TEST_CASE("ConstantThrustPolicy: zero thrust gives zero wrench", "[FlightDynamics][propulsion][constant]")
{
    Aetherion::FlightDynamics::ConstantThrustPolicy p{ 0.0 };
    SE3d g(Mat3d::Identity(), Vec3d(0.0, 0.0, 0.0));
    Vec6d nu = Vec6d::Zero();

    auto w = p(g, nu, 0.0, 0.0);
    CHECK(w.f.isZero());
}

TEST_CASE("ConstantThrustPolicy: thrust is independent of state and time", "[FlightDynamics][propulsion][constant]")
{
    const double F = 1000.0;
    Aetherion::FlightDynamics::ConstantThrustPolicy p{ F };
    Vec6d nu;
    nu << 0.1, 0.2, 0.3, 1.0, 2.0, 3.0;

    for (double t : {0.0, 5.0, 100.0}) {
        SE3d g(Mat3d::Identity(), Vec3d(0.0, 0.0, 7e6));
        auto w = p(g, nu, 1.0, t);
        CHECK_THAT(w.f(5), WithinAbs(F, 1e-12));
    }
}

// =============================================================================
// AeroPolicies
// =============================================================================

TEST_CASE("ZeroAeroPolicy: returns zero wrench for any state", "[FlightDynamics][aero][zero]")
{
    Aetherion::FlightDynamics::ZeroAeroPolicy p;
    SE3d g(Mat3d::Identity(), Vec3d(0.0, 0.0, 0.0));
    Vec6d nu;
    nu << 0.0, 0.0, 0.0, 100.0, 0.0, 0.0;

    auto w = p(g, nu, 0.0, 1.0);
    CHECK(w.f.isZero());
}

TEST_CASE("ZeroAeroPolicy: zero wrench with non-trivial velocity and pose", "[FlightDynamics][aero][zero]")
{
    Aetherion::FlightDynamics::ZeroAeroPolicy p;

    Mat3d R;
    R << 0.0, -1.0, 0.0,
         1.0,  0.0, 0.0,
         0.0,  0.0, 1.0;

    SE3d g(R, Vec3d(1e6, 2e6, 3e6));
    Vec6d nu;
    nu << 0.1, 0.2, 0.3, 300.0, 50.0, -10.0;

    auto w = p(g, nu, 1500.0, 0.8);
    CHECK(w.f.isZero());
}
