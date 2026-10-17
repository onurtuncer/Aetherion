// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_GravityWrenchOrigin.cpp
//
// Regression tests for the reference point of the gravity wrench in
// RigidBody::VectorField (second pre-DOI audit, finding N01).
//
// M is the spatial inertia about the body origin, so every wrench summed into
// the Newton-Euler equations must be about the origin. Gravity acts at the
// centre of mass, c from the origin, and the gravity policies return their
// wrench there: force m g and no moment. That used to be summed as a wrench
// about the origin, which drops the moment c x F, and a body at rest with its
// centre of mass off the origin picked up the angular acceleration
// I_C^-1 (c x F) under gravity alone: 98 rad/s^2 for the audit's 10 kg body
// with unit inertia and c = 1 m. VectorField now moves the gravity wrench
// from the centre of mass to the origin itself; the rocket example had been
// doing that in a gravity policy of its own, which is gone.
//
// The aero and propulsion policies are about the origin already (the moment
// reference centre of the vehicle model) and are summed as they are.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <numbers>

using namespace Aetherion;
using namespace Aetherion::RigidBody;
using Catch::Matchers::WithinAbs;

namespace {

    using SE3d = ODE::RKMK::Lie::SE3<double>;
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Vec7 = Eigen::Matrix<double, 7, 1>;

    InertialParameters params(double m, const Vec3& diag, const Vec3& c,
        const Vec3& products = Vec3::Zero())
    {
        InertialParameters ip;
        ip.mass_kg = m;
        ip.Ixx = diag.x();
        ip.Iyy = diag.y();
        ip.Izz = diag.z();
        ip.Ixy = products.x();
        ip.Iyz = products.y();
        ip.Ixz = products.z();
        ip.xbar_m = c.x();
        ip.ybar_m = c.y();
        ip.zbar_m = c.z();
        return ip;
    }

    Vec7 restState(double m)
    {
        Vec7 x = Vec7::Zero();
        x(6) = m;
        return x;
    }

    /// Pose at (a + h, 0, 0) in ECI, pitched up by @p pitch_rad about body y.
    SE3d pitchedPose(double h_m, double pitch_rad)
    {
        SE3d g = SE3d::Identity();
        g.p = Vec3(Environment::WGS84::kSemiMajorAxis_m + h_m, 0.0, 0.0);
        g.R = Eigen::AngleAxisd(pitch_rad, Vec3::UnitY()).toRotationMatrix();
        g.q = Eigen::Quaterniond(g.R).normalized();
        return g;
    }

    /// Gravity stand-in: a fixed wrench at the centre of mass, mass-independent.
    struct FixedWrenchAtCM
    {
        Vec6 w = Vec6::Zero();
        template <class S>
        Spatial::Wrench<S> operator()(const ODE::RKMK::Lie::SE3<S>&, S) const
        {
            Spatial::Wrench<S> out{};
            out.f = w.template cast<S>();
            return out;
        }
    };
    static_assert(FlightDynamics::GravityPolicy<FixedWrenchAtCM>);

    /// Aero stand-in: a fixed wrench about the body origin.
    struct FixedWrenchAtOrigin
    {
        Vec6 w = Vec6::Zero();
        template <class S>
        Spatial::Wrench<S> operator()(const ODE::RKMK::Lie::SE3<S>&,
            const Eigen::Matrix<S, 6, 1>&, S, S) const
        {
            Spatial::Wrench<S> out{};
            out.f = w.template cast<S>();
            return out;
        }
    };
    static_assert(FlightDynamics::AeroPolicy<FixedWrenchAtOrigin>);
}

TEST_CASE("Gravity wrench: the audit's offset body falls without turning",
    "[gravity][wrench_origin][regression]")
{
    // 10 kg, unit inertia about the centre of mass, centre of mass 1 m ahead
    // of the origin, at rest on the z axis one Earth radius out, central
    // gravity. The derivative came out with an angular acceleration of
    // 98 rad/s^2 and an origin acceleration of 108 m/s^2. At rest every point
    // of the body accelerates at g: no angular acceleration, and the origin
    // accelerates at mu / a^2 = 9.798 m/s^2 towards the Earth.
    const InertialParameters ip = params(10.0, { 1.0, 1.0, 1.0 }, { 1.0, 0.0, 0.0 });
    const VectorField<FlightDynamics::CentralGravityPolicy> vf(ip);

    SE3d g = SE3d::Identity();
    g.p = Vec3(0.0, 0.0, Environment::WGS84::kSemiMajorAxis_m);

    const Vec7 xdot = vf(0.0, g, restState(10.0));
    const double g0 = Environment::WGS84::kGM_m3_s2 / (Environment::WGS84::kSemiMajorAxis_m * Environment::WGS84::kSemiMajorAxis_m);

    CHECK(xdot.head<3>().norm() < 1e-9);
    CHECK_THAT(xdot(3), WithinAbs(0.0, 1e-9));
    CHECK_THAT(xdot(4), WithinAbs(0.0, 1e-9));
    CHECK_THAT(xdot(5), WithinAbs(-g0, 1e-9));
    CHECK_THAT(g0, WithinAbs(9.798, 1e-3));
}

TEMPLATE_TEST_CASE("Gravity wrench: a body at rest accelerates at g at every point, whatever the offset",
    "[gravity][wrench_origin][regression]",
    FlightDynamics::CentralGravityPolicy, FlightDynamics::J2GravityPolicy)
{
    // At rest the bias term vanishes, so M dnu = W. With the gravity wrench
    // moved to the origin, dnu = [0; F / m]: no angular acceleration and the
    // origin accelerates at g in body axes, for any offset, products of
    // inertia and attitude.
    const InertialParameters ip = params(12.5, { 3.0, 7.0, 9.0 }, { 0.8, -0.3, 0.45 }, { 0.2, -0.1, 0.35 });
    const VectorField<TestType> vf(ip);
    const SE3d g = pitchedPose(30'000.0, 55.0 * std::numbers::pi / 180.0);

    const Vec7 xdot = vf(0.0, g, restState(ip.mass_kg));
    const Vec3 a_body = vf.gravity(g, ip.mass_kg).f.template tail<3>() / ip.mass_kg;

    CHECK(xdot.head<3>().norm() < 1e-9);
    CHECK((xdot.segment<3>(3) - a_body).norm() < 1e-9);
    CHECK_THAT(a_body.norm(), WithinAbs(9.7, 0.15));   // 30 km up
}

TEST_CASE("Gravity wrench: VectorField adds the moment c x F of the gravity force and nothing else",
    "[gravity][wrench_origin][contract]")
{
    // A fixed wrench [tau; F] at the centre of mass, c off the origin. At rest
    // M dnu = W_ext, so M times the derivative is the wrench VectorField
    // summed: it must be [tau + c x F; F], the policy's own moment kept. An
    // aero wrench [tau_a; F_a] is about the origin already and comes through
    // unchanged.
    const Vec3 c(1.0, 0.5, -0.3);
    const Vec3 F(3.0, -4.0, 12.0);
    const Vec3 tau(0.1, 0.2, 0.3);
    const Vec3 F_a(-1.0, 2.0, 5.0);
    const Vec3 tau_a(0.7, -0.2, 0.4);

    FixedWrenchAtCM gravity;
    gravity.w << tau, F;
    FixedWrenchAtOrigin aero;
    aero.w << tau_a, F_a;

    VectorField<FixedWrenchAtCM, FixedWrenchAtOrigin> vf(params(10.0, { 1.0, 2.0, 3.0 }, c), gravity, aero);
    const SE3d g = SE3d::Identity();

    Vec6 expected;
    expected << tau + c.cross(F) + tau_a, F + F_a;

    const Vec7 xdot = vf(0.0, g, restState(10.0));
    const Vec6 W = vf.M * xdot.head<6>();
    CHECK((W - expected).norm() < 1e-9 * expected.norm());

    // The offset follows setInertialParameters, as a variable-mass vehicle's does.
    const Vec3 c2(-2.0, 0.0, 0.75);
    vf.setInertialParameters(params(10.0, { 1.0, 2.0, 3.0 }, c2));
    expected << tau + c2.cross(F) + tau_a, F + F_a;

    const Vec7 xdot2 = vf(0.0, g, restState(10.0));
    const Vec6 W2 = vf.M * xdot2.head<6>();
    CHECK((W2 - expected).norm() < 1e-9 * expected.norm());
}

TEST_CASE("Gravity wrench: the moment added for a CG ahead of the MRC is the rocket's My = -xcg Fz, Mz = +xcg Fy",
    "[gravity][wrench_origin][regression]")
{
    // The two-stage rocket integrates about the MRC with the CG DXCG ahead of
    // it. Its RocketGravityPolicy used to add this moment itself; the example
    // now uses J2GravityPolicy and VectorField adds the same moment, so the
    // trajectory does not change.
    constexpr double xcg = 3.5;
    const InertialParameters ip = params(280'000.0, { 330'000.0, 32e6, 32e6 }, { xcg, 0.0, 0.0 });
    const VectorField<FlightDynamics::J2GravityPolicy> vf(ip);
    const SE3d g = pitchedPose(30'000.0, 55.0 * std::numbers::pi / 180.0);

    const Vec3 F = vf.gravity(g, ip.mass_kg).f.tail<3>();
    const Vec7 xdot = vf(0.0, g, restState(ip.mass_kg));
    const Vec6 W = vf.M * xdot.head<6>();

    CHECK_THAT(W(0), WithinAbs(0.0, 1e-9 * F.norm()));
    CHECK_THAT(W(1), WithinAbs(-xcg * F.z(), 1e-9 * xcg * F.norm()));
    CHECK_THAT(W(2), WithinAbs(xcg * F.y(), 1e-9 * xcg * F.norm()));
    CHECK((W.tail<3>() - F).norm() < 1e-9 * F.norm());
    CHECK(xdot.head<3>().norm() < 1e-8);
}
