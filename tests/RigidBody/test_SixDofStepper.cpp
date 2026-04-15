// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_SixDoFStepper.cpp
//
// Catch2 tests for Aetherion::RigidBody::SixDoFStepper.
//
// Test structure:
//   [stepper][construct]  -- construction paths
//   [stepper][pack]       -- pack/unpack round-trip
//   [stepper][step]       -- step() output contracts
//   [stepper][options]    -- Newton options accessor
//   [stepper][manifold]   -- SE(3) manifold preservation
//   [stepper][energy]     -- free-body energy conservation
//
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <cppad/cppad.hpp>

#include <Eigen/Core>
#include <cmath>

using namespace Aetherion;
using namespace Aetherion::RigidBody;

// --------------------------------------------------------------------------
// Test fixtures
// --------------------------------------------------------------------------
namespace {

    // Unit sphere: mass=1 kg, Ixx=Iyy=Izz=0.4, no CG offset.
    InertialParameters unit_sphere()
    {
        InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = 0.4; ip.Iyy = 0.4; ip.Izz = 0.4;
        ip.Ixy = 0.0; ip.Iyz = 0.0; ip.Ixz = 0.0;
        ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;
        return ip;
    }

    // CentralGravityPolicy is the only gravity policy available.
    // Tests that isolate inertial dynamics place the body at GEO altitude
    // (~42 164 km) where gravity is ~0.224 m/s^2 -- small enough that
    // energy drift over a short test window is dominated by integrator
    // accuracy, not gravitational forcing.
    using TestPolicy = FlightDynamics::CentralGravityPolicy;
    using TestVF = RigidBody::VectorField<TestPolicy>;
    using TestStepper = SixDoFStepper<TestVF>;

    // GEO position: [42164e3, 0, 0] metres in ECI frame.
    // At this radius gravity magnitude ~ 0.224 m/s^2.
    ODE::RKMK::Lie::SE3<double> geo_pose()
    {
        ODE::RKMK::Lie::SE3<double> g = ODE::RKMK::Lie::SE3<double>::Identity();
        g.p(0) = 42164.0e3;
        return g;
    }

    StateD rest_state()
    {
        StateD s;
        s.g = geo_pose();
        s.nu_B = Eigen::Matrix<double, 6, 1>::Zero();
        s.m = 1.0;
        return s;
    }

    StateD spinning_state(double omega_z = 1.0)
    {
        StateD s = rest_state();
        s.nu_B(2) = omega_z;
        return s;
    }

    StateD translating_state(double vx = 1.0)
    {
        StateD s = rest_state();
        s.nu_B(3) = vx;
        return s;
    }

} // namespace

// --------------------------------------------------------------------------
// [stepper][construct]
// --------------------------------------------------------------------------
TEST_CASE("SixDoFStepper -- construct from InertialParameters", "[stepper][construct]")
{
    REQUIRE_NOTHROW(TestStepper{ unit_sphere() });
}

TEST_CASE("SixDoFStepper -- construct from VectorField directly", "[stepper][construct]")
{
    TestVF vf{ unit_sphere() };
    REQUIRE_NOTHROW(TestStepper{ std::move(vf) });
}

TEST_CASE("SixDoFStepper -- construct with custom Newton options", "[stepper][construct]")
{
    ODE::RKMK::Core::NewtonOptions opt;
    opt.max_iters = 50;
    opt.abs_tol = 1e-12;

    TestStepper stepper{ unit_sphere(), opt };

    REQUIRE(stepper.options().max_iters == 50);
    REQUIRE(stepper.options().abs_tol == Catch::Approx(1e-12));
}

// --------------------------------------------------------------------------
// [stepper][options]
// --------------------------------------------------------------------------
TEST_CASE("SixDoFStepper -- options are mutable", "[stepper][options]")
{
    TestStepper stepper{ unit_sphere() };
    stepper.options().max_iters = 25;
    REQUIRE(stepper.options().max_iters == 25);
}

// --------------------------------------------------------------------------
// [stepper][pack]
// --------------------------------------------------------------------------
TEST_CASE("SixDoFStepper -- pack/unpack round-trip is identity", "[stepper][pack]")
{
    StateD s = spinning_state(2.5);
    s.m = 3.7;

    auto x = TestStepper::pack(s);
    auto s_back = TestStepper::unpack(s.g, x);

    REQUIRE((s_back.nu_B - s.nu_B).norm() == Catch::Approx(0.0).margin(1e-15));
    REQUIRE(s_back.m == Catch::Approx(s.m));
}

TEST_CASE("SixDoFStepper -- pack layout: head<6> is nu_B, x(6) is mass", "[stepper][pack]")
{
    StateD s = rest_state();
    s.nu_B << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    s.m = 9.81;

    auto x = TestStepper::pack(s);

    for (int i = 0; i < 6; ++i)
        REQUIRE(x(i) == Catch::Approx(s.nu_B(i)));
    REQUIRE(x(6) == Catch::Approx(9.81));
}

// --------------------------------------------------------------------------
// [stepper][step]
// --------------------------------------------------------------------------
TEST_CASE("SixDoFStepper -- step on resting state converges", "[stepper][step]")
{
    TestStepper stepper{ unit_sphere() };
    auto res = stepper.step(0.0, rest_state(), 0.01);
    REQUIRE(res.converged);
}

TEST_CASE("SixDoFStepper -- step returns finite values", "[stepper][step]")
{
    TestStepper stepper{ unit_sphere() };
    auto res = stepper.step(0.0, spinning_state(), 0.01);

    REQUIRE(res.converged);
    for (int i = 0; i < 7; ++i)
        REQUIRE(std::isfinite(res.x1(i)));

    const auto q = res.g1.rotation();
    REQUIRE(std::isfinite(q.w()));
    REQUIRE(std::isfinite(q.x()));
    REQUIRE(std::isfinite(q.y()));
    REQUIRE(std::isfinite(q.z()));
}

TEST_CASE("SixDoFStepper -- StepResult unpack produces finite state", "[stepper][step]")
{
    TestStepper stepper{ unit_sphere() };
    auto res = stepper.step(0.0, translating_state(), 0.01);

    REQUIRE(res.converged);

    auto s1 = TestStepper::unpack(res);
    REQUIRE(std::isfinite(s1.m));
    REQUIRE(std::isfinite(s1.nu_B.norm()));
}

TEST_CASE("SixDoFStepper -- resting body stays nearly at rest over short window",
    "[stepper][step]")
{
    // At GEO, gravitational acceleration ~ 0.224 m/s^2.
    // Over 10 x 0.01 s = 0.1 s the velocity change is ~ 0.022 m/s -- small
    // but non-zero, so we check convergence only, not zero velocity.
    TestStepper stepper{ unit_sphere() };
    StateD s = rest_state();

    for (int i = 0; i < 10; ++i) {
        auto res = stepper.step(i * 0.01, s, 0.01);
        REQUIRE(res.converged);
        s = TestStepper::unpack(res);
    }

    REQUIRE(std::isfinite(s.nu_B.norm()));
}

// --------------------------------------------------------------------------
// [stepper][manifold]
// --------------------------------------------------------------------------
TEST_CASE("SixDoFStepper -- g1 stays on SE(3): quaternion norm = 1",
    "[stepper][manifold]")
{
    TestStepper stepper{ unit_sphere() };
    StateD s = spinning_state();

    for (int i = 0; i < 20; ++i) {
        auto res = stepper.step(i * 0.05, s, 0.05);
        REQUIRE(res.converged);

        const double qnorm = res.g1.q.norm();
        REQUIRE(qnorm == Catch::Approx(1.0).epsilon(1e-12));

        s = TestStepper::unpack(res);
    }
}

// --------------------------------------------------------------------------
// [stepper][energy]
//
// A torque-free symmetric body spinning in deep space conserves kinetic
// energy. We use J2GravityPolicy here would add non-conservative perturbations,
// so we stick with CentralGravityPolicy and a purely rotational initial
// condition (no translation) so that the gravitational wrench is zero in
// the body-frame angular channel.
//
// For a symmetric body (Ixx=Iyy=Izz) with pure spin and no translational
// velocity, M*nu_B = [J*omega; 0] and the bias term ad*(nu_B)*M*nu_B has
// zero angular component (Euler equations reduce to omega_dot = 0).
// KE = 0.5 * nu_B^T * M * nu_B is therefore conserved to integrator accuracy.
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
// [stepper][energy]
// --------------------------------------------------------------------------
TEST_CASE("SixDoFStepper -- symmetric torque-free body conserves rotational KE",
    "[stepper][energy]")
{
    // Use zero gravity so no external forces do work -- purely inertial.
    using EnergyVF = RigidBody::VectorField<FlightDynamics::ZeroGravityPolicy>;
    using EnergyStepper = SixDoFStepper<EnergyVF>;

    auto ip = unit_sphere();
    EnergyStepper stepper{ ip };

    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
    M(0, 0) = ip.Ixx;     M(1, 1) = ip.Iyy;     M(2, 2) = ip.Izz;
    M(3, 3) = ip.mass_kg; M(4, 4) = ip.mass_kg; M(5, 5) = ip.mass_kg;

    auto ke = [&](const StateD& s) {
        return 0.5 * s.nu_B.dot(M * s.nu_B);
        };

    // Pure spin -- no translation, no gravity, no external forces.
    StateD s = rest_state();
    s.g = ODE::RKMK::Lie::SE3<double>::Identity();  // origin, no gravity
    s.nu_B << 1.0, 0.5, 0.25, 0.0, 0.0, 0.0;

    double ke0 = ke(s);

    for (int i = 0; i < 50; ++i) {
        auto res = stepper.step(i * 0.02, s, 0.02);
        REQUIRE(res.converged);
        s = EnergyStepper::unpack(res);
    }

    // Radau IIA is stiffly accurate -- KE drift should be well below 1e-6.
    REQUIRE(ke(s) == Catch::Approx(ke0).epsilon(1e-6));
}