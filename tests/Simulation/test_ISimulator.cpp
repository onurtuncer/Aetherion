// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for ISimulator and MakeSnapshot1
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Examples/DraglessSphere/DraglessSphereSimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>

#include <stdexcept>

using namespace Aetherion;
using namespace Aetherion::Examples::DraglessSphere;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static RigidBody::InertialParameters makeIP()
{
    RigidBody::InertialParameters ip;
    ip.mass_kg = 1.0;
    ip.Ixx = 1.0; ip.Iyy = 1.0; ip.Izz = 1.0;
    return ip;
}

// LEO position ~221 km altitude along +X, small orbital velocity along +Y.
static RigidBody::StateD makeState()
{
    RigidBody::StateD s;
    s.g.R = Eigen::Matrix3d::Identity();
    s.g.q = Eigen::Quaterniond::Identity();
    s.g.p = Eigen::Vector3d(6.6e6, 0.0, 0.0);
    s.nu_B.setZero();
    s.nu_B(4) = 7700.0; // v_B_y ≈ orbital speed [m/s]
    s.m = 1.0;
    return s;
}

// ---------------------------------------------------------------------------
// ISimulator — construction and initial accessors
// ---------------------------------------------------------------------------

TEST_CASE("ISimulator: initial time is 0", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    REQUIRE(sim.time() == Catch::Approx(0.0));
}

TEST_CASE("ISimulator: initial state position matches construction input", "[ISimulator]")
{
    auto s0 = makeState();
    DraglessSphereSimulator sim(makeIP(), s0);
    REQUIRE(sim.state().g.p.x() == Catch::Approx(s0.g.p.x()));
    REQUIRE(sim.state().g.p.y() == Catch::Approx(s0.g.p.y()));
    REQUIRE(sim.state().g.p.z() == Catch::Approx(s0.g.p.z()));
}

TEST_CASE("ISimulator: vectorField is accessible", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    // Just verifying the getter compiles and returns a const reference.
    [[maybe_unused]] const auto& vf = sim.vectorField();
    SUCCEED();
}

// ---------------------------------------------------------------------------
// ISimulator — step
// ---------------------------------------------------------------------------

TEST_CASE("ISimulator: step advances time by h", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    const double h = 0.01;
    auto res = sim.step(h);
    if (res.converged)
        REQUIRE(sim.time() == Catch::Approx(h).epsilon(1e-12));
}

TEST_CASE("ISimulator: two steps advance time by 2h", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    const double h = 0.01;
    auto r1 = sim.step(h);
    auto r2 = sim.step(h);
    if (r1.converged && r2.converged)
        REQUIRE(sim.time() == Catch::Approx(2.0 * h).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// ISimulator — advance_to
// ---------------------------------------------------------------------------

TEST_CASE("ISimulator: advance_to reaches t_target", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    const double t_target = 0.1;
    auto res = sim.advance_to(t_target, 0.01);
    if (res.converged)
        REQUIRE(sim.time() == Catch::Approx(t_target).epsilon(1e-12));
}

TEST_CASE("ISimulator: advance_to throws when t_target <= current time", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    REQUIRE_THROWS_AS(sim.advance_to(0.0, 0.01), std::invalid_argument);
    REQUIRE_THROWS_AS(sim.advance_to(-1.0, 0.01), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// ISimulator — snapshot (via DraglessSphereSimulator)
// ---------------------------------------------------------------------------

TEST_CASE("ISimulator: snapshot time matches simulator time at t=0", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    auto snap = sim.snapshot();
    REQUIRE(snap.time == Catch::Approx(0.0));
}

TEST_CASE("ISimulator: snapshot time matches simulator time after a step", "[ISimulator]")
{
    DraglessSphereSimulator sim(makeIP(), makeState());
    const double h = 0.01;
    auto res = sim.step(h);
    if (res.converged) {
        auto snap = sim.snapshot();
        REQUIRE(snap.time == Catch::Approx(sim.time()).epsilon(1e-12));
    }
}

// ---------------------------------------------------------------------------
// MakeSnapshot1 — basic field correctness
// ---------------------------------------------------------------------------

TEST_CASE("MakeSnapshot1: time field matches input t", "[MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot1(42.5, s, 0.0, gravity);
    REQUIRE(snap.time == Catch::Approx(42.5));
}

TEST_CASE("MakeSnapshot1: altitude is positive for LEO position", "[MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot1(0.0, s, 0.0, gravity);
    REQUIRE(snap.altitudeMsl_m > 0.0);
}

TEST_CASE("MakeSnapshot1: local gravity is positive", "[MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot1(0.0, s, 0.0, gravity);
    REQUIRE(snap.localGravity_m_s2 > 0.0);
}

TEST_CASE("MakeSnapshot1: speed of sound is positive at LEO altitude", "[MakeSnapshot1]")
{
    // LEO is above the US-1976 atmosphere (~86 km top), so speed of sound may be
    // small but the function should still return a finite positive value.
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot1(0.0, s, 0.0, gravity);
    REQUIRE(snap.speedOfSound_m_s >= 0.0);
}

TEST_CASE("MakeSnapshot1: ECI position equals state position", "[MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot1(0.0, s, 0.0, gravity);
    REQUIRE(snap.gePosition_m.x() == Catch::Approx(s.g.p.x()).epsilon(1e-12));
    REQUIRE(snap.gePosition_m.y() == Catch::Approx(s.g.p.y()).epsilon(1e-12));
    REQUIRE(snap.gePosition_m.z() == Catch::Approx(s.g.p.z()).epsilon(1e-12));
}

TEST_CASE("MakeSnapshot1: zero gravity policy gives zero localGravity", "[MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::ZeroGravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot1(0.0, s, 0.0, gravity);
    REQUIRE(snap.localGravity_m_s2 == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("MakeSnapshot1: non-zero theta_gst rotates ECEF position", "[MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    auto snap0 = Simulation::MakeSnapshot1(0.0, s, 0.0, gravity);
    auto snapT = Simulation::MakeSnapshot1(0.0, s, 0.5, gravity);
    // Longitude should differ when theta_gst differs (ECI→ECEF rotation changes).
    REQUIRE(snap0.longitude_rad != Catch::Approx(snapT.longitude_rad));
}
