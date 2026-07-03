// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for MakeSnapshot2
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot2.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>

using namespace Aetherion;

// ---------------------------------------------------------------------------
// Helpers — mirror test_ISimulator.cpp's makeState()
// ---------------------------------------------------------------------------

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
// MakeSnapshot2 — two-policy overload (GravityPol + AeroPol)
// ---------------------------------------------------------------------------

TEST_CASE("MakeSnapshot2: time field matches input t", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap = Simulation::MakeSnapshot2(42.5, s, 0.0, gravity, aero);
    REQUIRE(snap.time == Catch::Approx(42.5));
}

TEST_CASE("MakeSnapshot2: altitude is positive for LEO position", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    REQUIRE(snap.altitudeMsl_m > 0.0);
}

TEST_CASE("MakeSnapshot2: local gravity is positive", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    REQUIRE(snap.localGravity_m_s2 > 0.0);
}

TEST_CASE("MakeSnapshot2: zero gravity policy gives zero localGravity", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::ZeroGravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    REQUIRE(snap.localGravity_m_s2 == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("MakeSnapshot2: zero aero policy gives zero aerodynamic force and moment", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    REQUIRE(snap.aero_bodyForce_N_X == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snap.aero_bodyForce_N_Y == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snap.aero_bodyForce_N_Z == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snap.aero_bodyMoment_Nm_L == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snap.aero_bodyMoment_Nm_M == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snap.aero_bodyMoment_Nm_N == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("MakeSnapshot2: non-zero drag policy produces an opposing aerodynamic force", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    FlightDynamics::DragOnlyAeroPolicy aero{ 1.0, 1.0 }; // CD=1, S_ref=1 m^2
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    // Drag opposes the body-frame velocity along the y-axis: expect a non-zero
    // force with sign opposite to the +v_B_y motion (Wrench.f tail = force).
    REQUIRE(snap.aero_bodyForce_N_Y < 0.0);
}

TEST_CASE("MakeSnapshot2: ECEF position matches ECI position at theta_gst = 0", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    REQUIRE(snap.gePosition_m.x() == Catch::Approx(s.g.p.x()).epsilon(1e-12));
    REQUIRE(snap.gePosition_m.y() == Catch::Approx(s.g.p.y()).epsilon(1e-12));
    REQUIRE(snap.gePosition_m.z() == Catch::Approx(s.g.p.z()).epsilon(1e-12));
}

TEST_CASE("MakeSnapshot2: non-zero theta_gst rotates ECEF position", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    FlightDynamics::ZeroAeroPolicy aero{};
    auto snap0 = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, aero);
    auto snapT = Simulation::MakeSnapshot2(0.0, s, 0.5, gravity, aero);
    REQUIRE(snap0.longitude_rad != Catch::Approx(snapT.longitude_rad));
}

TEST_CASE("MakeSnapshot2: matches MakeSnapshot1's shared kinematic/atmospheric fields", "[MakeSnapshot2][MakeSnapshot1]")
{
    auto s = makeState();
    FlightDynamics::CentralGravityPolicy gravity{};
    FlightDynamics::DragOnlyAeroPolicy aero{ 0.5, 2.0 };

    auto snap1 = Simulation::MakeSnapshot1(10.0, s, 0.25, gravity, aero);
    auto snap2 = Simulation::MakeSnapshot2(10.0, s, 0.25, gravity, aero);

    REQUIRE(snap2.time == Catch::Approx(snap1.time));
    REQUIRE(snap2.altitudeMsl_m == Catch::Approx(snap1.altitudeMsl_m).epsilon(1e-12));
    REQUIRE(snap2.localGravity_m_s2 == Catch::Approx(snap1.localGravity_m_s2).epsilon(1e-12));
    REQUIRE(snap2.mach == Catch::Approx(snap1.mach).epsilon(1e-12));
    REQUIRE(snap2.aero_bodyForce_N_X == Catch::Approx(snap1.aero_bodyForce_N_X).epsilon(1e-12));
    REQUIRE(snap2.aero_bodyForce_N_Y == Catch::Approx(snap1.aero_bodyForce_N_Y).epsilon(1e-12));
    REQUIRE(snap2.aero_bodyForce_N_Z == Catch::Approx(snap1.aero_bodyForce_N_Z).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// MakeSnapshot2 — single-policy (dragless) convenience overload
// ---------------------------------------------------------------------------

TEST_CASE("MakeSnapshot2 (dragless overload): equivalent to explicit ZeroAeroPolicy", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};

    auto snapDragless = Simulation::MakeSnapshot2(5.0, s, 0.1, gravity);
    auto snapExplicit = Simulation::MakeSnapshot2(5.0, s, 0.1, gravity, FlightDynamics::ZeroAeroPolicy{});

    REQUIRE(snapDragless.time == Catch::Approx(snapExplicit.time));
    REQUIRE(snapDragless.altitudeMsl_m == Catch::Approx(snapExplicit.altitudeMsl_m).epsilon(1e-12));
    REQUIRE(snapDragless.aero_bodyForce_N_X == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snapDragless.aero_bodyForce_N_Y == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(snapDragless.aero_bodyForce_N_Z == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("MakeSnapshot2 (dragless overload): altitude is positive for LEO position", "[MakeSnapshot2]")
{
    auto s = makeState();
    FlightDynamics::J2GravityPolicy gravity{};
    auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity);
    REQUIRE(snap.altitudeMsl_m > 0.0);
}
