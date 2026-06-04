// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_SpatialInertiaSignConvention.cpp
//
// Regression tests for the Featherstone 6×6 spatial inertia sign convention
// in RigidBody::VectorField.
//
// Background
// ──────────
// An earlier version had all 12 off-diagonal coupling terms with INVERTED
// signs in VectorField.h (and in TwoStageRocketSimulator::rebuildSpatialInertia).
// The wrong signs produce a spurious gravitational-pendulum angular acceleration
//
//   ω̇_y_wrong ≈ -2·xcg·Fz_body / I_CG                                    (≠ 0)
//
// instead of the physically correct value
//
//   ω̇_y_correct = 0   (free-falling body has no angular acceleration)
//
// At xcg = 5 m, m = 314 000 kg, Fz_body ≈ 8 MN, I_CG ≈ 30 MN·m²:
//
//   |ω̇_y_wrong| ≈ 2 × 5 × 8e6 / 30e6 ≈ 2.7 rad/s²   (catastrophic)
//
// The bug was invisible in all other validated examples because they have
// DXCG = 0; it only activates when the CG is offset from the MRC.
//
// Reference: Featherstone, "Rigid Body Dynamics Algorithms", 2008,
//            spatial inertia equation (2.63).
//
// Tests
// ─────
// [spatial_inertia][sign]     : M off-diagonals have correct Featherstone sign
// [spatial_inertia][symmetry] : M is symmetric
// [spatial_inertia][free_fall]: with CG offset, gravity → zero angular accel
// [spatial_inertia][magnitude]: M diagonal blocks have correct magnitude
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/Gravity.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <cmath>
#include <numbers>

using namespace Aetherion;
using namespace Aetherion::RigidBody;

// ── Test-local gravity policy with CG–MRC moment transfer ────────────────────
//
// Identical physics to RocketGravityPolicy but defined here so the test is
// self-contained and does not depend on the Examples directory.
//
// With r_CG = [xcg, 0, 0]^T (body frame), the moment about the MRC is:
//   M_MRC = r_CG × F_grav   →   M_y = -xcg·Fz,   M_z = +xcg·Fy
struct CgOffsetGravityPolicy
{
    double mu   { Environment::WGS84::kGM_m3_s2 };
    double Re   { Environment::WGS84::kSemiMajorAxis_m };
    double J2   { Environment::WGS84::kJ2 };
    double xcg_m{ 0.0 };

    template<class S>
    Spatial::Wrench<S>
    operator()(const ODE::RKMK::Lie::SE3<S>& g, S mass) const
    {
        const Environment::Vec3<S> r{ g.p(0), g.p(1), g.p(2) };
        const auto g_arr = Environment::J2(r, S(mu), S(Re), S(J2));

        const Eigen::Matrix<S, 3, 1> F_body =
            g.R.transpose().template cast<S>()
            * Eigen::Matrix<S, 3, 1>{ g_arr[0] * mass,
                                      g_arr[1] * mass,
                                      g_arr[2] * mass };

        Spatial::Wrench<S> w{};
        w.f.setZero();
        w.f.template tail<3>() = F_body;
        // CG→MRC moment transfer  (only xcg ≠ 0 for axisymmetric body)
        w.f(1) -= S(xcg_m) * F_body(2);   // My = −xcg · Fz
        w.f(2) += S(xcg_m) * F_body(1);   // Mz = +xcg · Fy
        return w;
    }
};

static_assert(FlightDynamics::GravityPolicy<CgOffsetGravityPolicy>);

// ── Rocket-like inertial parameters ──────────────────────────────────────────

namespace {

    // Parameters representative of the TwoStageRocket example mid-S1-burn
    // (the regime where the sign bug caused divergence).
    constexpr double kMass_kg  = 280'000.0;    // after ~7 s of burn
    constexpr double kIxx      =  330'000.0;
    constexpr double kIyy      =  32'000'000.0; // ≈ 30 MN·m²
    constexpr double kXcg_m    =       3.5;    // ~half of max 7.5 m offset

    InertialParameters rocket_params()
    {
        InertialParameters ip;
        ip.mass_kg = kMass_kg;
        ip.Ixx = kIxx;
        ip.Iyy = kIyy;
        ip.Izz = kIyy;      // axisymmetric
        ip.Ixy = 0.0; ip.Iyz = 0.0; ip.Ixz = 0.0;
        ip.xbar_m = kXcg_m; // CG forward of MRC
        ip.ybar_m = 0.0;
        ip.zbar_m = 0.0;
        return ip;
    }

    // Initial SE(3) state: equatorial launch site, pitched 55° from horizontal
    // (same as Scenario 17 initial conditions), placed at low Earth orbit
    // altitude so J2 gravity is well-defined.
    ODE::RKMK::Lie::SE3<double> pitched_pose()
    {
        constexpr double alt_m  = 30'000.0;     // 30 km — within atmosphere
        constexpr double pitch  = 55.0 * std::numbers::pi / 180.0;

        ODE::RKMK::Lie::SE3<double> g = ODE::RKMK::Lie::SE3<double>::Identity();
        // Place on equatorial +x axis at given altitude
        g.p(0) = Environment::WGS84::kSemiMajorAxis_m + alt_m;
        g.p(1) = 0.0;
        g.p(2) = 0.0;
        // Pitch 55° about body-y (nose up from equatorial horizontal)
        Eigen::AngleAxisd rot(pitch, Eigen::Vector3d::UnitY());
        g.R = rot.toRotationMatrix();
        g.q = Eigen::Quaterniond(g.R).normalized();
        return g;
    }

} // anonymous namespace


// ══════════════════════════════════════════════════════════════════════════════
// 1. Off-diagonal sign check — Featherstone h× convention
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Spatial inertia: Featherstone off-diagonal signs (xcg only)",
          "[spatial_inertia][sign][regression]")
{
    // With r_CG = [xcg, 0, 0]^T the cross-coupling matrix is
    //   h× = m · [r_CG×] = m · [[0,0,0],[0,0,-xcg],[0,xcg,0]]
    // giving the Featherstone spatial inertia:
    //   top-right  block: h×     → M[1,5]=-m·xcg, M[2,4]=+m·xcg
    //   bottom-left block: h×^T → M[5,1]=-m·xcg, M[4,2]=+m·xcg
    // All other off-diagonals are zero (no ry, rz offset).

    const auto ip = rocket_params();

    using TestVF = VectorField<FlightDynamics::J2GravityPolicy>;
    TestVF vf(ip);

    const double m   = ip.mass_kg;
    const double xcg = ip.xbar_m;

    using Catch::Approx;

    SECTION("M[1,5] and M[5,1] are negative (-m·xcg)")
    {
        REQUIRE(vf.M(1, 5) == Approx(-m * xcg).epsilon(1e-12));
        REQUIRE(vf.M(5, 1) == Approx(-m * xcg).epsilon(1e-12));
    }

    SECTION("M[2,4] and M[4,2] are positive (+m·xcg)")
    {
        REQUIRE(vf.M(2, 4) == Approx( m * xcg).epsilon(1e-12));
        REQUIRE(vf.M(4, 2) == Approx( m * xcg).epsilon(1e-12));
    }

    SECTION("Other off-diagonals vanish for r=[xcg,0,0]")
    {
        using Catch::Matchers::WithinAbs;
        // rows/cols that couple ry, rz (all zero here)
        CHECK_THAT(vf.M(0, 4), WithinAbs(0.0, 1e-10));
        CHECK_THAT(vf.M(0, 5), WithinAbs(0.0, 1e-10));
        CHECK_THAT(vf.M(1, 3), WithinAbs(0.0, 1e-10));
        CHECK_THAT(vf.M(2, 3), WithinAbs(0.0, 1e-10));
        CHECK_THAT(vf.M(2, 5), WithinAbs(0.0, 1e-10));
        CHECK_THAT(vf.M(3, 1), WithinAbs(0.0, 1e-10));
        CHECK_THAT(vf.M(3, 2), WithinAbs(0.0, 1e-10));
    }

    SECTION("Wrong-sign sentinel: sign must be strictly negative for M[1,5]")
    {
        // This single check is the minimal regression gate: if the signs were
        // inverted (+m·xcg) this assertion would fail immediately.
        REQUIRE(vf.M(1, 5) < 0.0);
    }
}


// ══════════════════════════════════════════════════════════════════════════════
// 2. Symmetry
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Spatial inertia: matrix is symmetric for arbitrary CG offset",
          "[spatial_inertia][symmetry]")
{
    // Test with all three offset components non-zero to cover every term.
    InertialParameters ip;
    ip.mass_kg = 1000.0;
    ip.Ixx = 500.0; ip.Iyy = 2000.0; ip.Izz = 2100.0;
    ip.Ixy = 10.0;  ip.Iyz = 5.0;    ip.Ixz = 8.0;
    ip.xbar_m = 1.5; ip.ybar_m = 0.3; ip.zbar_m = 0.2;

    using TestVF = VectorField<FlightDynamics::J2GravityPolicy>;
    TestVF vf(ip);

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            REQUIRE(vf.M(i, j) == Catch::Approx(vf.M(j, i)).epsilon(1e-12));
}


// ══════════════════════════════════════════════════════════════════════════════
// 3. Diagonal blocks correct magnitude
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Spatial inertia: diagonal blocks have correct magnitude",
          "[spatial_inertia][magnitude]")
{
    const auto ip = rocket_params();

    using TestVF = VectorField<FlightDynamics::J2GravityPolicy>;
    TestVF vf(ip);

    using Catch::Approx;

    // Rotational block (top-left 3×3)
    REQUIRE(vf.M(0, 0) == Approx( ip.Ixx).epsilon(1e-12));
    REQUIRE(vf.M(1, 1) == Approx( ip.Iyy).epsilon(1e-12));
    REQUIRE(vf.M(2, 2) == Approx( ip.Izz).epsilon(1e-12));
    REQUIRE(vf.M(0, 1) == Approx(-ip.Ixy).epsilon(1e-12));
    REQUIRE(vf.M(0, 2) == Approx(-ip.Ixz).epsilon(1e-12));
    REQUIRE(vf.M(1, 2) == Approx(-ip.Iyz).epsilon(1e-12));

    // Translational block (bottom-right 3×3)
    REQUIRE(vf.M(3, 3) == Approx(ip.mass_kg).epsilon(1e-12));
    REQUIRE(vf.M(4, 4) == Approx(ip.mass_kg).epsilon(1e-12));
    REQUIRE(vf.M(5, 5) == Approx(ip.mass_kg).epsilon(1e-12));
}


// ══════════════════════════════════════════════════════════════════════════════
// 4. Free-fall physical regression test
// ══════════════════════════════════════════════════════════════════════════════
//
// KEY REGRESSION CHECK
// ────────────────────
// A rigid body with CG ≠ MRC in free fall under gravity must experience ZERO
// angular acceleration (equivalence principle / torque-free free fall).
//
// With the CORRECT Featherstone sign convention AND the CG→MRC gravity moment
// (M_y = -xcg·Fz), the 2×2 pitching subsystem gives:
//
//   [Iyy   -m·xcg] [ω̇_y]   [-xcg·Fz]
//   [-m·xcg  m   ] [v̇_z] = [  Fz   ]
//   ⟹  ω̇_y = 0,  v̇_z = Fz/m   ✓
//
// With the WRONG signs (+m·xcg off-diagonal), the same system gives:
//
//   [Iyy   +m·xcg] [ω̇_y]   [-xcg·Fz]
//   [+m·xcg  m   ] [v̇_z] = [  Fz   ]
//   ⟹  ω̇_y = -2·m·xcg·Fz / (m·I_CG)  ≈ −2.7×10⁻³ rad/s²  ✗
//
// After dt = 0.001 s this gives Δω_y ≈ 2.7×10⁻⁶ rad/s — detectable at the
// 1×10⁻⁸ rad/s tolerance used below.

TEST_CASE("Spatial inertia: free-fall with CG offset gives zero angular acceleration",
          "[spatial_inertia][free_fall][regression]")
{
    const auto ip = rocket_params();

    // VectorField with CG-aware gravity (adds the CG→MRC moment transfer).
    CgOffsetGravityPolicy grav;
    grav.xcg_m = ip.xbar_m;

    using TestVF      = VectorField<CgOffsetGravityPolicy>;
    using TestStepper = SixDoFStepper<TestVF>;

    TestVF      vf(ip, grav);
    TestStepper stepper(vf);

    StateD s0;
    s0.g    = pitched_pose();
    s0.nu_B = Eigen::Matrix<double, 6, 1>::Zero();  // at rest
    s0.m    = ip.mass_kg;

    constexpr double dt = 0.001;   // 1 ms — same as Scenario 17 time step

    auto result = stepper.step(0.0, s0, dt);

    REQUIRE(result.converged);

    const StateD s1 = TestStepper::unpack(result);

    // ω_y after one step must be essentially zero.
    // Wrong-sign bug gives |Δω_y| ≈ 2.7×10⁻⁶ rad/s after dt = 0.001 s.
    // Correct result: |Δω_y| < 1×10⁻⁸ rad/s (pure numerical noise).
    const double omega_y = s1.nu_B(1);
    REQUIRE(std::abs(omega_y) < 1.0e-8);

    SECTION("Pitch angular velocity stays zero after 100 steps")
    {
        // Accumulate 100 ms of integration; wrong-sign bug would give
        // ω_y ≈ 2.7×10⁻⁴ rad/s → clearly detectable.
        StateD s = s0;
        for (int i = 0; i < 100; ++i)
        {
            auto res = stepper.step(i * dt, s, dt);
            REQUIRE(res.converged);
            s = TestStepper::unpack(res);
        }
        REQUIRE(std::abs(s.nu_B(1)) < 1.0e-7);   // 100 steps of noise
    }
}


// ══════════════════════════════════════════════════════════════════════════════
// 5. DXCG=0 baseline: existing behaviour unchanged
// ══════════════════════════════════════════════════════════════════════════════
//
// When xcg = 0 (all previously-validated examples), the off-diagonal blocks
// are zero and M is block-diagonal — the sign fix has no effect.

TEST_CASE("Spatial inertia: no cross-coupling when CG is at MRC (DXCG=0)",
          "[spatial_inertia][sign][baseline]")
{
    InertialParameters ip;
    ip.mass_kg = 500.0;
    ip.Ixx = 1200.0; ip.Iyy = 5000.0; ip.Izz = 5000.0;
    ip.Ixy = 0.0; ip.Iyz = 0.0; ip.Ixz = 0.0;
    ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;  // CG = MRC

    using TestVF = VectorField<FlightDynamics::J2GravityPolicy>;
    TestVF vf(ip);

    using Catch::Matchers::WithinAbs;

    // All off-diagonal coupling terms must be exactly zero.
    for (int i = 0; i < 3; ++i)
        for (int j = 3; j < 6; ++j)
        {
            CHECK_THAT(vf.M(i, j), WithinAbs(0.0, 1e-12));
            CHECK_THAT(vf.M(j, i), WithinAbs(0.0, 1e-12));
        }
}
