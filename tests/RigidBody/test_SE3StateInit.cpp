// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_SE3StateInit.cpp
//
// Regression tests for the buildInitialState q/R bug:
//
//   The RadauIIA RKMK integrator extracts the initial rotation from g.q
//   (via extract_q_double) and passes that quaternion to the Newton residual.
//   Every buildInitialState() was setting state.g.R but leaving state.g.q at
//   the default Eigen::Quaternion::Identity(), so all implicit stage evaluations
//   used the identity rotation regardless of the actual attitude.  For an F-16
//   trimmed at 2.65° pitch and 45° heading this caused ~75° roll in 0.1 s.
//
//   Fix: also set state.g.q = q.normalized() so both members are consistent.
//
// Test groups:
//   [state][init]              pure StateD / SE3 construction invariants
//   [state][init][regression]  direct detection of the old bug pattern
//   [stepper][regression]      integrator reads g.q, not g.R
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <numbers>

using namespace Aetherion;
using namespace Aetherion::RigidBody;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

// F-16 Scenario-11-like attitude: heading 45°, pitch 2.65°, roll -0.172°
Eigen::Quaterniond f16_trim_quaternion()
{
    return (Eigen::AngleAxisd(45.0 * kDeg,    Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(2.65 * kDeg,   Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(-0.172 * kDeg, Eigen::Vector3d::UnitX()))
          .normalized();
}

// Generic non-trivial attitude: 30° pitch, 45° yaw (easier to reason about)
Eigen::Quaterniond pitched_quaternion()
{
    return (Eigen::AngleAxisd(45.0 * kDeg, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(30.0 * kDeg, Eigen::Vector3d::UnitY()))
          .normalized();
}

// Isotropic inertia (sphere-like) so torque-free dynamics are trivial
InertialParameters isotropic_ip(double mass_kg = 100.0, double moi = 40.0)
{
    InertialParameters ip;
    ip.mass_kg = mass_kg;
    ip.Ixx = ip.Iyy = ip.Izz = moi;
    ip.Ixy = ip.Iyz = ip.Ixz = 0.0;
    ip.xbar_m = ip.ybar_m = ip.zbar_m = 0.0;
    return ip;
}

// GEO position so J2 gravity is non-zero but small (~0.224 m/s²)
Eigen::Vector3d geo_pos() { return { 42164.0e3, 0.0, 0.0 }; }

// Build a correct StateD (both g.q and g.R set from the same quaternion)
StateD make_state_correct(const Eigen::Quaterniond& q)
{
    StateD s;
    s.g.q = q.normalized();
    s.g.R = s.g.q.toRotationMatrix();
    s.g.p = geo_pos();
    s.nu_B.setZero();
    s.m = 100.0;
    return s;
}

// Build a buggy StateD (only g.R set; g.q stays at identity)
StateD make_state_buggy(const Eigen::Quaterniond& q)
{
    StateD s = make_state_correct(q);
    s.g.q = Eigen::Quaterniond::Identity();   // old bug: q never set
    // s.g.R still holds the correct rotation matrix
    return s;
}

} // namespace

// ── [state][init] ──────────────────────────────────────────────────────────────

TEST_CASE("StateD default g.q is identity", "[state][init]")
{
    StateD s;
    const Eigen::Quaterniond id = Eigen::Quaterniond::Identity();
    REQUIRE(s.g.q.angularDistance(id) == Catch::Approx(0.0).margin(1e-14));
    REQUIRE((s.g.R - Eigen::Matrix3d::Identity()).norm() == Catch::Approx(0.0).margin(1e-14));
}

TEST_CASE("StateD: setting g.q then deriving g.R gives consistent members", "[state][init]")
{
    const Eigen::Quaterniond q = f16_trim_quaternion();

    StateD s;
    s.g.q = q;
    s.g.R = s.g.q.toRotationMatrix();  // the correct pattern

    const Eigen::Matrix3d R_from_q = s.g.q.toRotationMatrix();
    REQUIRE((s.g.R - R_from_q).norm() == Catch::Approx(0.0).margin(1e-14));
}

TEST_CASE("StateD: setting g.q then deriving g.R preserves the input rotation",
    "[state][init]")
{
    const Eigen::Quaterniond q = pitched_quaternion();

    StateD s;
    s.g.q = q;
    s.g.R = s.g.q.toRotationMatrix();

    // Body x-axis in ECI: first column of R
    const Eigen::Vector3d x_body = s.g.R.col(0);
    // At 30° pitch and 45° yaw the body-x D-component (ECI Z, roughly NED-down)
    // is -sin(30°) ≈ -0.5 — verify the attitude is preserved
    const double expected_z = -std::sin(30.0 * kDeg);
    REQUIRE(x_body.z() == Catch::Approx(expected_z).margin(0.01));
}

// ── [state][init][regression] ─────────────────────────────────────────────────

TEST_CASE("REGRESSION: old pattern sets only g.R, leaving g.q as identity",
    "[state][init][regression]")
{
    // This test documents the bug that was present before the fix.
    // Setting state.g.R = q.toRotationMatrix() without also setting state.g.q
    // leaves g.q = Identity — g.q and g.R then disagree.
    const Eigen::Quaterniond q = f16_trim_quaternion();

    StateD buggy;
    buggy.g.R = q.toRotationMatrix();   // only R set — old bug

    const Eigen::Matrix3d R_from_q = buggy.g.q.toRotationMatrix();  // identity
    const double mismatch = (buggy.g.R - R_from_q).norm();

    // The mismatch must be substantial (identity vs actual 2.65° pitch / 45° heading)
    REQUIRE(mismatch > 0.1);
}

TEST_CASE("REGRESSION: correct pattern eliminates q/R mismatch", "[state][init][regression]")
{
    const Eigen::Quaterniond q = f16_trim_quaternion();

    StateD fixed;
    fixed.g.q = q.normalized();           // also set q — the fix
    fixed.g.R = fixed.g.q.toRotationMatrix();

    const Eigen::Matrix3d R_from_q = fixed.g.q.toRotationMatrix();
    const double mismatch = (fixed.g.R - R_from_q).norm();

    REQUIRE(mismatch == Catch::Approx(0.0).margin(1e-14));
}

// ── [stepper][regression] ─────────────────────────────────────────────────────

// The integrator passes q0 = g.q (not g.R) to the Newton residual.
// Two states with identical g.R but different g.q must produce different
// step results — proving that g.q is what the integrator actually reads.
TEST_CASE("REGRESSION: stepper reads g.q, so mismatched g.q gives wrong dynamics",
    "[stepper][regression]")
{
    using VF = RigidBody::VectorField<FlightDynamics::J2GravityPolicy>;
    using Stepper = SixDoFStepper<VF>;

    const Eigen::Quaterniond q = pitched_quaternion();  // 30° pitch, 45° yaw
    const auto ip = isotropic_ip();

    Stepper stepper{ ip };

    // Correct initialisation: both g.q and g.R reflect the actual attitude
    const StateD s_correct = make_state_correct(q);

    // Buggy initialisation: g.R is correct but g.q is identity
    const StateD s_buggy = make_state_buggy(q);

    const auto res_c = stepper.step(0.0, s_correct, 0.1);
    const auto res_b = stepper.step(0.0, s_buggy,   0.1);

    REQUIRE(res_c.converged);
    REQUIRE(res_b.converged);

    const auto s1_c = Stepper::unpack(res_c);
    const auto s1_b = Stepper::unpack(res_b);

    // The residual uses q0 to rotate the gravity vector into body frame.
    // With correct q (30° pitch):  body-Z gravity ≠ body-Z gravity with q=identity.
    // After 0.1 s the body-Z velocity must differ meaningfully.
    const double dv_z = std::abs(s1_c.nu_B(5) - s1_b.nu_B(5));
    REQUIRE(dv_z > 1e-4);   // at least 0.1 mm/s difference — conservative threshold
}

// With the fix in place: a step from a non-trivially pitched state
// must converge and produce a negligibly small attitude change
// (no external torques, short timestep).
TEST_CASE("Stepper: non-trivial initial attitude step is stable with correct q/R",
    "[stepper][regression]")
{
    using VF = RigidBody::VectorField<FlightDynamics::J2GravityPolicy>;
    using Stepper = SixDoFStepper<VF>;

    const auto q = f16_trim_quaternion();
    const auto ip = isotropic_ip();

    const StateD s0 = make_state_correct(q);

    Stepper stepper{ ip };
    const auto res = stepper.step(0.0, s0, 0.1);

    REQUIRE(res.converged);

    const StateD s1 = Stepper::unpack(res);

    // Angular distance between start and end quaternion must be tiny.
    // At GEO there is no external torque → rotation barely changes in 0.1 s.
    const double angle_rad = s0.g.q.angularDistance(s1.g.q);
    REQUIRE(angle_rad < 1e-4);   // < 0.006 degrees
}

// Sanity: stepping from a state where g.q ≠ g.R (buggy) deviates more
// from the correct trajectory than the machine-epsilon mismatch.
TEST_CASE("Stepper: buggy initialisation diverges from correct trajectory",
    "[stepper][regression]")
{
    using VF = RigidBody::VectorField<FlightDynamics::J2GravityPolicy>;
    using Stepper = SixDoFStepper<VF>;

    const auto q = pitched_quaternion();
    const auto ip = isotropic_ip();

    Stepper stepper{ ip };

    const StateD s_correct = make_state_correct(q);
    const StateD s_buggy   = make_state_buggy(q);

    // Integrate a few steps to amplify the divergence
    StateD sc = s_correct;
    StateD sb = s_buggy;
    for (int i = 0; i < 5; ++i) {
        auto rc = stepper.step(i * 0.1, sc, 0.1);
        auto rb = stepper.step(i * 0.1, sb, 0.1);
        REQUIRE(rc.converged);
        REQUIRE(rb.converged);
        sc = Stepper::unpack(rc);
        sb = Stepper::unpack(rb);
    }

    // After 0.5 s the body velocities must differ measurably
    const double vel_diff = (sc.nu_B - sb.nu_B).norm();
    REQUIRE(vel_diff > 1e-5);
}
