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
//   FlightDynamics/Policies/AeroPolicies.h      (ZeroAeroPolicy, DragOnlyAeroPolicy)
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/Atmosphere.h>

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

// =============================================================================
// DragOnlyAeroPolicy — regression tests for atmosphere-relative airspeed
//
// Key invariant: drag must be computed using v_rel = v_B − R^T*(ω_E × r_ECI),
// not the raw body-frame ECI velocity v_B.  A body co-rotating with the Earth
// (v_B = R^T*(ω_E × r_ECI)) has zero airspeed relative to the atmosphere and
// must produce zero drag.  Using v_B directly would give ~426 N of spurious drag
// at the equator, breaking the NASA TM-2015-218675 Scenario 6 sphere free-fall.
// =============================================================================

namespace {
    // Build the body-frame ECI velocity that matches the Earth surface velocity
    // at position r_ECI for a given rotation matrix R.
    // v_surface_body = R^T * (omega_E x r_ECI)
    Vec3d surfaceVelocityBody(const Mat3d& R, const Vec3d& r_ECI)
    {
        constexpr double omegaE = Aetherion::Environment::WGS84::kRotationRate_rad_s;
        const Vec3d omega_E(0.0, 0.0, omegaE);
        return R.transpose() * omega_E.cross(r_ECI);
    }
}

TEST_CASE("DragOnlyAeroPolicy: zero drag when body co-rotates with Earth (equator)",
    "[FlightDynamics][aero][dragonly][regression]")
{
    // A body sitting on the equator at 9144 m altitude, body frame aligned with
    // ECI (R = I).  Its ECI velocity equals the Earth surface velocity, so the
    // atmosphere-relative airspeed is exactly zero → drag must be zero.
    using namespace Aetherion::Environment;
    constexpr double alt_m = 9144.0;
    const double r = WGS84::kSemiMajorAxis_m + alt_m;

    const Mat3d R = Mat3d::Identity();
    const Vec3d r_ECI(r, 0.0, 0.0);   // equator, prime meridian
    const Vec3d v_surf = surfaceVelocityBody(R, r_ECI);  // ≈ [0, +465.1, 0] m/s

    SE3d g(R, r_ECI);
    Vec6d nu;
    nu << 0.0, 0.0, 0.0,              // angular rates (zero)
          v_surf.x(), v_surf.y(), v_surf.z();  // body velocity = surface velocity

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.47, 0.018241465};
    auto w = p(g, nu, 14.59, 0.0);

    // All moment and force components must be zero (moments are always zero for
    // DragOnlyAeroPolicy; forces must be zero because airspeed is zero).
    CHECK_THAT(w.f(0), WithinAbs(0.0, 1e-6));
    CHECK_THAT(w.f(1), WithinAbs(0.0, 1e-6));
    CHECK_THAT(w.f(2), WithinAbs(0.0, 1e-6));
    CHECK_THAT(w.f(3), WithinAbs(0.0, 1e-6));
    CHECK_THAT(w.f(4), WithinAbs(0.0, 1e-6));
    CHECK_THAT(w.f(5), WithinAbs(0.0, 1e-6));
}

TEST_CASE("DragOnlyAeroPolicy: nonzero drag for body with 3-D atmosphere-relative airspeed",
    "[FlightDynamics][aero][dragonly]")
{
    // Body at equator (r_ECI along +x), R = I.  Impose a known atmosphere-relative
    // velocity dv = [20, -15, -10] m/s beyond the Earth surface velocity.
    // Expected drag: F = -½ ρ CD S |dv| dv  → each component checkable independently.
    using namespace Aetherion::Environment;
    constexpr double alt_m = 9144.0;
    const double r = WGS84::kSemiMajorAxis_m + alt_m;

    const Mat3d R = Mat3d::Identity();
    const Vec3d r_ECI(r, 0.0, 0.0);
    const Vec3d v_surf = surfaceVelocityBody(R, r_ECI);

    const Vec3d dv(20.0, -15.0, -10.0);   // atmosphere-relative airspeed (body frame)

    SE3d g(R, r_ECI);
    Vec6d nu;
    nu << 0.0, 0.0, 0.0,
          v_surf.x() + dv.x(), v_surf.y() + dv.y(), v_surf.z() + dv.z();

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.47, 0.018241465};
    auto w = p(g, nu, 14.59, 0.0);

    // F = -½ ρ CD S |v_rel| v_rel, component-wise expected values
    const double rho    = US1976Atmosphere(alt_m).rho;
    const double v_mag  = dv.norm();
    const double k      = 0.5 * rho * 0.47 * 0.018241465 * v_mag;
    const Vec3d  F_exp  = -k * dv;

    CHECK_THAT(w.f(3), WithinAbs(F_exp.x(), std::abs(F_exp.x()) * 1e-9 + 1e-15));
    CHECK_THAT(w.f(4), WithinAbs(F_exp.y(), std::abs(F_exp.y()) * 1e-9 + 1e-15));
    CHECK_THAT(w.f(5), WithinAbs(F_exp.z(), std::abs(F_exp.z()) * 1e-9 + 1e-15));
}

TEST_CASE("DragOnlyAeroPolicy: drag magnitude scales with airspeed squared (quadratic law)",
    "[FlightDynamics][aero][dragonly]")
{
    // Double the airspeed → drag force doubles in magnitude (|F| = k|v|²
    // because F = ½ρ CD S |v| v, so |F| = ½ρ CD S v²).
    using namespace Aetherion::Environment;
    constexpr double alt_m = 5000.0;
    const double r = WGS84::kSemiMajorAxis_m + alt_m;

    const Mat3d R = Mat3d::Identity();
    const Vec3d r_ECI(r, 0.0, 0.0);
    const Vec3d v_surf = surfaceVelocityBody(R, r_ECI);

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.47, 0.018241465};

    auto makeNu = [&](double dv) {
        Vec6d nu;
        nu << 0, 0, 0, v_surf.x() + dv, v_surf.y(), v_surf.z();
        return nu;
    };

    SE3d g(R, r_ECI);
    const double F1 = p(g, makeNu(50.0), 14.59, 0.0).f(3);
    const double F2 = p(g, makeNu(100.0), 14.59, 0.0).f(3);

    // |F2| / |F1| should be (100/50)² = 4
    CHECK_THAT(F2 / F1, WithinAbs(4.0, 1e-6));
}

TEST_CASE("DragOnlyAeroPolicy: drag opposes atmosphere-relative velocity direction",
    "[FlightDynamics][aero][dragonly]")
{
    // With R = I and body moving in +x in ECI beyond surface velocity,
    // the drag force x-component must be negative (opposing motion).
    using namespace Aetherion::Environment;
    constexpr double alt_m = 9144.0;
    const double r = WGS84::kSemiMajorAxis_m + alt_m;

    const Mat3d R = Mat3d::Identity();
    const Vec3d r_ECI(0.0, r, 0.0);   // body on +y axis (equator, 90° lon)
    const Vec3d v_surf = surfaceVelocityBody(R, r_ECI);

    SE3d g(R, r_ECI);
    Vec6d nu;
    // Add +200 m/s in x beyond the surface velocity in x
    nu << 0, 0, 0, v_surf.x() + 200.0, v_surf.y(), v_surf.z();

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.47, 0.018241465};
    auto w = p(g, nu, 14.59, 0.0);

    CHECK(w.f(3) < 0.0);  // drag opposes +x motion
    CHECK_THAT(w.f(4), WithinAbs(0.0, 1e-8));
    CHECK_THAT(w.f(5), WithinAbs(0.0, 1e-8));
}

TEST_CASE("DragOnlyAeroPolicy: moments are always zero (pure drag, no moment)",
    "[FlightDynamics][aero][dragonly]")
{
    using namespace Aetherion::Environment;
    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    SE3d g(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    Vec6d nu;
    nu << 0.1, 0.2, 0.3, 200.0, 50.0, -30.0;

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.47, 0.018241465};
    auto w = p(g, nu, 14.59, 0.0);

    CHECK_THAT(w.f(0), WithinAbs(0.0, 1e-15));
    CHECK_THAT(w.f(1), WithinAbs(0.0, 1e-15));
    CHECK_THAT(w.f(2), WithinAbs(0.0, 1e-15));
}

TEST_CASE("DragOnlyAeroPolicy: zero CD gives zero drag regardless of velocity",
    "[FlightDynamics][aero][dragonly]")
{
    using namespace Aetherion::Environment;
    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    SE3d g(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    Vec6d nu;
    nu << 0, 0, 0, 500.0, 200.0, -100.0;

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.0, 0.018241465};
    auto w = p(g, nu, 14.59, 0.0);
    CHECK(w.f.isZero());
}

TEST_CASE("DragOnlyAeroPolicy: evaluates with AD<double> scalars",
    "[FlightDynamics][aero][dragonly][AD]")
{
    using AD  = CppAD::AD<double>;
    using SE3AD  = Aetherion::ODE::RKMK::Lie::SE3<AD>;
    using Vec6AD = Eigen::Matrix<AD, 6, 1>;
    using Mat3AD = Eigen::Matrix<AD, 3, 3>;
    using Vec3AD = Eigen::Matrix<AD, 3, 1>;
    using namespace Aetherion::Environment;

    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    Mat3AD R = Mat3AD::Identity();
    Vec3AD rECI(AD(r), AD(0.0), AD(0.0));

    constexpr double omegaE = WGS84::kRotationRate_rad_s;
    const Vec3AD omega_E(AD(0.0), AD(0.0), AD(omegaE));
    const Vec3AD v_surf = R.transpose() * omega_E.cross(rECI);

    SE3AD g(R, rECI);
    Vec6AD nu;
    nu << AD(0), AD(0), AD(0), v_surf(0) + AD(50.0), v_surf(1), v_surf(2);

    Aetherion::FlightDynamics::DragOnlyAeroPolicy p{0.47, 0.018241465};
    auto w = p(g, nu, AD(14.59), AD(0.0));

    // x and z force components must be zero (airspeed purely in x beyond surface)
    CHECK_THAT(CppAD::Value(w.f(4)), WithinAbs(0.0, 1e-8));
    CHECK_THAT(CppAD::Value(w.f(5)), WithinAbs(0.0, 1e-8));
    // drag must be negative in x (opposing +x relative motion)
    CHECK(CppAD::Value(w.f(3)) < 0.0);
}

// =============================================================================
// BrickDampingAeroPolicy — regression tests
//
// Key invariants:
//   1. Zero velocity → zero drag AND zero damping moments.
//   2. Moments oppose angular rates (Clp<0 → L opposes p).
//   3. Moments scale with ρ·|V|·S·ref²·rate (correct formula).
//   4. Drag part is identical to DragOnlyAeroPolicy.
//   5. AD<double> evaluates without error.
// =============================================================================

TEST_CASE("BrickDampingAeroPolicy: zero velocity gives zero wrench",
    "[FlightDynamics][aero][brickdamping]")
{
    using namespace Aetherion::Environment;
    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    const Vec3d  v_surf = surfaceVelocityBody(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));

    SE3d g(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    Vec6d nu;
    nu << 0.1, 0.2, 0.3,                          // body angular rates
          v_surf.x(), v_surf.y(), v_surf.z();      // co-rotating = zero airspeed

    Aetherion::FlightDynamics::BrickDampingAeroPolicy p{
        0.01, 0.020645, 0.1016, 0.2032, -1.0, -1.0, -1.0};
    auto w = p(g, nu, 2.268, 0.0);

    // Zero airspeed → no drag, no damping moments (all scale with |V|)
    for (int i = 0; i < 6; ++i)
        CHECK_THAT(w.f(i), WithinAbs(0.0, 1e-6));
}

TEST_CASE("BrickDampingAeroPolicy: moments oppose angular rates",
    "[FlightDynamics][aero][brickdamping]")
{
    // Body co-rotating + extra roll rate p = +0.5 rad/s → roll moment must be negative
    using namespace Aetherion::Environment;
    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    const Vec3d  v_surf = surfaceVelocityBody(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    const double V_airspeed = 50.0;   // add downward airspeed to activate moments

    SE3d g(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    Vec6d nu;
    nu << 0.5, 0.3, -0.2,
          v_surf.x(), v_surf.y(), v_surf.z() + (-V_airspeed); // downward

    Aetherion::FlightDynamics::BrickDampingAeroPolicy p{
        0.01, 0.020645, 0.1016, 0.2032, -1.0, -1.0, -1.0};
    auto w = p(g, nu, 2.268, 0.0);

    // Clp = -1 → roll moment opposes positive p → L < 0
    CHECK(w.f(0) < 0.0);
    // Cmq = -1 → pitch moment opposes positive q → M < 0
    CHECK(w.f(1) < 0.0);
    // Cnr = -1 → yaw moment opposes negative r → N > 0
    CHECK(w.f(2) > 0.0);
    // Moments are zero: drag force must be non-zero (opposing downward airspeed)
    CHECK(w.f(5) > 0.0);
}

TEST_CASE("BrickDampingAeroPolicy: roll moment formula matches expected value",
    "[FlightDynamics][aero][brickdamping]")
{
    // L = ¼ ρ |V| S b² Clp p
    using namespace Aetherion::Environment;
    constexpr double alt_m = 9144.0;
    constexpr double Vair  = 100.0;   // m/s airspeed
    constexpr double p_rad = 0.5;     // rad/s roll rate
    const double     r     = WGS84::kSemiMajorAxis_m + alt_m;
    const Vec3d      v_surf = surfaceVelocityBody(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));

    SE3d g(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    Vec6d nu;
    nu << p_rad, 0.0, 0.0,
          v_surf.x(), v_surf.y(), v_surf.z() + (-Vair);

    constexpr double CD=0.01, S=0.020645, b=0.1016, cbar=0.2032, Clp=-1.0, Cmq=-1.0, Cnr=-1.0;
    Aetherion::FlightDynamics::BrickDampingAeroPolicy pol{CD, S, b, cbar, Clp, Cmq, Cnr};
    auto w = pol(g, nu, 2.268, 0.0);

    const double rho     = US1976Atmosphere(alt_m).rho;
    const double L_expected = 0.25 * rho * Vair * S * b * b * Clp * p_rad;

    CHECK_THAT(w.f(0), WithinAbs(L_expected, std::abs(L_expected) * 1e-9 + 1e-15));
    // q=r=0 → no pitch or yaw moment
    CHECK_THAT(w.f(1), WithinAbs(0.0, 1e-12));
    CHECK_THAT(w.f(2), WithinAbs(0.0, 1e-12));
}

TEST_CASE("BrickDampingAeroPolicy: drag part matches DragOnlyAeroPolicy",
    "[FlightDynamics][aero][brickdamping]")
{
    // With zero angular rates, the drag force should equal DragOnlyAeroPolicy
    using namespace Aetherion::Environment;
    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    const Vec3d  v_surf = surfaceVelocityBody(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    const Vec3d  dv(20.0, -15.0, -10.0);

    SE3d g(Mat3d::Identity(), Vec3d(r, 0.0, 0.0));
    Vec6d nu;
    nu << 0.0, 0.0, 0.0,
          v_surf.x() + dv.x(), v_surf.y() + dv.y(), v_surf.z() + dv.z();

    Aetherion::FlightDynamics::BrickDampingAeroPolicy   bd{0.01, 0.020645, 0.1016, 0.2032, 0.0, 0.0, 0.0};
    Aetherion::FlightDynamics::DragOnlyAeroPolicy       donly{0.01, 0.020645};

    auto wb = bd(g, nu, 2.268, 0.0);
    auto wd = donly(g, nu, 2.268, 0.0);

    // Forces must match; moments are zero in both cases
    for (int i = 3; i < 6; ++i)
        CHECK_THAT(wb.f(i), WithinAbs(wd.f(i), std::abs(wd.f(i)) * 1e-12 + 1e-20));
    for (int i = 0; i < 3; ++i)
        CHECK_THAT(wb.f(i), WithinAbs(0.0, 1e-15));
}

TEST_CASE("BrickDampingAeroPolicy: evaluates with AD<double>",
    "[FlightDynamics][aero][brickdamping][AD]")
{
    using AD   = CppAD::AD<double>;
    using SE3AD  = Aetherion::ODE::RKMK::Lie::SE3<AD>;
    using Vec6AD = Eigen::Matrix<AD, 6, 1>;
    using Mat3AD = Eigen::Matrix<AD, 3, 3>;
    using Vec3AD = Eigen::Matrix<AD, 3, 1>;
    using namespace Aetherion::Environment;

    const double r = WGS84::kSemiMajorAxis_m + 9144.0;
    constexpr double omegaE = WGS84::kRotationRate_rad_s;
    const Vec3AD rECI(AD(r), AD(0.0), AD(0.0));
    const Vec3AD omega_E(AD(0.0), AD(0.0), AD(omegaE));
    const Vec3AD v_surf = Mat3AD::Identity().transpose() * omega_E.cross(rECI);

    SE3AD g(Mat3AD::Identity(), rECI);
    Vec6AD nu;
    nu << AD(0.5), AD(0.3), AD(-0.2),
          v_surf(0) + AD(20.0), v_surf(1), v_surf(2) + AD(-50.0);

    Aetherion::FlightDynamics::BrickDampingAeroPolicy pol{
        0.01, 0.020645, 0.1016, 0.2032, -1.0, -1.0, -1.0};
    auto w = pol(g, nu, AD(2.268), AD(0.0));

    // roll moment must oppose positive p
    CHECK(CppAD::Value(w.f(0)) < 0.0);
    // drag must be non-zero
    CHECK(CppAD::Value(w.f(3)) != 0.0);
}
