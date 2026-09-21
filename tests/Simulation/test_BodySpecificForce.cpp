// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for BodySpecificForce
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// The error worth designing these tests around is gravity inclusion: adding a
// gravity term to specific force produces a clean ~9.81 m/s² offset that looks
// plausible on every plot.  The dragless-sphere case below is the sharp oracle
// for it — only gravity acts on that vehicle, so specific force is exactly zero
// and a gravity-inclusion bug reads about ±9.81 m/s².
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Simulation/BodySpecificForce.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <cmath>

using namespace Aetherion;
using Catch::Approx;

namespace {

// A vehicle in the atmosphere: ~10 km altitude above the equator on the ECI +X
// axis, flying along body +x, nose level.  Body axes are aligned with ECI here
// only to keep the expected values readable — the policies are evaluated in the
// body frame either way.
RigidBody::StateD makeAtmosphericState(double mass_kg = 1000.0,
                                       double u_m_s   = 200.0)
{
    RigidBody::StateD s;
    s.g.R = Eigen::Matrix3d::Identity();
    s.g.q = Eigen::Quaterniond::Identity();
    s.g.p = Eigen::Vector3d(Environment::WGS84::kSemiMajorAxis_m + 10000.0, 0.0, 0.0);
    s.nu_B.setZero();
    s.nu_B(3) = u_m_s;   // body-x velocity
    s.m       = mass_kg;
    return s;
}

// Far above the atmosphere, so DragOnlyAeroPolicy returns a vanishing force.
RigidBody::StateD makeExoatmosphericState(double mass_kg = 1000.0)
{
    RigidBody::StateD s = makeAtmosphericState(mass_kg);
    s.g.p = Eigen::Vector3d(Environment::WGS84::kSemiMajorAxis_m + 400000.0, 0.0, 0.0);
    return s;
}

// Constant body-frame force, for arithmetic checks that do not depend on any
// physical model.  Satisfies AeroPolicy / PropulsionPolicy.
struct ConstantBodyForcePolicy {
    double Fx{ 0.0 };
    double Fy{ 0.0 };
    double Fz{ 0.0 };

    template<class S>
    Spatial::Wrench<S> operator()(const ODE::RKMK::Lie::SE3<S>&,
                                  const Eigen::Matrix<S, 6, 1>&,
                                  S, S) const
    {
        Spatial::Wrench<S> w{};
        w.f.setZero();
        // Non-zero moments, to prove they are discarded rather than leaking into
        // the force channels.
        w.f(0) = S(1234.0);
        w.f(1) = S(-5678.0);
        w.f(2) = S(9012.0);
        w.f(3) = S(Fx);
        w.f(4) = S(Fy);
        w.f(5) = S(Fz);
        return w;
    }
};
static_assert(FlightDynamics::AeroPolicy<ConstantBodyForcePolicy>);

} // namespace

// ---------------------------------------------------------------------------
// Acceptance criterion 1 — DraglessSphere: specific force is identically zero.
//
// DraglessSphereVF = J2Gravity + ZeroAero + ZeroPropulsion + ConstantMass, so
// gravity is the only force acting.  Specific force must be exactly zero, to
// floating point, everywhere along the trajectory.
// ---------------------------------------------------------------------------

TEST_CASE("BodySpecificForce: dragless sphere reads exactly zero", "[BodySpecificForce]")
{
    const FlightDynamics::ZeroAeroPolicy       aero{};
    const FlightDynamics::ZeroPropulsionPolicy thrust{};

    // Several points spanning the atmosphere and beyond, with attitude and
    // velocity varied so any frame-handling slip would show up.
    for (const double alt_m : { 0.0, 10000.0, 100000.0, 400000.0 }) {
        RigidBody::StateD s = makeAtmosphericState();
        s.g.p = Eigen::Vector3d(Environment::WGS84::kSemiMajorAxis_m + alt_m, 0.0, 0.0);
        s.g.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
        s.g.R = s.g.q.toRotationMatrix();
        s.nu_B << 0.1, -0.2, 0.3, 250.0, -30.0, 15.0;

        const Eigen::Vector3d f_B =
            Simulation::BodySpecificForce_m_s2(3.75, s, aero, thrust);

        // Exact zero, not an Approx: a gravity-inclusion error would land near
        // ±9.81 m/s² here, and there is no round-off to tolerate in 0/m.
        REQUIRE(f_B.x() == 0.0);
        REQUIRE(f_B.y() == 0.0);
        REQUIRE(f_B.z() == 0.0);
    }
}

TEST_CASE("BodySpecificForce: gravity magnitude at the same state is not zero",
          "[BodySpecificForce]")
{
    // Guards the test above from passing vacuously: confirm the state really is
    // in a strong gravity field, so "zero" is a statement about exclusion of
    // gravitation and not about there being none.
    const RigidBody::StateD s = makeAtmosphericState();
    const FlightDynamics::J2GravityPolicy gravity{};
    const Eigen::Vector3d g_B = gravity(s.g, 1.0).f.tail<3>();

    REQUIRE(g_B.norm() > 9.0);
    REQUIRE(g_B.norm() < 10.0);
}

// ---------------------------------------------------------------------------
// Acceptance criterion 2 — SphereWithAtmosphericDrag: specific force = drag/m only,
// antiparallel to the velocity vector, vanishing as density does.
// ---------------------------------------------------------------------------

TEST_CASE("BodySpecificForce: drag-only sphere is antiparallel to velocity",
          "[BodySpecificForce]")
{
    const FlightDynamics::DragOnlyAeroPolicy   aero{ 0.5, 1.0 };  // CD, S_ref [m²]
    const FlightDynamics::ZeroPropulsionPolicy thrust{};

    RigidBody::StateD s = makeAtmosphericState(1000.0, 200.0);
    s.nu_B(3) = 180.0;
    s.nu_B(4) =  60.0;
    s.nu_B(5) = -40.0;

    const Eigen::Vector3d f_B = Simulation::BodySpecificForce_m_s2(0.0, s, aero, thrust);

    // The atmosphere co-rotates, so the aerodynamic velocity is the body
    // velocity minus the surface velocity — compare against that, not nu_B.
    const Eigen::Vector3d omega_E(0.0, 0.0, Environment::WGS84::kRotationRate_rad_s);
    const Eigen::Vector3d v_rel =
        s.nu_B.tail<3>() - s.g.R.transpose() * omega_E.cross(s.g.p);

    REQUIRE(f_B.norm() > 0.0);

    // cos of the angle between specific force and airspeed must be exactly −1.
    const double cosAngle = f_B.dot(v_rel) / (f_B.norm() * v_rel.norm());
    REQUIRE(cosAngle == Approx(-1.0).margin(1.0e-12));

    // Magnitude: |f| = ½ ρ CD S |v|² / m.
    const double rho =
        Environment::US1976Atmosphere(s.g.p.norm() - Environment::WGS84::kSemiMajorAxis_m).rho;
    const double expected = 0.5 * rho * 0.5 * 1.0 * v_rel.squaredNorm() / s.m;
    REQUIRE(f_B.norm() == Approx(expected).epsilon(1.0e-10));
}

TEST_CASE("BodySpecificForce: drag-only sphere goes to zero as density does",
          "[BodySpecificForce]")
{
    const FlightDynamics::DragOnlyAeroPolicy   aero{ 0.5, 1.0 };
    const FlightDynamics::ZeroPropulsionPolicy thrust{};

    const RigidBody::StateD low  = makeAtmosphericState();
    const RigidBody::StateD high = makeExoatmosphericState();

    const double f_low  = Simulation::BodySpecificForce_m_s2(0.0, low,  aero, thrust).norm();
    const double f_high = Simulation::BodySpecificForce_m_s2(0.0, high, aero, thrust).norm();

    REQUIRE(f_low > 1.0e-3);
    REQUIRE(f_high < 1.0e-9);
    REQUIRE(f_high < f_low);
}

// ---------------------------------------------------------------------------
// Composition: forces sum, mass divides, moments are discarded.
// ---------------------------------------------------------------------------

TEST_CASE("BodySpecificForce: aero and thrust forces sum and divide by mass",
          "[BodySpecificForce]")
{
    const ConstantBodyForcePolicy aero  { -400.0,  200.0, -1000.0 };
    const ConstantBodyForcePolicy thrust{ 3400.0,    0.0,     0.0 };

    RigidBody::StateD s = makeAtmosphericState();
    s.m = 2000.0;

    const Eigen::Vector3d f_B = Simulation::BodySpecificForce_m_s2(0.0, s, aero, thrust);

    REQUIRE(f_B.x() == Approx(3000.0 / 2000.0));
    REQUIRE(f_B.y() == Approx( 200.0 / 2000.0));
    REQUIRE(f_B.z() == Approx(-1000.0 / 2000.0));
}

TEST_CASE("BodySpecificForce: independent of vehicle position and attitude for a "
          "fixed body force", "[BodySpecificForce]")
{
    // A body-frame force divided by mass is a body-frame quantity: moving the
    // vehicle must not change it.  This fails if a gravity or frame-rotation
    // term has crept in, since both depend on position.
    const ConstantBodyForcePolicy aero  { 100.0, -50.0, 25.0 };
    const ConstantBodyForcePolicy thrust{ 900.0,   0.0,  0.0 };

    RigidBody::StateD a = makeAtmosphericState(500.0);
    RigidBody::StateD b = makeExoatmosphericState(500.0);
    b.g.q = Eigen::Quaterniond(Eigen::AngleAxisd(2.1, Eigen::Vector3d(0, 1, 0)));
    b.g.R = b.g.q.toRotationMatrix();

    const Eigen::Vector3d f_a = Simulation::BodySpecificForce_m_s2(0.0, a, aero, thrust);
    const Eigen::Vector3d f_b = Simulation::BodySpecificForce_m_s2(9.0, b, aero, thrust);

    REQUIRE((f_a - f_b).norm() == Approx(0.0).margin(1.0e-15));
    REQUIRE(f_a.x() == Approx(1000.0 / 500.0));
}

TEST_CASE("BodySpecificForce: explicit force-sum overload matches the policy form",
          "[BodySpecificForce]")
{
    // The FMUs use the explicit overload with an aero force taken from a
    // Snapshot, to avoid re-evaluating the aero tables.  The two forms must
    // agree exactly.
    const FlightDynamics::DragOnlyAeroPolicy aero{ 0.5, 1.0 };
    const ConstantBodyForcePolicy            thrust{ 5000.0, 0.0, 0.0 };

    const RigidBody::StateD s = makeAtmosphericState();

    const Eigen::Vector3d viaPolicies =
        Simulation::BodySpecificForce_m_s2(1.5, s, aero, thrust);

    const Eigen::Vector3d F_sum = Simulation::PolicyBodyForce_N(1.5, s, aero)
                                + Simulation::PolicyBodyForce_N(1.5, s, thrust);
    const Eigen::Vector3d viaSum = Simulation::BodySpecificForce_m_s2(F_sum, s.m);

    REQUIRE((viaPolicies - viaSum).norm() == 0.0);
}

// ---------------------------------------------------------------------------
// Degenerate mass guard.
// ---------------------------------------------------------------------------

TEST_CASE("BodySpecificForce: non-positive mass returns zero rather than dividing",
          "[BodySpecificForce]")
{
    const ConstantBodyForcePolicy aero  { 100.0, 200.0, 300.0 };
    const ConstantBodyForcePolicy thrust{ 400.0,   0.0,   0.0 };

    RigidBody::StateD s = makeAtmosphericState();
    s.m = 0.0;

    const Eigen::Vector3d f_B = Simulation::BodySpecificForce_m_s2(0.0, s, aero, thrust);

    REQUIRE(f_B.x() == 0.0);
    REQUIRE(f_B.y() == 0.0);
    REQUIRE(f_B.z() == 0.0);
    REQUIRE(std::isfinite(f_B.norm()));

    const Eigen::Vector3d negative =
        Simulation::BodySpecificForce_m_s2(Eigen::Vector3d(1.0, 2.0, 3.0), -5.0);
    REQUIRE(negative.norm() == 0.0);
}

// ---------------------------------------------------------------------------
// Trim: in straight-and-level unaccelerated flight the non-gravitational
// forces balance weight, so |f_B| equals local gravity magnitude, and body-z
// specific force is negative (lift acts along body −z).  Acceptance criterion 3
// in miniature, without needing the full F-16 model.
// ---------------------------------------------------------------------------

TEST_CASE("BodySpecificForce: trimmed level flight reads |g| with negative body-z",
          "[BodySpecificForce]")
{
    // Straight-and-level unaccelerated flight above the equator on the ECI +X
    // axis.  Orient the body so that body z (down) points at the Earth:
    //   body x → ECI +Y,  body y → ECI −Z,  body z → ECI −X.
    RigidBody::StateD s = makeAtmosphericState(1000.0, 200.0);
    s.g.R.col(0) = Eigen::Vector3d( 0.0,  1.0,  0.0);
    s.g.R.col(1) = Eigen::Vector3d( 0.0,  0.0, -1.0);
    s.g.R.col(2) = Eigen::Vector3d(-1.0,  0.0,  0.0);
    s.g.q        = Eigen::Quaterniond(s.g.R).normalized();

    const FlightDynamics::J2GravityPolicy gravity{};
    const Eigen::Vector3d F_grav_B = gravity(s.g, s.m).f.tail<3>();

    // Sanity: with this attitude gravity acts along body +z (down).
    REQUIRE(F_grav_B.z() > 0.0);
    REQUIRE(std::abs(F_grav_B.x()) < 1.0e-6);
    REQUIRE(std::abs(F_grav_B.y()) < 1.0e-6);

    // In trim the non-gravitational forces exactly balance weight.
    const ConstantBodyForcePolicy aeroAndThrust{ -F_grav_B.x(), -F_grav_B.y(), -F_grav_B.z() };
    const FlightDynamics::ZeroPropulsionPolicy none{};

    const Eigen::Vector3d f_B = Simulation::BodySpecificForce_m_s2(0.0, s, aeroAndThrust, none);

    // |f_B| equals local gravity magnitude ...
    const double g_local = F_grav_B.norm() / s.m;
    REQUIRE(f_B.norm() == Approx(g_local).epsilon(1.0e-12));
    REQUIRE(g_local > 9.0);
    REQUIRE(g_local < 10.0);

    // ... and the z channel is negative: lift acts up, body z points down.
    // A sign error in the z channel shows up here as +g.
    REQUIRE(f_B.z() == Approx(-g_local).epsilon(1.0e-12));
}
