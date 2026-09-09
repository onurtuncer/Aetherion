// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// BodySpecificForce.h
//
// Body-frame specific force at the centre of gravity — the non-gravitational
// acceleration an ideal accelerometer mounted at the CG would sense.
//
//   f_B = ( sum of non-gravitational body forces ) / m
//       = ( F_aero + F_thrust + ... ) / m                            [m/s²]
//
// This is the quantity an inertial measurement unit measures, and it is NOT
// the acceleration of the vehicle: gravitation is excluded entirely, so a body
// in free fall reads exactly zero.
//
// Why the force-sum form
// ─────────────────────────────────────────────────────────────────────────────
// The kinematically equivalent definition
//
//   f_B = R_BI ( a_I − g_gravitation )
//
// requires subtracting mass-attraction gravity only — never plumb-bob gravity,
// because the centrifugal term is something a real strapdown accelerometer
// genuinely does sense.  The force-sum form used here carries no gravity term
// at all, so it cannot get that subtraction wrong.  Aetherion's gravity
// policies (CentralGravityPolicy, J2GravityPolicy, RocketGravityPolicy) are
// pure mass-attraction models and are simply never consulted here.
//
// Reference point
// ─────────────────────────────────────────────────────────────────────────────
// F/m is the specific force at the **centre of gravity**, where Newton's second
// law F = m·a_CG holds exactly.  This is true irrespective of where the body
// frame's origin sits (the two-stage rocket integrates about the moment
// reference centre, which is DXCG metres aft of the CG); the origin choice
// affects moment bookkeeping, not the force sum.
//
// An accelerometer mounted a lever arm r from the CG additionally senses the
// transport terms omega_dot × r + omega × (omega × r).  Those are not applied
// here — callers wanting an off-CG pickoff must add them, which needs
// omega_dot from the vector field.
//
// Frame and units
// ─────────────────────────────────────────────────────────────────────────────
// Body axes, matching the aero/propulsion wrench convention used throughout
// Aetherion: x forward, y right, z down.  Units m/s².
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/RigidBody/State.h>

#include <Eigen/Dense>

namespace Aetherion::Simulation {

    // ─────────────────────────────────────────────────────────────────────────
    // PolicyBodyForce_N
    //
    // Evaluates an aerodynamic or propulsion policy at the given state and
    // returns just its body-frame force [N] — wrench.f(3:5) under the
    // Featherstone [M; F] storage convention.  The moment components are
    // discarded; they do not contribute to specific force at the CG.
    //
    // Parameters
    //   t  – simulation time                                              [s]
    //   s  – rigid-body state (s.g = SE3 pose, s.nu_B = [omega_B; v_B], s.m)
    //   p  – policy instance, normally taken from the VectorField the
    //        integrator holds so that the reported force is computed with
    //        exactly the model and settings that were integrated
    // ─────────────────────────────────────────────────────────────────────────
    template <FlightDynamics::AeroPolicy Pol>
    [[nodiscard]]
    Eigen::Vector3d PolicyBodyForce_N(
        double                   t,
        const RigidBody::StateD& s,
        const Pol&               p)
    {
        return p(s.g, s.nu_B, s.m, t).f.tail<3>();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // BodySpecificForce_m_s2  (explicit force sum)
    //
    // Divides an already-summed non-gravitational body force by mass.  Use this
    // overload when the caller already holds the individual force terms (e.g.
    // an aero force taken from a Snapshot) and would rather not re-evaluate the
    // policies.
    //
    // The caller is responsible for the sum containing every non-gravitational
    // force the vehicle model carries — and no gravity term.
    //
    // Returns the zero vector for a non-positive mass rather than dividing;
    // that state is unphysical and only reachable before initialisation.
    // ─────────────────────────────────────────────────────────────────────────
    [[nodiscard]]
    inline Eigen::Vector3d BodySpecificForce_m_s2(
        const Eigen::Vector3d& nonGravitationalBodyForce_N,
        double                 mass_kg)
    {
        if (mass_kg <= 0.0)
            return Eigen::Vector3d::Zero();

        return nonGravitationalBodyForce_N / mass_kg;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // BodySpecificForce_m_s2  (aero + propulsion policies)
    //
    // Evaluates both policies at the given state and returns
    // (F_aero + F_thrust) / m in body axes.  Gravity is structurally excluded:
    // no gravity policy is taken, so none can be added by accident.
    //
    // For a vehicle with no propulsion, pass FlightDynamics::ZeroPropulsionPolicy{};
    // for one with no aerodynamics, FlightDynamics::ZeroAeroPolicy{}.
    // ─────────────────────────────────────────────────────────────────────────
    template <FlightDynamics::AeroPolicy       AeroPol,
              FlightDynamics::PropulsionPolicy PropPol>
    [[nodiscard]]
    Eigen::Vector3d BodySpecificForce_m_s2(
        double                   t,
        const RigidBody::StateD& s,
        const AeroPol&           aero,
        const PropPol&           thrust)
    {
        const Eigen::Vector3d F_B = PolicyBodyForce_N(t, s, aero)
                                  + PolicyBodyForce_N(t, s, thrust);
        return BodySpecificForce_m_s2(F_B, s.m);
    }

} // namespace Aetherion::Simulation
