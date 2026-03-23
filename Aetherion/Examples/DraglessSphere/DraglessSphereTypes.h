// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DraglessSphereTypes.h
//
// Type aliases for the NASA TM-2015-218675 Atmospheric Scenario 1 simulation:
//   - J2 gravitation (WGS-84 second harmonic)
//   - Zero aerodynamic forces (dragless, no lift, no moments)
//   - Zero thrust
//   - Constant mass
//
// Reference: NASA TM-2015-218675, Section B.1.1 / Table 25.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples::DraglessSphere {

    // -------------------------------------------------------------------------
    // NASA Atmos-01: dragless sphere, J2 gravitation, no aero, no thrust,
    // constant mass.
    //
    // J2GravityPolicy uses:
    //   mu  = 3.986004418e14  m^3/s^2   (WGS-84 gravitational parameter)
    //   Re  = 6378137.0       m          (WGS-84 semi-major axis)
    //   J2  = 1.08262668e-3              (WGS-84 second zonal harmonic)
    // -------------------------------------------------------------------------
    using DraglessSphereVF = RigidBody::VectorField<
        FlightDynamics::J2GravityPolicy,       // gravity:  J2 (WGS-84)
        FlightDynamics::ZeroAeroPolicy,        // aero:     none (CD=0)
        FlightDynamics::ZeroPropulsionPolicy,  // thrust:   none
        FlightDynamics::ConstantMassPolicy     // mass:     constant
    >;

    using DraglessSphereStepper = RigidBody::SixDoFStepper<DraglessSphereVF>;

} // namespace Aetherion::Examples::DraglessSphere