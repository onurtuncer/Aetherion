// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// Examples/DraglessSphere/Config.h
#pragma once
#include <Aetherion/FlightDynamics/RigidBodyVectorField.h>
#include <Aetherion/FlightDynamics/RigidBody6DoFStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples {

    // NASA check case 1: dragless sphere, central gravity
    using DraglessSphereVF = FlightDynamics::RigidBodyVectorField
        FlightDynamics::CentralGravityPolicy,  // gravity
        FlightDynamics::ZeroAeroPolicy,        // aero:     none
        FlightDynamics::ZeroPropulsionPolicy,  // thrust:   none
        FlightDynamics::ConstantMassPolicy     // mass:     constant
    > ;

    using DraglessSphereStepper = FlightDynamics::RigidBody6DoFStepper<DraglessSphereVF>;

    // NASA check case 2: same orbit with J2
    using J2SphereVF = FlightDynamics::RigidBodyVectorField
        FlightDynamics::J2GravityPolicy,
        FlightDynamics::ZeroAeroPolicy,
        FlightDynamics::ZeroPropulsionPolicy,
        FlightDynamics::ConstantMassPolicy
    > ;

    using J2SphereStepper = FlightDynamics::RigidBody6DoFStepper<J2SphereVF>;

    // Future: powered ascent
    // using PoweredAscentVF = FlightDynamics::RigidBodyVectorField
    //     FlightDynamics::CentralGravityPolicy,
    //     FlightDynamics::ZeroAeroPolicy,
    //     FlightDynamics::ConstantThrustPolicy,
    //     FlightDynamics::LinearBurnPolicy
    // >;

} // namespace Aetherion::Examples