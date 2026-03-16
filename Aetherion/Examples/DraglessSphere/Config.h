// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// Examples/DraglessSphere/Config.h
#pragma once
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDoFStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples::DraglessSphere {

    // NASA check case 1: dragless sphere, central gravity
    using DraglessSphereVF = RigidBody::VectorField<
        FlightDynamics::CentralGravityPolicy,  // gravity
        FlightDynamics::ZeroAeroPolicy,        // aero:     none
        FlightDynamics::ZeroPropulsionPolicy,  // thrust:   none
        FlightDynamics::ConstantMassPolicy     // mass:     constant
    > ;

    using DraglessSphereStepper = RigidBody::SixDoFStepper<DraglessSphereVF>;

   

} // namespace Aetherion::Examples