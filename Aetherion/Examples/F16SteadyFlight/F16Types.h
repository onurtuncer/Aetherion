// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16Types.h
//
// Type aliases for the F-16 Scenario 11 (steady straight-and-level flight)
// six-DoF simulation.
//
//   Gravity:     J2 oblateness gravity
//   Aerodynamics: F16AeroPolicy — DAVE-ML table-driven (DAVEMLAeroModel)
//   Propulsion:  F16PropPolicy  — DAVE-ML table-driven (DAVEMLPropModel)
//   Mass:        ConstantMassPolicy (fixed fuel state, no burn model)
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16AeroPolicy.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16PropPolicy.h>

namespace Aetherion::Examples::F16SteadyFlight {

/// @brief Six-DoF VectorField for the F-16 Scenario 11:
///        J2 gravity + DAVE-ML aero + DAVE-ML prop + constant mass.
using F16VF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,
    FlightDynamics::F16AeroPolicy,
    FlightDynamics::F16PropPolicy,
    FlightDynamics::ConstantMassPolicy
>;

/// @brief RKMK Radau IIA stepper for the F-16 VectorField.
using F16Stepper = RigidBody::SixDoFStepper<F16VF>;

} // namespace Aetherion::Examples::F16SteadyFlight
