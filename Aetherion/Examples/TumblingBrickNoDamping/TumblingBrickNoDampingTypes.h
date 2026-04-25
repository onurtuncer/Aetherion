// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickNoDampingTypes.h
//
// Type aliases for the NASA TM-2015-218675 Atmospheric Scenario 2 simulation:
//   - J2 gravitation (WGS-84 second harmonic)
//   - Zero aerodynamics (no drag, no moments — free tumble)
//   - Zero thrust
//   - Constant mass
//
// Reference: NASA TM-2015-218675, Section B.1.2 / Table 26.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples::TumblingBrickNoDamping {

/// @brief Six-DoF vector field for the NASA Atmos-02 tumbling brick case.
///
/// Policy configuration:
/// - Gravity  : J2 oblateness perturbation (WGS-84: μ, Re, J2)
/// - Aero     : None — free torque-free tumble (Euler equations only)
/// - Thrust   : None
/// - Mass     : Constant (no propellant consumption)
using TumblingBrickNoDampingVF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,       ///< Gravity: J2 perturbation (WGS-84).
    FlightDynamics::ZeroAeroPolicy,        ///< Aerodynamics: none (no damping).
    FlightDynamics::ZeroPropulsionPolicy,  ///< Propulsion: none.
    FlightDynamics::ConstantMassPolicy     ///< Mass: constant.
>;

/// @brief RKMK integrator stepper for @c TumblingBrickNoDampingVF.
using TumblingBrickNoDampingStepper = RigidBody::SixDoFStepper<TumblingBrickNoDampingVF>;

} // namespace Aetherion::Examples::TumblingBrickNoDamping
