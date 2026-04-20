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

/// @brief Six-DoF vector field for the NASA Atmos-01 dragless sphere: J2 gravity, zero aero, zero thrust, constant mass.
///
/// Policy configuration (all WGS-84 constants):
/// - mu  = 3.986004418×10¹⁴ m³/s²
/// - Re  = 6378137.0 m
/// - J2  = 1.08262668×10⁻³
using DraglessSphereVF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,       ///< Gravity model: J2 oblateness perturbation (WGS-84).
    FlightDynamics::ZeroAeroPolicy,        ///< Aerodynamics: none (C_D = C_L = 0).
    FlightDynamics::ZeroPropulsionPolicy,  ///< Propulsion: none (zero thrust).
    FlightDynamics::ConstantMassPolicy     ///< Mass: constant (no propellant consumption).
>;

/// @brief RKMK integrator stepper instantiated for the DraglessSphereVF vector field.
using DraglessSphereStepper = RigidBody::SixDoFStepper<DraglessSphereVF>;

} // namespace Aetherion::Examples::DraglessSphere
