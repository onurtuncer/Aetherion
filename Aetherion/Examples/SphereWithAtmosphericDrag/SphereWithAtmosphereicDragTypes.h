// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// SphereWithAtmosphereicDragTypes.h
//
// Type aliases for the NASA TM-2015-218675 Atmospheric Scenario 6 simulation:
//   - J2 gravitation (WGS-84 second harmonic)
//   - Drag-only aerodynamics (DragOnlyAeroPolicy: CD, S_ref, US 1976 density)
//   - Zero thrust
//   - Constant mass
//
// Reference: NASA TM-2015-218675, Section B.1.6 / Table 30.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples::SphereWithAtmosphericDrag {

/// @brief Six-DoF vector field for the NASA Atmos-06 sphere-with-drag case.
///
/// Policy configuration:
/// - Gravity  : J2 oblateness perturbation (WGS-84: μ, Re, J2)
/// - Aero     : Drag only — F = −½ ρ(h) CD S_ref |v_B| v_B  (body frame)
/// - Thrust   : None
/// - Mass     : Constant (no propellant consumption)
using SphereWithAtmosphericDragVF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,        ///< Gravity: J2 perturbation (WGS-84).
    FlightDynamics::DragOnlyAeroPolicy,     ///< Aerodynamics: drag only, no lift.
    FlightDynamics::ZeroPropulsionPolicy,   ///< Propulsion: none.
    FlightDynamics::ConstantMassPolicy      ///< Mass: constant.
>;

/// @brief RKMK integrator stepper for @c SphereWithAtmosphericDragVF.
using SphereWithAtmosphericDragStepper = RigidBody::SixDoFStepper<SphereWithAtmosphericDragVF>;

} // namespace Aetherion::Examples::SphereWithAtmosphericDrag