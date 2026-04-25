// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickWithDampingTypes.h
//
// Type aliases for the NASA TM-2015-218675 Atmospheric Scenario 3 simulation:
//   - J2 gravitation (WGS-84 second harmonic)
//   - Drag + rotary damping moments (BrickDampingAeroPolicy: CD, S, b, cbar,
//     Clp, Cmq, Cnr from NASA TM Table 5)
//   - Zero thrust
//   - Constant mass
//
// Reference: NASA TM-2015-218675, Section B.1.3 — Tables 4 & 5.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples::TumblingBrickWithDamping {

/// @brief Six-DoF vector field for the NASA Atmos-03 tumbling brick with
///        aerodynamic damping case.
///
/// Policy configuration:
/// - Gravity  : J2 oblateness perturbation (WGS-84)
/// - Aero     : Drag + rotary damping (Clp, Cmq, Cnr from NASA TM Table 5)
/// - Thrust   : None
/// - Mass     : Constant
using TumblingBrickWithDampingVF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,          ///< Gravity: J2 perturbation.
    FlightDynamics::BrickDampingAeroPolicy,   ///< Aero: drag + rate damping.
    FlightDynamics::ZeroPropulsionPolicy,     ///< Propulsion: none.
    FlightDynamics::ConstantMassPolicy        ///< Mass: constant.
>;

/// @brief RKMK integrator stepper for @c TumblingBrickWithDampingVF.
using TumblingBrickWithDampingStepper = RigidBody::SixDoFStepper<TumblingBrickWithDampingVF>;

} // namespace Aetherion::Examples::TumblingBrickWithDamping
