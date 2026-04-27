// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>

namespace Aetherion::Examples::DroppedSphereSteadyWind {

/// @brief Six-DoF vector field for the NASA Atmos-07 sphere-in-steady-wind case.
///
/// J2 gravity + wind-aware drag (SteadyWindDragPolicy) + no thrust + constant mass.
using DroppedSphereSteadyWindVF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,
    FlightDynamics::SteadyWindDragPolicy,
    FlightDynamics::ZeroPropulsionPolicy,
    FlightDynamics::ConstantMassPolicy
>;

using DroppedSphereSteadyWindStepper =
    RigidBody::SixDoFStepper<DroppedSphereSteadyWindVF>;

} // namespace Aetherion::Examples::DroppedSphereSteadyWind
