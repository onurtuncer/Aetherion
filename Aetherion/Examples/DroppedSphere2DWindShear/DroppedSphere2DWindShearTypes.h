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
#include <Aetherion/FlightDynamics/WindModels.h>

namespace Aetherion::Examples::DroppedSphere2DWindShear {

/// @brief Wind-aware drag with a linear altitude shear profile.
using ShearWindPolicy = FlightDynamics::WindAwareDragPolicy<FlightDynamics::LinearWindShear>;

/// @brief Six-DoF VF for NASA Atmos-08: J2 gravity + altitude-shear wind drag.
using DroppedSphere2DWindShearVF = RigidBody::VectorField<
    FlightDynamics::J2GravityPolicy,
    ShearWindPolicy,
    FlightDynamics::ZeroPropulsionPolicy,
    FlightDynamics::ConstantMassPolicy
>;

using DroppedSphere2DWindShearStepper =
    RigidBody::SixDoFStepper<DroppedSphere2DWindShearVF>;

} // namespace Aetherion::Examples::DroppedSphere2DWindShear
