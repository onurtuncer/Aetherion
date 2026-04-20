// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "GeodeticPoseNED.h"
#include "InertialParameters.h"
#include "AerodynamicParameters.h"
#include "VelocityNED.h"
#include "BodyRates.h"

namespace Aetherion::RigidBody {

/// @brief Aggregated configuration for a single rigid body simulation run.
///
/// Collects all initial conditions and physical parameters needed to initialise
/// the six-degree-of-freedom state and the associated dynamics policies.
/// Typically populated by deserialising a JSON input file via
/// Aetherion::Serialization::LoadConfig().
struct Config
{
    GeodeticPoseNED      pose;                  ///< Initial geodetic position and attitude in the NED frame.
    VelocityNED          velocityNED;           ///< Initial velocity expressed in the local NED frame [m/s].
    BodyRates            bodyRates;             ///< Initial angular rates expressed in the body frame [rad/s].
    InertialParameters   inertialParameters;    ///< Mass and inertia properties of the rigid body.
    AerodynamicParameters aerodynamicParameters;///< Reference geometry and aerodynamic coefficients.
};

} // namespace Aetherion::RigidBody
