// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "SimulationParameters.h"
#include "PoseWGS84_NED.h"
#include "Aetherion/RigidBody/Parameters/Inertial.h"
#include "Aetherion/RigidBody/Parameters/Aerodynamic.h"
#include "VelocityNED.h"
#include "RotationRateAboutBodyAxes.h"

namespace Aetherion::FlightDynamics {

    struct SimulationConfig
    {
        SimulationParameters simulation;
        PoseWGS84_NED pose;
        VelocityNED velocityNED;
        InitialRotationAboutBodyAxes initialRotationAboutBodyAxes;
        RigidBody::Parameters::Inertial inertialParameters;
        RigidBody::Parameters::Aerodynamic aerodynamicParameters;
    };

} // namespace Aetherion::FlightDynamics