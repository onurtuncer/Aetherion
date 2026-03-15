// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "SimulationParameters.h"
#include "Aetherion/RigidBody/GeodeticPoseNED.h"
#include "Aetherion/RigidBody/Parameters/Inertial.h"
#include "Aetherion/RigidBody/Parameters/Aerodynamic.h"
#include "Aetherion/RigidBody/VelocityNED.h"
#include "RotationRateAboutBodyAxes.h"

namespace Aetherion::FlightDynamics {

    struct SimulationConfig
    {
        SimulationParameters simulation;
        PoseWGS84_NED pose;
        RigidBody::VelocityNED velocityNED;
        RotationRateAboutBodyAxes initialRotationAboutBodyAxes;
        RigidBody::Parameters::Inertial inertialParameters;
        RigidBody::Parameters::Aerodynamic aerodynamicParameters;
    };

} // namespace Aetherion::FlightDynamics