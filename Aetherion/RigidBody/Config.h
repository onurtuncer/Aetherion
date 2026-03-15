// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/RigidBody/GeodeticPoseNED.h"
#include "Aetherion/RigidBody/Parameters/Inertial.h"
#include "Aetherion/RigidBody/Parameters/Aerodynamic.h"
#include "Aetherion/RigidBody/VelocityNED.h"
#include "Aetherion/RigidBody/BodyRates.h"

namespace Aetherion::FlightDynamics {

    struct SimulationConfig
    {
        RigidBody::GeodeticPoseNED pose;
        RigidBody::VelocityNED velocityNED;
        RigidBody::BodyRates initialRotationAboutBodyAxes;  //TODO [Onur] rename this!!!
        RigidBody::Parameters::Inertial inertialParameters;
        RigidBody::Parameters::Aerodynamic aerodynamicParameters;
    };

} // namespace Aetherion::FlightDynamics