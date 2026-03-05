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
#include "InertialParameters.h"
#include "AerodynamicParameters.h"
#include "VelocityNED.h"
#include "InitialRotationAboutBodyAxes.h"

namespace Aetherion::FlightDynamics {

    struct SimulationConfig
    {
        SimulationParameters simulation;
        PoseWGS84_NED pose;
        VelocityNED velocityNED;
        InitialRotationAboutBodyAxes initialRotationAboutBodyAxes;
        InertialParameters inertialParameters;
        AerodynamicParameters aerodynamicParameters;
    };

} // namespace Aetherion::FlightDynamics