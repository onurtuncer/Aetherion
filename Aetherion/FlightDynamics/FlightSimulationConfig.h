// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "SimulationParameters.h"
#include "InitialPoseWGS84_NED.h"
#include "InertialParameters.h"
#include "AerodynamicParameters.h"

namespace Aetherion::FlightDynamics {

    struct FlightSimulationConfig
    {
        SimulationParameters simulation;
        InitialPoseWGS84_NED initialPose;
        InertialParameters inertialParameters;
        AerodynamicParameters aerodynamicParameters;
    };

} // namespace Aetherion::FlightDynamics