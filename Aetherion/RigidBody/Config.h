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

    struct Config
    {
        GeodeticPoseNED pose;
        VelocityNED velocityNED;
        BodyRates bodyRates;  
        InertialParameters inertialParameters;
        AerodynamicParameters aerodynamicParameters;
    };

} // namespace Aetherion::RigidBody
