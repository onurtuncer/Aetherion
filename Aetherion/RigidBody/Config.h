// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "GeodeticPoseNED.h"
#include "Parameters/Inertial.h"
#include "Parameters/Aerodynamic.h"
#include "VelocityNED.h"
#include "BodyRates.h"

namespace Aetherion::RigidBody {

    struct Config
    {
        GeodeticPoseNED pose;
        VelocityNED velocityNED;
        BodyRates initialRotationAboutBodyAxes;  //TODO [Onur] rename this!!!
        Parameters::Inertial inertialParameters;
        Parameters::Aerodynamic aerodynamicParameters;
    };

} // namespace Aetherion::RigidBody