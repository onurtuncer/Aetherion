// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once
#include <string>
#include "Aetherion/FlightDynamics/FlightSimulationConfig.h"

namespace Aetherion::FlightDynamics::Serialization {
    FlightSimulationConfig LoadFlightSimulationConfigFromFile(const std::string& filename);
}