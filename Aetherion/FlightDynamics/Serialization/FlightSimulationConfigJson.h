// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/FlightDynamics/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::FlightDynamics {
    struct FlightSimulationConfig;
}

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::FlightSimulationConfig& fsc);
    void to_json(nlohmann::json& j, const FlightDynamics::FlightSimulationConfig& fsc);

} // namespace Aetherion::FlightDynamics::Serialization
