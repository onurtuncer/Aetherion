// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::FlightDynamics {
    struct SimulationConfig;
}

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::SimulationConfig& fsc);
    void to_json(nlohmann::json& j, const FlightDynamics::SimulationConfig& fsc);

} // namespace Aetherion::FlightDynamics::Serialization
