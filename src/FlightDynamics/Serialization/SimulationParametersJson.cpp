// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/SimulationParameters.h"
#include "Aetherion/FlightDynamics/Serialization/SimulationParametersJson.h"

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::SimulationParameters& sp)
    {
        sp.startTime = j.at("startTime").get<double>();
        sp.duration = j.at("duration").get<double>();
    }

    void to_json(nlohmann::json& j, const FlightDynamics::SimulationParameters& sp)
    {
        j = {
            {"startTime",  sp.startTime},
            {"duration", sp.duration}
        };
    }

} // namespace Aetherion::FlightDynamics::Serialization