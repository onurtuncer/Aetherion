// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/Serialization/LoadFlightSimulationConfigFromFile.h"

#include "Aetherion/FlightDynamics/FlightSimulationConfig.h"
#include "Aetherion/FlightDynamics/Serialization/FlightSimulationConfigJson.h" // <-- MUST (declares from_json)

#include <fstream>
#include <stdexcept>
#include <vendor/nlohmann/json.hpp>

namespace Aetherion::FlightDynamics::Serialization {

    FlightSimulationConfig LoadFlightSimulationConfigFromFile(const std::string& filename)
    {
        std::ifstream f(filename);
        if (!f.is_open())
            throw std::runtime_error("Cannot open config file: " + filename);

        nlohmann::json j;
        f >> j;

        FlightSimulationConfig cfg{};
        ::Aetherion::FlightDynamics::Serialization::from_json(j, cfg); 
        return cfg;
    }

} // namespace Aetherion::FlightDynamics::Serialization
