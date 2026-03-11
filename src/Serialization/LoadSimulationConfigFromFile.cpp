// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/Serialization/LoadSimulationConfigFromFile.h"

#include "Aetherion/FlightDynamics/SimulationConfig.h"
#include "Aetherion/Serialization/SimulationConfigJson.h" 

#include <fstream>
#include <stdexcept>
#include <vendor/nlohmann/json.hpp>

namespace Aetherion::Serialization {

    FlightDynamics::SimulationConfig LoadSimulationConfigFromFile(const std::string& filename)
    {
        std::ifstream f(filename);
        if (!f.is_open())
            throw std::runtime_error("Cannot open config file: " + filename);

        nlohmann::json j;
        f >> j;

        FlightDynamics::SimulationConfig cfg{};
        ::Aetherion::Serialization::from_json(j, cfg); 
        return cfg;
    }

} // namespace Aetherion::Serialization
