

// Aetherion/FlightDynamics/Serialization/LoadSimulationConfigFromString.h
#pragma once
#include <string>
#include "Aetherion/FlightDynamics/SimulationConfig.h"

namespace Aetherion::FlightDynamics::Serialization {
    SimulationConfig LoadSimulationConfigFromString(const std::string& json_text);
}