// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------


#include "Aetherion/FlightDynamics/InitialVelocityNED.h"
#include "Aetherion/FlightDynamics/Serialization/InitialVelocityNEDJson.h"

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::InitialVelocityNED& v)
    {
        v.north_mps = j.at("north_mps").get<double>();
        v.east_mps = j.at("east_mps").get<double>();
        v.down_mps = j.at("down_mps").get<double>();
    }

    void to_json(nlohmann::json& j, const FlightDynamics::InitialVelocityNED& v)
    {
        j = nlohmann::json::object();
        j["north_mps"] = v.north_mps;
        j["east_mps"] = v.east_mps;
        j["down_mps"] = v.down_mps;
    }

} // namespace Aetherion::FlightDynamics::Serialization