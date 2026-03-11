// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/InitialRotationAboutBodyAxes.h"
#include "Aetherion/FlightDynamics/Serialization/InitialRotationAboutBodyAxesJson.h"

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::InitialRotationAboutBodyAxes& w)
    {
        w.roll_rad_s = j.at("roll_rad_s").get<double>();
        w.pitch_rad_s = j.at("pitch_rad_s").get<double>();
        w.yaw_rad_s = j.at("yaw_rad_s").get<double>();
    }

    void to_json(nlohmann::json& j, const FlightDynamics::InitialRotationAboutBodyAxes& w)
    {
        j = nlohmann::json::object();
        j["roll_rad_s"] = w.roll_rad_s;
        j["pitch_rad_s"] = w.pitch_rad_s;
        j["yaw_rad_s"] = w.yaw_rad_s;
    }

} // namespace Aetherion::FlightDynamics::Serialization