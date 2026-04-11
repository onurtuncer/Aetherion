// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/RigidBody/BodyRates.h"
#include "Aetherion/Serialization/BodyRatesJson.h"

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::BodyRates& w)
    {
        auto get = [&](const char* key, double& field)
            {
                try {
                    field = j.at(key).get<double>();
                }
                catch (const nlohmann::json::out_of_range&) {
                    throw std::runtime_error(
                        std::string("BodyRates: missing key '") + key + "'");
                }
                catch (const nlohmann::json::type_error&) {
                    throw std::runtime_error(
                        std::string("BodyRates: key '") + key + "' is not a number");
                }
            };

        get("roll_rad_s", w.roll_rad_s);
        get("pitch_rad_s", w.pitch_rad_s);
        get("yaw_rad_s", w.yaw_rad_s);
    }

    void to_json(nlohmann::json& j, const RigidBody::BodyRates& w)
    {
        j = nlohmann::json::object();
        j["roll_rad_s"] = w.roll_rad_s;
        j["pitch_rad_s"] = w.pitch_rad_s;
        j["yaw_rad_s"] = w.yaw_rad_s;
    }

} // namespace Aetherion::Serialization