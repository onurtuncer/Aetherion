// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/RigidBody/VelocityNED.h"
#include "Aetherion/Serialization/VelocityNEDJson.h"

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::VelocityNED& v)
    {
        auto get = [&](const char* key, double& field)
            {
                try {
                    field = j.at(key).get<double>();
                }
                catch (const nlohmann::json::out_of_range&) {
                    throw std::runtime_error(
                        std::string("VelocityNED: missing key '") + key + "'");
                }
                catch (const nlohmann::json::type_error&) {
                    throw std::runtime_error(
                        std::string("VelocityNED: key '") + key + "' is not a number");
                }
            };

        get("north_mps", v.north_mps);
        get("east_mps", v.east_mps);
        get("down_mps", v.down_mps);
    }

     void to_json(nlohmann::json& j, const RigidBody::VelocityNED& v)
    {
        j = nlohmann::json::object();
        j["north_mps"] = v.north_mps;
        j["east_mps"] = v.east_mps;
        j["down_mps"] = v.down_mps;
    }

} // namespace Aetherion::Serialization