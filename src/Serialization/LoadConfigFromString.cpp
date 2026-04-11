// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/Serialization/LoadConfigFromString.h"
#include "Aetherion/RigidBody/Config.h"
#include "Aetherion/Serialization/ConfigJson.h"
#include <stdexcept>
#include <vendor/nlohmann/json.hpp>

namespace Aetherion::Serialization {

    RigidBody::Config LoadConfigFromString(const std::string& jsonString)
    {
        if (jsonString.empty())
            throw std::runtime_error("LoadConfigFromString: input string is empty");

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(jsonString);
        }
        catch (const nlohmann::json::parse_error& e) {
            throw std::runtime_error(
                std::string("LoadConfigFromString: JSON syntax error: ") + e.what());
        }

        RigidBody::Config cfg{};
        try {
            ::Aetherion::Serialization::from_json(j, cfg);
        }
        catch (const std::exception& e) {
            throw std::runtime_error(
                std::string("LoadConfigFromString: failed to deserialize config: ") + e.what());
        }
        return cfg;
    }

} // namespace Aetherion::Serialization

