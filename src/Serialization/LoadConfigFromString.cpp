// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
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
            throw std::runtime_error("Config string is empty");
        nlohmann::json j = nlohmann::json::parse(jsonString);
        RigidBody::Config cfg{};
        ::Aetherion::Serialization::from_json(j, cfg);
        return cfg;
    }
} // namespace Aetherion::Serialization