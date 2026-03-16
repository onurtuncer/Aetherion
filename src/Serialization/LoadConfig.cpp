// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/Serialization/LoadConfig.h"

#include "Aetherion/RigidBody/Config.h"
#include "Aetherion/Serialization/ConfigJson.h" 

#include <fstream>
#include <stdexcept>
#include <vendor/nlohmann/json.hpp>

namespace Aetherion::Serialization {

    RigidBody::Config LoadConfig(const std::string& filename)
    {
        std::ifstream f(filename);
        if (!f.is_open())
            throw std::runtime_error("Cannot open config file: " + filename);

        nlohmann::json j;
        f >> j;

        RigidBody::Config cfg{};
        ::Aetherion::Serialization::from_json(j, cfg); 
        return cfg;
    }

    RigidBody::Config LoadConfig(const nlohmann::json& j)
    {
        RigidBody::Config cfg{};
        ::Aetherion::Serialization::from_json(j, cfg);
        return cfg;
    }

} // namespace Aetherion::Serialization
