// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once
#include <string>
#include <vendor/nlohmann/json.hpp> 

namespace Aetherion::RigidBody {
    struct Config;
}

namespace Aetherion::Serialization {
    RigidBody::Config LoadConfig(const std::string& filename);
    RigidBody::Config LoadConfig(const nlohmann::json& j);
}