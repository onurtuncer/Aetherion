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

/// @brief Deserialise a RigidBody::Config from a JSON file on disk.
/// @param filename Path to the JSON configuration file.
/// @return Fully populated RigidBody::Config.
/// @throws std::runtime_error if the file cannot be opened or the JSON is malformed.
RigidBody::Config LoadConfig(const std::string& filename);

/// @brief Deserialise a RigidBody::Config from an already-parsed nlohmann::json object.
/// @param j      Parsed JSON object containing the rigid-body configuration fields.
/// @param source Optional hint shown in error messages (e.g. the originating filename).
/// @return Fully populated RigidBody::Config.
/// @throws std::runtime_error if required fields are missing or have wrong types.
RigidBody::Config LoadConfig(const nlohmann::json& j, const std::string& source = "");

} // namespace Aetherion::Serialization
