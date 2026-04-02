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

    // -------------------------------------------------------------------------
    // Internal helper — single deserialization path shared by both overloads.
    // The 'source' string is used only to build the error message so callers
    // always know where the bad JSON came from.
    // -------------------------------------------------------------------------
    static RigidBody::Config LoadConfigImpl(const nlohmann::json& j,
        const std::string& source)
    {
        RigidBody::Config cfg{};
        try {
            ::Aetherion::Serialization::from_json(j, cfg);
        }
        catch (const std::exception& e) {
            throw std::runtime_error(
                "Failed to deserialize config from '" + source + "': " + e.what());
        }
        return cfg;
    }

    // -------------------------------------------------------------------------
    // Load from file path
    // -------------------------------------------------------------------------
    RigidBody::Config LoadConfig(const std::string& filename)
    {
        std::ifstream f(filename);
        if (!f.is_open())
            throw std::runtime_error(
                "Cannot open config file: '" + filename + "'");

        nlohmann::json j;
        try {
            f >> j;
        }
        catch (const nlohmann::json::parse_error& e) {
            throw std::runtime_error(
                "JSON syntax error in '" + filename + "': " + e.what());
        }

        return LoadConfigImpl(j, filename);
    }

    // -------------------------------------------------------------------------
    // Load from an already-parsed JSON object.
    // The optional 'source' hint is forwarded into any error message so that
    // callers who know the origin (e.g. a filename they parsed earlier) can
    // pass it in for clearer diagnostics.
    // -------------------------------------------------------------------------
    RigidBody::Config LoadConfig(const nlohmann::json& j,
        const std::string& source)
    {
        return LoadConfigImpl(j, source);
    }

} // namespace Aetherion::Serialization

//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
////
//// SPDX - License - Identifier: MIT
//// License - Filename: LICENSE
//// ------------------------------------------------------------------------------
//
//#include "Aetherion/Serialization/LoadConfig.h"
//
//#include "Aetherion/RigidBody/Config.h"
//#include "Aetherion/Serialization/ConfigJson.h" 
//
//#include <fstream>
//#include <stdexcept>
//#include <vendor/nlohmann/json.hpp>
//
//namespace Aetherion::Serialization {
//
//    RigidBody::Config LoadConfig(const std::string& filename)
//    {
//        std::ifstream f(filename);
//        if (!f.is_open())
//            throw std::runtime_error("Cannot open config file: " + filename);
//
//        nlohmann::json j;
//        try {
//            f >> j;
//        }
//        catch (const nlohmann::json::parse_error& e) {
//            throw std::runtime_error(
//                "JSON syntax error in '" + filename + "': " + e.what());
//        }
//
//        RigidBody::Config cfg{};
//        try {
//            ::Aetherion::Serialization::from_json(j, cfg);
//        }
//        catch (const std::exception& e) {
//            throw std::runtime_error(
//                "Failed to deserialize config from '" + filename + "': " + e.what());
//        }
//        return cfg;
//    }
//
//    RigidBody::Config LoadConfig(const nlohmann::json& j)
//    {
//        RigidBody::Config cfg{};
//        try {
//            ::Aetherion::Serialization::from_json(j, cfg);
//        }
//        catch (const std::exception& e) {
//            throw std::runtime_error(
//                std::string("Failed to deserialize config from JSON object: ") + e.what());
//        }
//        return cfg;
//    }
//
//} // namespace Aetherion::Serialization
