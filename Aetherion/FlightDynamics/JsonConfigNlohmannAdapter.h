// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// JsonConfig_NlohmannAdapter.h
// Implements the Json adapter declared in JsonConfig.h using nlohmann::json.
//
// Usage:
//   - Vendor nlohmann/json.hpp (recommended) or use package manager.
//   - Include this header in exactly ONE translation unit OR make it a .cpp.
//
// Option A (header-only, simplest):
//   #define AETHERION_JSONCFG_NLOHMANN_HEADER_ONLY
//   #include "JsonConfig_NlohmannAdapter.h"
//
// Option B (recommended, compile once):
//   Create JsonConfig_NlohmannAdapter.cpp that includes this header
//   (WITHOUT defining AETHERION_JSONCFG_NLOHMANN_HEADER_ONLY) and compile it.
//
// This implements:
//   Json parse_json_file(...)
//   json_has, json_at, json_get_number, json_get_bool, json_get_string, json_get_array
//

#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include <memory>


#include "Aetherion/FlightDynamics/JsonAdapter.h"

#include "Aetherion/FlightDynamics/JsonConfig.h" // for Aetherion::Cfg::Json declaration + ConfigError

// ---- nlohmann json ----
#include <vendor/nlohmann/json.hpp> //TODO fix this later: should not specify vendor directory explicitly

namespace Aetherion::FlightDynamics {

    // ============================================================================
    // Json opaque handle implementation
    // ============================================================================
    //
    // In JsonConfig.h we declared: struct Json;
    // Here we define it. Since this is a definition, you must ensure only one
    // definition across the program. If you want strict ODR safety, move this
    // to a .cpp and compile once.
    //
    struct Json {
        // We hold a pointer-like view to a JSON node.
        // - If owner != nullptr, this Json owns the root document.
        // - node points into owner or to an external node.
        std::shared_ptr<nlohmann::json> owner;
        const nlohmann::json* node{ nullptr };

        Json() = default;

        explicit Json(std::shared_ptr<nlohmann::json> root_owner)
            : owner(std::move(root_owner)), node(owner.get()) {
        }

        explicit Json(const nlohmann::json* n, std::shared_ptr<nlohmann::json> root_owner = {})
            : owner(std::move(root_owner)), node(n) {
        }

        const nlohmann::json& j() const {
            if (!node) throw ConfigError("Json adapter: null Json node.");
            return *node;
        }
    };

    // ============================================================================
    // Helpers
    // ============================================================================
    inline std::string to_string(std::string_view sv) { return std::string(sv.begin(), sv.end()); }

    inline void ensure_object(const Json& j, std::string_view what) {
        if (!j.j().is_object()) {
            throw ConfigError("Json adapter: expected object for " + to_string(what) + ".");
        }
    }

    inline void ensure_array(const Json& j, std::string_view what) {
        if (!j.j().is_array()) {
            throw ConfigError("Json adapter: expected array for " + to_string(what) + ".");
        }
    }

    // ============================================================================
    // Required adapter API
    // ============================================================================
    inline Json parse_json_file(const std::filesystem::path& path) {
        std::ifstream in(path);
        if (!in.is_open()) {
            throw ConfigError("Failed to open JSON file: " + path.string());
        }

        auto root = std::make_shared<nlohmann::json>();
        try {
            in >> *root;
        }
        catch (const std::exception& e) {
            throw ConfigError("Failed to parse JSON file '" + path.string() + "': " + std::string(e.what()));
        }

        return Json{ std::move(root) };
    }

    inline bool json_has(const Json& j, std::string_view key) {
        ensure_object(j, "json_has()");
        const auto& obj = j.j();
        return obj.contains(to_string(key));
    }

    inline Json json_at(const Json& j, std::string_view key) {
        ensure_object(j, "json_at()");
        const auto& obj = j.j();
        const std::string k = to_string(key);

        if (!obj.contains(k)) {
            throw ConfigError("JSON missing key: '" + k + "'");
        }

        // Preserve ownership of root document so returned Json stays valid.
        return Json{ &obj.at(k), j.owner };
    }

    inline double json_get_number(const Json& j) {
        const auto& v = j.j();
        if (!v.is_number()) {
            throw ConfigError("Json adapter: expected number.");
        }
        return v.get<double>();
    }

    inline bool json_get_bool(const Json& j) {
        const auto& v = j.j();
        if (!v.is_boolean()) {
            throw ConfigError("Json adapter: expected boolean.");
        }
        return v.get<bool>();
    }

    inline std::string json_get_string(const Json& j) {
        const auto& v = j.j();
        if (!v.is_string()) {
            throw ConfigError("Json adapter: expected string.");
        }
        return v.get<std::string>();
    }

    inline std::vector<Json> json_get_array(const Json& j) {
        ensure_array(j, "json_get_array()");
        const auto& arr = j.j();

        std::vector<Json> out;
        out.reserve(arr.size());
        for (std::size_t i = 0; i < arr.size(); ++i) {
            out.emplace_back(&arr.at(i), j.owner);
        }
        return out;
    }

} // namespace Aetherion::FlightDynamics
