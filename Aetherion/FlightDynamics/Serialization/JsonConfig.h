// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// JsonConfig.h
// Config loader for Aetherion flight dynamics.
//
// This header is JSON-library agnostic; implement the Json adapter functions
// using your chosen JSON library (nlohmann/json, rapidjson, etc).
//

#pragma once

//#include <cmath>
#include <filesystem>
//#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
//#include <vector>

#include "Aetherion/FlightDynamics/Serialization/JsonAdapter.h"
#include "Aetherion/FlightDynamics/InitialPoseWGS84_NED.h"

namespace Aetherion::FlightDynamics::Serialization {

    struct Json;

    // ============================================================================
    // Errors
    // ============================================================================
    class ConfigError final : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    inline bool is_finite(double x) noexcept { return std::isfinite(x); }

    template <class T, class Fn>
    T json_get_or(const Json& parent, std::string_view key, T default_value, Fn getter) {
        if (!json_has(parent, key)) return default_value;
        return getter(json_at(parent, key));
    }
  
 //   FlightDynamics::InitialPoseWGS84_NED load_initial_pose(const std::filesystem::path& path); //TODO remove this!
    
} // namespace Aetherion::FlightDynamics::Serialization
