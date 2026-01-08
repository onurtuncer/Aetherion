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
// Initial conditions are specified by geodetic launch-site parameters:
//   latitude, longitude, altitude, initial roll.
//
// This header is JSON-library agnostic; implement the Json adapter functions
// using your chosen JSON library (nlohmann/json, rapidjson, etc).
//

#pragma once

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "Aetherion/FlightDynamics/JsonAdapter.h"
#include "Aetherion/FlightDynamics/InitialPoseWGS84_NED.h"

namespace Aetherion::FlightDynamics {

    struct Json;

    // ============================================================================
    // Errors
    // ============================================================================
    class ConfigError final : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    inline bool is_finite(double x) noexcept { return std::isfinite(x); }

    // ============================================================================
    // Small POD types
    // ============================================================================
   /*struct Vec3 { double x{0}, y{0}, z{0}; };

    struct QuatWxyz {
        double w{ 1 }, x{ 0 }, y{ 0 }, z{ 0 }; // (w,x,y,z)
    }; */

    // NOTE: lat/lon/alt + azimuth/zenith/roll are "input conveniences" (e.g., from JSON).
   // Dynamics should primarily use pW and qWB; conversion can be done after parsing.

    template <class T, class Fn>
    T json_get_or(const Json& parent, std::string_view key, T default_value, Fn getter) {
        if (!json_has(parent, key)) return default_value;
        return getter(json_at(parent, key));
    }

   // Vec3 parse_vec3(const Json& j, std::string_view key);
    
    InitialPoseWGS84_NED load_initial_pose(const std::filesystem::path& path);
    
} // namespace Aetherion::FlightDynamics
