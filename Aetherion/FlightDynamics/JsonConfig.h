// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
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

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "JsonAdapter.h"


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
    struct Vec3 { double x{ 0 }, y{ 0 }, z{ 0 }; };

    struct QuatWxyz {
        double w{ 1 }, x{ 0 }, y{ 0 }, z{ 0 }; // (w,x,y,z)
    };

    // NOTE: lat/lon/alt + r/p/y are "input conveniences" (e.g., from JSON).
   // Dynamics should primarily use pW and qWB; conversion can be done after parsing.

    struct InitialConditions {
        double t0{ 0.0 };

        // --- Geo/Euler (JSON input convenience) ---
        double lat_deg{ 0.0 };
        double lon_deg{ 0.0 };
        double alt_m{ 0.0 };
        double roll_deg{ 0.0 };
        double pitch_deg{ 0.0 };
        double yaw_deg{ 0.0 };

        // --- Expanded initial conditions used by the dynamics ---
        Vec3     pW{ 0,0,0 };        // ECI position
        QuatWxyz qWB{ 1,0,0,0 };     // body->ECI quaternion

        Vec3 omegaB{ 0,0,0 };
        Vec3 vB{ 0,0,0 };
        double m{ 1.0 };
    };



    template <class T, class Fn>
    T json_get_or(const Json& parent, std::string_view key, T default_value, Fn getter) {
        if (!json_has(parent, key)) return default_value;
        return getter(json_at(parent, key));
    }

    inline Vec3 parse_vec3(const Json& j) {
        const auto arr = json_get_array(j);
        if (arr.size() != 3) throw ConfigError("Vec3 must be [x,y,z].");
        return Vec3{ json_get_number(arr[0]), json_get_number(arr[1]), json_get_number(arr[2]) };
    }

    InitialConditions load_initial_conditions(const std::filesystem::path& path);

  
    // TODO: move this to .cpp file
    // ============================================================================
    // Loaders
    // ============================================================================

    // initial_conditions.json (new schema)
    // Suggested:
    // {
    //   "t0": 0.0,
    //   "launch": {
    //     "lat_deg": 41.0,
    //     "lon_deg": 29.0,
    //     "alt_m": 100.0,
    //     "roll_deg": 0.0
    //   },
    //   "omegaB": [0,0,0],
    //   "vB": [0,0,0],
    //   "m": 1.0
    // }
    //
    // You pass theta_g_fn from your existing Aetherion gravity/frames module.
    
} // namespace Aetherion::FlightDynamics
