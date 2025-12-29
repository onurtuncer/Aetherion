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

#pragma once

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Aetherion::Cfg {

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

   
    // Expanded initial conditions used by the dynamics
    struct InitialConditions {
        double t0{ 0.0 };

        Vec3     pW{ 0,0,0 };        // ECI position
        QuatWxyz qWB{ 1,0,0,0 };     // body->ECI quaternion

        Vec3 omegaB{ 0,0,0 };
        Vec3 vB{ 0,0,0 };
        double m{ 1.0 };
    };

    // ============================================================================
    // JSON adapter interface (implement elsewhere)
    // ============================================================================
    struct Json; // opaque handle

    Json parse_json_file(const std::filesystem::path& path);

    bool        json_has(const Json& j, std::string_view key);
    Json        json_at(const Json& j, std::string_view key);
    double      json_get_number(const Json& j);
    bool        json_get_bool(const Json& j);
    std::string json_get_string(const Json& j);
    std::vector<Json> json_get_array(const Json& j);

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
    inline LaunchSiteInit load_launch_site_init(const std::filesystem::path& path) {
        const Json root = parse_json_file(path);

        LaunchSiteInit ls{};
        ls.t0 = json_get_or<double>(root, "t0", ls.t0, [](const Json& j) { return json_get_number(j); });

        if (!json_has(root, "launch")) {
            throw ConfigError("initial_conditions.json must contain object 'launch'.");
        }
        const Json launch = json_at(root, "launch");

        ls.lat_deg = json_get_or<double>(launch, "lat_deg", 0.0, [](const Json& j) { return json_get_number(j); });
        ls.lon_deg = json_get_or<double>(launch, "lon_deg", 0.0, [](const Json& j) { return json_get_number(j); });
        ls.alt_m = json_get_or<double>(launch, "alt_m", 0.0, [](const Json& j) { return json_get_number(j); });
        ls.roll_deg = json_get_or<double>(launch, "roll_deg", 0.0, [](const Json& j) { return json_get_number(j); });

        // Validate rough ranges
        if (!(ls.lat_deg >= -90.0 && ls.lat_deg <= 90.0)) throw ConfigError("lat_deg must be in [-90,90].");
        if (!(ls.lon_deg >= -180.0 && ls.lon_deg <= 180.0)) throw ConfigError("lon_deg must be in [-180,180].");
        if (!is_finite(ls.alt_m)) throw ConfigError("alt_m must be finite.");
        if (!is_finite(ls.roll_deg)) throw ConfigError("roll_deg must be finite.");

        return ls;
    }

    // Expanded IC loader (uses launch site init + your sidereal-angle function)
    inline InitialConditions load_initial_conditions(const std::filesystem::path& path,
        SiderealAngleFn theta_g_fn,
        const Wgs84& wgs = Wgs84{})
    {
        const Json root = parse_json_file(path);

        const LaunchSiteInit ls = load_launch_site_init(path);
        InitialConditions ic = expand_launch_site_init(ls, wgs, theta_g_fn);

        // Allow overrides for twist + mass in the same file
        ic.omegaB = json_get_or<Vec3>(root, "omegaB", ic.omegaB, [](const Json& j) { return parse_vec3(j); });
        ic.vB = json_get_or<Vec3>(root, "vB", ic.vB, [](const Json& j) { return parse_vec3(j); });
        ic.m = json_get_or<double>(root, "m", ic.m, [](const Json& j) { return json_get_number(j); });

        if (!(ic.m > 0.0)) throw ConfigError("Initial mass m must be > 0.");
        return ic;
    }

} // namespace Aetherion::Cfg
