// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <filesystem>

#include "Aetherion/FlightDynamics/Serialization/JsonConfig.h"
#include "Aetherion/FlightDynamics/Serialization/JsonConfigNlohmannAdapter.h" // defines Json + functions

namespace Aetherion::FlightDynamics::Serialization {


    FlightDynamics::InitialPoseWGS84_NED load_initial_pose(const std::filesystem::path& path) {

        const Json root = parse_json_file(path);
        InitialPoseWGS84_NED ls{};
      //  ls.t0 = json_get_or<double>(root, "t0", ls.t0, [](const Json& j) { return json_get_number(j); });

        if (!json_has(root, "launch")) {
            throw ConfigError("initial_conditions.json must contain object 'launch'.");
        }
        const Json launch = json_at(root, "launch");

        ls.lat_deg = json_get_or<double>(launch, "lat_deg", 0.0, [](const Json& j) { return json_get_number(j); });
        ls.lon_deg = json_get_or<double>(launch, "lon_deg", 0.0, [](const Json& j) { return json_get_number(j); });
        ls.alt_m = json_get_or<double>(launch, "alt_m", 0.0, [](const Json& j) { return json_get_number(j); });
        ls.roll_deg = json_get_or<double>(launch, "roll_deg", 0.0, [](const Json& j) { return json_get_number(j); });

        // validate
        if (!(ls.lat_deg >= -90.0 && ls.lat_deg <= 90.0))  throw ConfigError("lat_deg must be in [-90,90].");
        if (!(ls.lon_deg >= -180.0 && ls.lon_deg <= 180.0)) throw ConfigError("lon_deg must be in [-180,180].");

        return ls;
    }

} // namespace Aetherion::FlightDynamics::Serialization
