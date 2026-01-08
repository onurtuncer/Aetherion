// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/JsonConfig.h"
#include "Aetherion/FlightDynamics/JsonConfigNlohmannAdapter.h" // defines Json + functions

namespace Aetherion::FlightDynamics {

 /*   Vec3 parse_vec3(const Json& parent, std::string_view key)
    {
        const Json a = json_at(parent, key);          // OK: Json is complete in this TU
        const std::size_t n = json_array_size(a);
        if (n != 3) throw ConfigError("Expected array size 3 for '" + std::string(key) + "'");

        return Vec3{
            json_get_number(json_array_at(a, 0)),
            json_get_number(json_array_at(a, 1)),
            json_get_number(json_array_at(a, 2))
        };
    } */

    InitialPoseWGS84_NED load_initial_pose(const std::filesystem::path& path) {

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

   /* static InitialConditions load_initial_conditions(const std::filesystem::path& path,
        SiderealAngleFn theta_g_fn,
        const Wgs84& wgs)
    {
        const Json root = parse_json_file(path);

        const LaunchSiteInit ls = load_launch_site_init(path);
        InitialConditions ic = expand_launch_site_init(ls, wgs, theta_g_fn);

        ic.omegaB = json_get_or<Vec3>(root, "omegaB", ic.omegaB, [](const Json& j) { return parse_vec3(j); });
        ic.vB = json_get_or<Vec3>(root, "vB", ic.vB, [](const Json& j) { return parse_vec3(j); });
        ic.m = json_get_or<double>(root, "m", ic.m, [](const Json& j) { return json_get_number(j); });

        if (!(ic.m > 0.0)) throw ConfigError("Initial mass m must be > 0.");
        return ic;
    } */

} // namespace Aetherion::FlightDynamics
