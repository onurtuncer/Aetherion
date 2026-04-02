// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/RigidBody/GeodeticPoseNED.h"
#include "Aetherion/Serialization/GeodeticPoseNEDJson.h"

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::GeodeticPoseNED& pose)
    {
        auto get = [&](const char* key, double& field)
            {
                try {
                    field = j.at(key).get<double>();
                }
                catch (const nlohmann::json::out_of_range&) {
                    throw std::runtime_error(
                        std::string("GeodeticPoseNED: missing key '") + key + "'");
                }
                catch (const nlohmann::json::type_error&) {
                    throw std::runtime_error(
                        std::string("GeodeticPoseNED: key '") + key + "' is not a number");
                }
            };

        get("lat_deg", pose.lat_deg);
        get("lon_deg", pose.lon_deg);
        get("alt_m", pose.alt_m);
        get("azimuth_deg", pose.azimuth_deg);
        get("zenith_deg", pose.zenith_deg);
        get("roll_deg", pose.roll_deg);
    }

    void to_json(nlohmann::json& j, const RigidBody::GeodeticPoseNED& pose)
    {
        j = {
            {"lat_deg", pose.lat_deg},
            {"lon_deg", pose.lon_deg},
            {"alt_m",   pose.alt_m},

            {"azimuth_deg", pose.azimuth_deg},
            {"zenith_deg",  pose.zenith_deg},
            {"roll_deg",    pose.roll_deg}
        };
    }

} // namespace Aetherion::Serialization
