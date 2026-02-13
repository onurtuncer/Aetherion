// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/InitialPoseWGS84_NED.h"
#include "Aetherion/FlightDynamics/Serialization/InitialPoseWGS84_NEDJson.h"

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, InitialPoseWGS84_NED& pose)
    {
        pose.lat_deg = j.at("lat_deg").get<double>();
        pose.lon_deg = j.at("lon_deg").get<double>();
        pose.alt_m = j.at("alt_m").get<double>();

        pose.azimuth_deg = j.at("azimuth_deg").get<double>();
        pose.zenith_deg = j.at("zenith_deg").get<double>();
        pose.roll_deg = j.at("roll_deg").get<double>();
    }

    void to_json(nlohmann::json& j, const InitialPoseWGS84_NED& pose)
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

} // namespace Aetherion::FlightDynamics::Serialization
