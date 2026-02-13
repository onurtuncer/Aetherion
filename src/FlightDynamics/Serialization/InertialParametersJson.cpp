// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------


#include "Aetherion/FlightDynamics/InertialParameters.h"
#include "Aetherion/FlightDynamics/Serialization/InertialParametersJson.h"

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::InertialParameters& ip)
    {
        ip.mass_kg = j.at("mass_kg").get<double>();

        const auto& I = j.at("inertia_kgm2");
        ip.Ixx = I.at("Ixx").get<double>();
        ip.Iyy = I.at("Iyy").get<double>();
        ip.Izz = I.at("Izz").get<double>();
        ip.Ixy = I.at("Ixy").get<double>();
        ip.Iyz = I.at("Iyz").get<double>();
        ip.Ixz = I.at("Ixz").get<double>();

        const auto& r = j.at("body_origin_wrt_cog_m");
        ip.xbar_m = r.at("x").get<double>();
        ip.ybar_m = r.at("y").get<double>();
        ip.zbar_m = r.at("z").get<double>();
    }

    void to_json(nlohmann::json& j, const FlightDynamics::InertialParameters& ip)
    {
        j = {
            {"mass_kg", ip.mass_kg},
            {"inertia_kgm2", {
                {"Ixx", ip.Ixx},
                {"Iyy", ip.Iyy},
                {"Izz", ip.Izz},
                {"Ixy", ip.Ixy},
                {"Iyz", ip.Iyz},
                {"Ixz", ip.Ixz}
            }},
            {"body_origin_wrt_cog_m", {
                {"x", ip.xbar_m},
                {"y", ip.ybar_m},
                {"z", ip.zbar_m}
            }}
        };
    }

} // namespace Aetherion::FlightDynamics::Serialization
