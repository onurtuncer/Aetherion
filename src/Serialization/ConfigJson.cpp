// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/RigidBody/Config.h"
#include "Aetherion/Serialization/ConfigJson.h"

#include "Aetherion/Serialization/SimulationParametersJson.h"
#include "Aetherion/Serialization/GeodeticPoseNEDJson.h"
#include "Aetherion/Serialization/InertialParametersJson.h"
#include "Aetherion/Serialization/AerodynamicParametersJson.h"
#include "Aetherion/Serialization/VelocityNEDJson.h"
#include "Aetherion/Serialization/BodyRatesJson.h"

#include <vendor/nlohmann/json.hpp>

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::Config& cfg)
    {
        auto deserialize = [&](const char* section, auto fn)
            {
                try {
                    if (!j.contains(section))
                        throw std::runtime_error(
                            std::string("missing top-level key '") + section + "'");
                    fn(j.at(section));
                }
                catch (const std::exception& e) {
                    throw std::runtime_error(
                        std::string("Config[") + section + "]: " + e.what());
                }
            };

        // Fully qualify to avoid accidentally picking nlohmann::from_json
        deserialize("pose", [&](const auto& v) { ::Aetherion::Serialization::from_json(v, cfg.pose); });
        deserialize("velocityNED", [&](const auto& v) { ::Aetherion::Serialization::from_json(v, cfg.velocityNED); });
        deserialize("bodyRates", [&](const auto& v) { ::Aetherion::Serialization::from_json(v, cfg.bodyRates); });
        deserialize("inertialParameters", [&](const auto& v) { ::Aetherion::Serialization::from_json(v, cfg.inertialParameters); });
        deserialize("aerodynamicParameters", [&](const auto& v) { ::Aetherion::Serialization::from_json(v, cfg.aerodynamicParameters); });

        // Optional: altitude-varying wind shear
        if (j.contains("windShear")) {
            const auto& ws = j.at("windShear");
            auto get_ws = [&](const char* key, double& field) {
                if (ws.contains(key)) field = ws.at(key).get<double>();
            };
            get_ws("north_ref_mps", cfg.windShear.north_ref_mps);
            get_ws("east_ref_mps",  cfg.windShear.east_ref_mps);
            get_ws("down_ref_mps",  cfg.windShear.down_ref_mps);
            get_ws("h_ref_m",       cfg.windShear.h_ref_m);
            get_ws("shear_exp",     cfg.windShear.shear_exp);
        }

        // Optional: ambient wind in NED frame (defaults to calm if absent)
        if (j.contains("wind")) {
            const auto& w = j.at("wind");
            auto get_opt = [&](const char* key, double& field) {
                if (w.contains(key)) field = w.at(key).get<double>();
            };
            get_opt("north_mps", cfg.wind.north_mps);
            get_opt("east_mps",  cfg.wind.east_mps);
            get_opt("down_mps",  cfg.wind.down_mps);
        }
    }

    //void from_json(const nlohmann::json& j, RigidBody::Config& cfg)
    //{
    //    // Fully qualify to avoid accidentally picking nlohmann::from_json
    //    ::Aetherion::Serialization::from_json(j.at("pose"), cfg.pose);
    //    ::Aetherion::Serialization::from_json(j.at("velocityNED"), cfg.velocityNED);
    //    ::Aetherion::Serialization::from_json(j.at("bodyRates"), cfg.bodyRates);
    //    ::Aetherion::Serialization::from_json(j.at("inertialParameters"), cfg.inertialParameters);
    //    ::Aetherion::Serialization::from_json(j.at("aerodynamicParameters"), cfg.aerodynamicParameters);
    //}

    void to_json(nlohmann::json& j, const RigidBody::Config& cfg)
    {
        nlohmann::json pose, vel, rot, inert, aero;
        to_json(pose, cfg.pose);
        ::Aetherion::Serialization::to_json(vel, cfg.velocityNED);
        ::Aetherion::Serialization::to_json(rot, cfg.bodyRates);
        ::Aetherion::Serialization::to_json(inert, cfg.inertialParameters);
        ::Aetherion::Serialization::to_json(aero, cfg.aerodynamicParameters);

        j = nlohmann::json::object();
        j["pose"] = std::move(pose);
        j["velocityNED"] = std::move(vel);
        j["bodyRates"] = std::move(rot);
        j["inertialParameters"] = std::move(inert);
        j["aerodynamicParameters"] = std::move(aero);
    }
} // namespace Aetherion::Serialization
