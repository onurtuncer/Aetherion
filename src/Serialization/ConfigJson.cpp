// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
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
        // Fully qualify to avoid accidentally picking nlohmann::from_json
        ::Aetherion::Serialization::from_json(j.at("pose"), cfg.pose);
        ::Aetherion::Serialization::from_json(j.at("velocityNED"), cfg.velocityNED);
        ::Aetherion::Serialization::from_json(j.at("initialRotationAboutBodyAxes"), cfg.initialRotationAboutBodyAxes);
        ::Aetherion::Serialization::from_json(j.at("inertialParameters"), cfg.inertialParameters);
        ::Aetherion::Serialization::from_json(j.at("aerodynamicParameters"), cfg.aerodynamicParameters);
    }

    void to_json(nlohmann::json& j, const RigidBody::Config& cfg)
    {
        nlohmann::json pose, vel, rot, inert, aero;
        to_json(pose, cfg.pose);
        ::Aetherion::Serialization::to_json(vel, cfg.velocityNED);
        ::Aetherion::Serialization::to_json(rot, cfg.initialRotationAboutBodyAxes);
        ::Aetherion::Serialization::to_json(inert, cfg.inertialParameters);
        ::Aetherion::Serialization::to_json(aero, cfg.aerodynamicParameters);

        j = nlohmann::json::object();
        j["pose"] = std::move(pose);
        j["velocityNED"] = std::move(vel);
        j["initialRotationAboutBodyAxes"] = std::move(rot);
        j["inertialParameters"] = std::move(inert);
        j["aerodynamicParameters"] = std::move(aero);
    }
} // namespace Aetherion::Serialization