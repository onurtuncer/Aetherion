// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/SimulationConfig.h"
#include "Aetherion/Serialization/SimulationConfigJson.h"

#include "Aetherion/Serialization/SimulationParametersJson.h"
#include "Aetherion/Serialization/PoseWGS84_NEDJson.h"     
#include "Aetherion/Serialization/InertialParametersJson.h"
#include "Aetherion/Serialization/AerodynamicParametersJson.h"
#include "Aetherion/Serialization/VelocityNEDJson.h"
#include "Aetherion/Serialization/InitialRotationAboutBodyAxesJson.h"

#include <vendor/nlohmann/json.hpp>

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::SimulationConfig& cfg)
    {
        // Fully qualify to avoid accidentally picking nlohmann::from_json
        ::Aetherion::Serialization::from_json(j.at("simulation"), cfg.simulation);
        ::Aetherion::Serialization::from_json(j.at("pose"), cfg.pose);
        ::Aetherion::Serialization::from_json(j.at("velocityNED"), cfg.velocityNED);
        ::Aetherion::Serialization::from_json(j.at("initialRotationAboutBodyAxes"), cfg.initialRotationAboutBodyAxes);
        ::Aetherion::Serialization::from_json(j.at("inertialParameters"), cfg.inertialParameters);
        ::Aetherion::Serialization::from_json(j.at("aerodynamicParameters"), cfg.aerodynamicParameters);
    }

    void to_json(nlohmann::json& j, const FlightDynamics::SimulationConfig& cfg)
    {
        nlohmann::json sim, pose, vel, rot, inert, aero;
        ::Aetherion::Serialization::to_json(sim, cfg.simulation);
        to_json(pose, cfg.pose);
        ::Aetherion::Serialization::to_json(vel, cfg.velocityNED);
        ::Aetherion::Serialization::to_json(rot, cfg.initialRotationAboutBodyAxes);
        ::Aetherion::Serialization::to_json(inert, cfg.inertialParameters);
        ::Aetherion::Serialization::to_json(aero, cfg.aerodynamicParameters);

        j = nlohmann::json::object();
        j["simulation"] = std::move(sim);
        j["pose"] = std::move(pose);
        j["velocityNED"] = std::move(vel);
        j["initialRotationAboutBodyAxes"] = std::move(rot);
        j["inertialParameters"] = std::move(inert);
        j["aerodynamicParameters"] = std::move(aero);
    }
} // namespace Aetherion::Serialization