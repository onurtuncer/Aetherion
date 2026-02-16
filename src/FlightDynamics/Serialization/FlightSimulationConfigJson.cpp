// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/FlightSimulationConfig.h"
#include "Aetherion/FlightDynamics/Serialization/FlightSimulationConfigJson.h"

#include "Aetherion/FlightDynamics/Serialization/SimulationParametersJson.h"
#include "Aetherion/FlightDynamics/Serialization/InitialPoseWGS84_NEDJson.h"     
#include "Aetherion/FlightDynamics/Serialization/InertialParametersJson.h"
#include "Aetherion/FlightDynamics/Serialization/AerodynamicParametersJson.h"

#include <vendor/nlohmann/json.hpp>

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightSimulationConfig& cfg)
    {
        // Fully qualify to avoid accidentally picking nlohmann::from_json
        ::Aetherion::FlightDynamics::Serialization::from_json(j.at("simulation"), cfg.simulation);
        ::Aetherion::FlightDynamics::Serialization::from_json(j.at("initialPose"), cfg.initialPose);
        ::Aetherion::FlightDynamics::Serialization::from_json(j.at("inertialParameters"), cfg.inertialParameters);
        ::Aetherion::FlightDynamics::Serialization::from_json(j.at("aerodynamicParameters"), cfg.aerodynamicParameters);
    }

    void to_json(nlohmann::json& j, const FlightSimulationConfig& cfg)
    {
        // Build sub-objects using own serializers (no ADL)
        nlohmann::json sim, pose, inert, aero;

        ::Aetherion::FlightDynamics::Serialization::to_json(sim, cfg.simulation);
        ::Aetherion::FlightDynamics::Serialization::to_json(pose, cfg.initialPose);
        ::Aetherion::FlightDynamics::Serialization::to_json(inert, cfg.inertialParameters);
        ::Aetherion::FlightDynamics::Serialization::to_json(aero, cfg.aerodynamicParameters);

        j = nlohmann::json::object();
        j["simulation"] = std::move(sim);
        j["initialPose"] = std::move(pose);
        j["inertialParameters"] = std::move(inert);
        j["aerodynamicParameters"] = std::move(aero);
    }
} // namespace Aetherion::FlightDynamics::Serialization