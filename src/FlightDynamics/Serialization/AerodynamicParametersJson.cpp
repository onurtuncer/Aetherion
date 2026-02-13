// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/FlightDynamics/AerodynamicParameters.h"
#include "Aetherion/FlightDynamics/Serialization/InertialParametersJson.h"

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::AerodynamicParameters& aero)
    {
        aero.S = j.at("S").get<double>();
        aero.CL = j.at("CL").get<double>();
        aero.CD = j.at("CD").get<double>();
        aero.CY = j.at("CY").get<double>();
        aero.Cl = j.at("Cl").get<double>();
        aero.Cm = j.at("Cm").get<double>();
        aero.Cn = j.at("Cn").get<double>();
    }

    void to_json(nlohmann::json& j, const FlightDynamics::AerodynamicParameters& aero)
    {
        j = {
            {"S",  aero.S},
            {"CL", aero.CL},
            {"CD", aero.CD},
            {"CY", aero.CY},
            {"Cl", aero.Cl},
            {"Cm", aero.Cm},
            {"Cn", aero.Cn}
        };
    }

} // namespace Aetherion::FlightDynamics::Serialization
