// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/RigidBody/AerodynamicParameters.h"
#include "Aetherion/Serialization/InertialParametersJson.h"

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::AerodynamicParameters& aero)
    {
        auto get = [&](const char* key, double& field)
            {
                try {
                    field = j.at(key).get<double>();
                }
                catch (const nlohmann::json::out_of_range&) {
                    throw std::runtime_error(
                        std::string("AerodynamicParameters: missing key '") + key + "'");
                }
                catch (const nlohmann::json::type_error&) {
                    throw std::runtime_error(
                        std::string("AerodynamicParameters: key '") + key + "' is not a number");
                }
            };

        get("S", aero.S);
        get("CL", aero.CL);
        get("CD", aero.CD);
        get("CY", aero.CY);
        get("Cl", aero.Cl);
        get("Cm", aero.Cm);
        get("Cn", aero.Cn);
    }

    void to_json(nlohmann::json& j, const RigidBody::AerodynamicParameters& aero)
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

} // namespace Aetherion::Serialization
