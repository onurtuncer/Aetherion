// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/RigidBody/AerodynamicParameters.h"
#include "Aetherion/Serialization/InertialParametersJson.h"

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::AerodynamicParameters& aero)
    {
        // Required fields — throw if absent or wrong type.
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

        // Optional fields — keep the struct default (0.0) when absent.
        auto get_opt = [&](const char* key, double& field)
        {
            if (!j.contains(key)) return;
            try {
                field = j.at(key).get<double>();
            }
            catch (const nlohmann::json::type_error&) {
                throw std::runtime_error(
                    std::string("AerodynamicParameters: key '") + key + "' is not a number");
            }
        };

        // ── Required: present in every scenario config ────────────────────
        get("S",  aero.S);
        get("CL", aero.CL);
        get("CD", aero.CD);
        get("CY", aero.CY);
        get("Cl", aero.Cl);
        get("Cm", aero.Cm);
        get("Cn", aero.Cn);

        // ── Optional: reference lengths and rotary damping derivatives ────
        get_opt("b",    aero.b);
        get_opt("cbar", aero.cbar);
        get_opt("Clp",  aero.Clp);
        get_opt("Clr",  aero.Clr);
        get_opt("Cmq",  aero.Cmq);
        get_opt("Cnp",  aero.Cnp);
        get_opt("Cnr",  aero.Cnr);
    }

    void to_json(nlohmann::json& j, const RigidBody::AerodynamicParameters& aero)
    {
        j = {
            {"S",    aero.S},
            {"b",    aero.b},
            {"cbar", aero.cbar},
            {"CL",   aero.CL},
            {"CD",   aero.CD},
            {"CY",   aero.CY},
            {"Cl",   aero.Cl},
            {"Cm",   aero.Cm},
            {"Cn",   aero.Cn},
            {"Clp",  aero.Clp},
            {"Clr",  aero.Clr},
            {"Cmq",  aero.Cmq},
            {"Cnp",  aero.Cnp},
            {"Cnr",  aero.Cnr},
        };
    }

} // namespace Aetherion::Serialization
