// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/FlightDynamics/Serialization/JsonConfigNlohmannAdapter.h"

// #include <nlohmann/json.hpp>

namespace Aetherion::FlightDynamics::Serialization {

    struct FlightDynamics::InertialParameters;

    void from_json(const nlohmann::json& j, FlightDynamics::InertialParameters& ip);
    void to_json(nlohmann::json& j, const FlightDynamics::InertialParameters& ip);

} // namespace Aetherion::FlightDynamics::Serialization

