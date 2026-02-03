// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/FlightDynamics/JsonConfigNlohmannAdapter.h"

// #include <nlohmann/json.hpp>

namespace Aetherion::FlightDynamics::Serialization {

    struct InertialParameters;

    void from_json(const nlohmann::json& j, InertialParameters& ip);
    void to_json(nlohmann::json& j, const InertialParameters& ip);

} // namespace Aetherion::FlightDynamics::Serialization

