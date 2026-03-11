// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::FlightDynamics {
    struct VelocityNED;
} // namespace Aetherion::FlightDynamics

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::VelocityNED& vned);
    void to_json(nlohmann::json& j, const FlightDynamics::VelocityNED& vned);

} // namespace Aetherion::Serialization