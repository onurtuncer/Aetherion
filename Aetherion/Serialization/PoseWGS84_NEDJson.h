// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::FlightDynamics{

    struct PoseWGS84_NED;
} // namespace Aetherion::FlightDynamics

namespace Aetherion::FlightDynamics::Serialization {

    struct FlightDynamics::PoseWGS84_NED;

    void from_json(const nlohmann::json& j, FlightDynamics::PoseWGS84_NED& pose);
    void to_json(nlohmann::json& j, const FlightDynamics::PoseWGS84_NED& pose);

} // namespace Aetherion::FlightDynamics::Serialization