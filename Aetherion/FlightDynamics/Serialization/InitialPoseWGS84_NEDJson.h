// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/FlightDynamics/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::FlightDynamics::Serialization {

    struct FlightDynamics::InitialPoseWGS84_NED;

    void from_json(const nlohmann::json& j, FlightDynamics::InitialPoseWGS84_NED& pose);
    void to_json(nlohmann::json& j, const FlightDynamics::InitialPoseWGS84_NED& pose);

} // namespace Aetherion::FlightDynamics::Serialization