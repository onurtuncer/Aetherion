// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/FlightDynamics/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::FlightDynamics {
    struct InitialRotationAboutBodyAxes;
} // namespace Aetherion::FlightDynamics

namespace Aetherion::FlightDynamics::Serialization {

    void from_json(const nlohmann::json& j, FlightDynamics::InitialRotationAboutBodyAxes& ir);
    void to_json(nlohmann::json& j, const FlightDynamics::InitialRotationAboutBodyAxes& ir);

} // namespace Aetherion::FlightDynamics::Serialization