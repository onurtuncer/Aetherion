// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::RigidBody::Parameters {
    struct Aerodynamic;
}

namespace Aetherion::Serialization {


    void from_json(const nlohmann::json& j, RigidBody::Parameters::Aerodynamic& ap);
    void to_json(nlohmann::json& j, const RigidBody::Parameters::Aerodynamic& ap);

} // namespace Aetherion::Serialization