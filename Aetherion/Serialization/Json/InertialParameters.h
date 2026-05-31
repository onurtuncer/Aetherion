// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/Json/ConfigNlohmannAdapter.h"

namespace Aetherion::RigidBody {
    struct InertialParameters;
}

namespace Aetherion::Serialization::Json {


    void from_json(const nlohmann::json& j, RigidBody::InertialParameters& ip);
    void to_json(nlohmann::json& j, const RigidBody::InertialParameters& ip);

} // namespace Aetherion::Serialization

