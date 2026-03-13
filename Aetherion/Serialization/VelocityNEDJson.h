// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::RigidBody {
    struct VelocityNED;
} // namespace Aetherion::RigidBody

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::VelocityNED& vned);
    void to_json(nlohmann::json& j, const RigidBody::VelocityNED& vned);

} // namespace Aetherion::RigidBody