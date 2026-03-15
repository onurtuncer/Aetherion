// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Serialization/JsonConfigNlohmannAdapter.h"

namespace Aetherion::RigidBody{

    struct GeodeticPoseNED;
} // namespace Aetherion::RigidBody

namespace Aetherion::Serialization {

    void from_json(const nlohmann::json& j, RigidBody::GeodeticPoseNED& pose);
    void to_json(nlohmann::json& j, const RigidBody::GeodeticPoseNED& pose);

} // namespace Aetherion::Serialization