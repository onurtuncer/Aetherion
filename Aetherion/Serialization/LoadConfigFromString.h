// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <string>

namespace Aetherion::RigidBody {
    struct Config;
}

namespace Aetherion::Serialization {
    RigidBody::Config LoadConfigFromString(const std::string& jsonString);
} // namespace Aetherion::Serialization
