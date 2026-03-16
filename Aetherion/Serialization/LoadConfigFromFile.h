// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once
#include <string>
#include "Aetherion/RigidBody/Config.h"

namespace Aetherion::Serialization {
    RigidBody::Config LoadConfigFromFile(const std::string& filename);
}