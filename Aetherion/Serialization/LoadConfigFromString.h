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

namespace Aetherion::FlightDynamics::Serialization {
    RigidBody::Config LoadConfigFromString(const std::string& json_text);
}