// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//
// JsonConfig.h
// Config loader for Aetherion flight dynamics.
//
// This header is JSON-library agnostic; implement the Json adapter functions
// using your chosen JSON library (nlohmann/json, rapidjson, etc).
//

#pragma once

#include <stdexcept>
#include <string>

#include "Aetherion/Serialization/Json/Adapter.h"

namespace Aetherion::FlightDynamics::Serialization::Json {

    class ConfigError final : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

} // namespace Aetherion::FlightDynamics::Serialization::Json
