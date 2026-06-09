// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TwoStageRocketSimulator.cpp
//
// Explicit instantiation of the default IntegratorPolicy specialisation.
// All method bodies are defined in the header (required for a class template).
// ------------------------------------------------------------------------------

#include <Aetherion/Examples/TwoStageRocket/TwoStageRocketSimulator.h>

namespace Aetherion::Examples::TwoStageRocket {

template class TwoStageRocketSimulator<>;

} // namespace Aetherion::Examples::TwoStageRocket
