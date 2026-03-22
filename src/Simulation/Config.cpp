// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Config.h"
#include <iostream>

namespace Aetherion::Simulation {

void Config::print() const {
    std::cout << "Configuration:\n"
              << "  timeStep       = " << timeStep       << "\n"
              << "  startTime      = " << startTime      << "\n"
              << "  endTime        = " << endTime        << "\n"
              << "  outputFileName = " << outputFileName  << "\n";
}

} // namespace Aetherion::Simulation
