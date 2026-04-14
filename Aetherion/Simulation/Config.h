// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <string>

namespace Aetherion::Simulation {

    // ─────────────────────────────────────────────────────────────
    // Config  –  plain data container for all program settings
    // ─────────────────────────────────────────────────────────────
    class Config {
    public:
        double      timeStep = 0.01;
        double      startTime = 0.0;
        double      endTime = 1.0;
        std::size_t writeInterval = 1;
        std::string inputFileName = "";
        std::string outputFileName = "output.txt";

        void print() const;
    };

} // namespace Aetherion::Simulation

