// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Config.h"
#include "ArgumentParser.h"

namespace Aetherion::Simulation {

// ─────────────────────────────────────────────────────────────
// Application  –  wires the parser to the config and runs
// ─────────────────────────────────────────────────────────────
class Application {
public:
    explicit Application(int argc, char* argv[]);

    void run() const;

private:
    Config         config_;
    ArgumentParser parser_;

    void registerArguments();
};

} // namespace Aetherion::Simulation
