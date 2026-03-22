// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Application.h"
#include <cstdlib>

int main(int argc, char* argv[]) {
    Aetherion::Simulation::Application app(argc, argv);
    app.run();
    return EXIT_SUCCESS;
}
