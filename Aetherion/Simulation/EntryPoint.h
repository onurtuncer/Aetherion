// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Application.h"
#include <cstdlib>

namespace Aetherion::Simulation {

/// @brief Factory function that the client application **must** define to create its Application instance.
///
/// The implementation is provided by the user in exactly one client translation unit.
/// EntryPoint.h defines `main()` which calls this function, takes ownership of the returned
/// pointer, invokes Application::run(), and deletes it on exit. The client must never define
/// their own `main()`.
///
/// @param argc Argument count forwarded from the process entry point.
/// @param argv Argument vector forwarded from the process entry point.
/// @return Heap-allocated Application (or subclass) instance. Ownership is transferred to the entry point.
/// @throws std::invalid_argument if command-line arguments are invalid (caught by EntryPoint before run()).
Application* CreateApplication(int argc, char* argv[]);

} // namespace Aetherion::Simulation

/// @brief Program entry point defined by Aetherion — include this header in exactly ONE client translation unit.
///
/// Calls Aetherion::Simulation::CreateApplication(), runs the simulation, and handles top-level
/// exceptions. The client must never define their own `main()`.
int main(int argc, char* argv[]) {
    Aetherion::Simulation::Application* app = nullptr;
    try {
        app = Aetherion::Simulation::CreateApplication(argc, argv);
    }
    catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << "\n\n";
        // Re-create a temporary parser just to print usage, or store it —
        // simplest: just print the error and exit
        return EXIT_FAILURE;
    }

    try {
        app->run();
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error during simulation: " << e.what() << "\n";
        delete app;
        return EXIT_FAILURE;
    }

    delete app;
    return EXIT_SUCCESS;
}
