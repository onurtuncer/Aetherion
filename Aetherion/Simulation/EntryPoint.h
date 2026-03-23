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

// ─────────────────────────────────────────────────────────────
// Forward-declare the factory the client MUST define.
// The client returns a heap-allocated Application (or subclass).
// EntryPoint owns the lifetime and deletes it after run().
// ─────────────────────────────────────────────────────────────
namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char* argv[]);
}

// ─────────────────────────────────────────────────────────────
// main is defined here — the client never writes their own.
// Include this header in exactly ONE client translation unit.
// ─────────────────────────────────────────────────────────────
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
    app->run();
    delete app;
    return EXIT_SUCCESS;
}