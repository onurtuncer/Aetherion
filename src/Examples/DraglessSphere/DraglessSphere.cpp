// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DraglessSphere.cpp
//
// Entry point for the NASA TM-2015-218675 Atmospheric Scenario 1 executable.
//
// Usage:
//   DraglessSphere.exe \
//       --inputFileName  nasa_atmos_01_dragless_sphere.json \
//       --outputFileName atmos_01_output.csv                \
//       --startTime      0.0                                 \
//       --endTime        30.0                                \
//       --timeStep       0.01
//
// All flags have sensible defaults (see Simulation::Application / Config.h).
//
// The EntryPoint.h header defines main() and calls CreateApplication().
// This translation unit defines CreateApplication() so that
// DraglessSphereApplication is instantiated and returned.
// ------------------------------------------------------------------------------

#include <Aetherion/Examples/DraglessSphere/DraglessSphereApplication.h>
#include <Aetherion/Simulation/EntryPoint.h>   // defines main(); must be last

// ─────────────────────────────────────────────────────────────────────────────
// Client factory — called by the EntryPoint-generated main().
// Returns a heap-allocated DraglessSphereApplication; EntryPoint owns lifetime.
// ─────────────────────────────────────────────────────────────────────────────
namespace Aetherion::Simulation {

    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::DraglessSphere::DraglessSphereApplication(argc, argv);
    }

} // namespace Aetherion::Simulation