// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

//TODO [Onur] remove this struct as these parameters need to be setup by the simulation tool
// i.e. simulink or other consumer

namespace Aetherion::FlightDynamics {

    struct SimulationParameters
    {
        double startTime{ 0.0 }; // seconds
        double duration{ 0.0 };  // seconds
    };

} // namespace Aetherion::FlightDynamics