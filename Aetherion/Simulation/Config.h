// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <cstddef>
#include <string>

namespace Aetherion::Simulation {

    /// @brief Plain-data container holding all simulation-level run parameters.
    ///
    /// Populated from JSON via Aetherion::Serialization::LoadConfig() and passed to
    /// the Application before the integration loop starts. All fields have sensible
    /// defaults so a minimal JSON file need only override what differs.
    struct Config
    {
        double timeStep = 0.01;                    ///< Integration time step [s].
        double startTime = 0.0;                    ///< Epoch of the run [s]: sets the initial Earth rotation angle; the output's time column is time elapsed since it.
        double endTime = 1.0;                      ///< End of the run on the same scale [s]; must be > startTime. The run lasts endTime - startTime.
        std::size_t writeInterval = 1;             ///< Number of integration steps between consecutive output writes.
        std::string inputFileName;                 ///< Path to the JSON configuration file (may be empty when config is provided programmatically).
        std::string outputFileName = "output.txt"; ///< Path for the CSV/text output file.

        /// @brief Prints a human-readable summary of all fields to stdout.
        void print() const;
    };

} // namespace Aetherion::Simulation
