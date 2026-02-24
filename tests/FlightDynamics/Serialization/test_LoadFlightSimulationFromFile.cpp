// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>
#include <catch2/catch_approx.hpp>

#include <string>
#include <optional>

#include "Aetherion/FlightDynamics/Serialization/LoadFlightSimulationConfigFromFile.h"

namespace {

    std::optional<std::string> g_configFile;

} // anonymous namespace

// Custom main so we can pass filename argument
int main(int argc, char* argv[])
{
    Catch::Session session;

    using namespace Catch::Clara;

    std::string filename;

    auto cli =
        session.cli()
        | Opt(filename, "filename")
        ["--config"]
        ("Path to FlightSimulationConfig JSON file");

    session.cli(cli);

    int returnCode = session.applyCommandLine(argc, argv);
    if (returnCode != 0)
        return returnCode;

    if (!filename.empty())
        g_configFile = filename;

    return session.run();
}

TEST_CASE("FlightSimulationConfig loads correctly from JSON file", "[config][file]")
{
    REQUIRE(g_configFile.has_value());

    auto cfg =
        Aetherion::FlightDynamics::Serialization::
        LoadFlightSimulationConfigFromFile(*g_configFile);

    // Example sanity checks
    REQUIRE(cfg.inertialParameters.mass_kg > 0.0);
    REQUIRE(cfg.simulation.duration > 0.0);

    REQUIRE(cfg.initialPose.alt_m >= 0.0);

    // Earth rotation sanity 
    REQUIRE(cfg.initialRotationAboutBodyAxes.roll_rad_s ==
        Catch::Approx(-7.292113023867705e-05));
}