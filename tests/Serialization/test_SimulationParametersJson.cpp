// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vendor/nlohmann/json.hpp>

#include "Aetherion/FlightDynamics/SimulationParameters.h"
#include "Aetherion/FlightDynamics/Serialization/SimulationParametersJson.h"

namespace Ser = Aetherion::FlightDynamics::Serialization;
using namespace Aetherion::FlightDynamics;

TEST_CASE("SimulationParameters: from_json parses expected values", "[json][simulation]")
{
    const char* text = R"(
    {
      "startTime": 1.25,
      "duration":  10.0
    })";

    const nlohmann::json j = nlohmann::json::parse(text);

    SimulationParameters sp{};
    Ser::from_json(j, sp);

    REQUIRE(sp.startTime == Catch::Approx(1.25));
    REQUIRE(sp.duration == Catch::Approx(10.0));
}

TEST_CASE("SimulationParameters: to_json emits expected keys and values", "[json][simulation]")
{
    SimulationParameters sp{};
    sp.startTime = 2.5;
    sp.duration = 60.0;

    nlohmann::json j;
    Ser::to_json(j, sp);

    REQUIRE(j.at("startTime").get<double>() == Catch::Approx(2.5));
    REQUIRE(j.at("duration").get<double>() == Catch::Approx(60.0));
}

TEST_CASE("SimulationParameters: JSON round-trip preserves values", "[json][simulation]")
{
    SimulationParameters in{};
    in.startTime = 0.5;
    in.duration = 123.456;

    nlohmann::json j;
    Ser::to_json(j, in);

    SimulationParameters out{};
    Ser::from_json(j, out);

    REQUIRE(out.startTime == Catch::Approx(in.startTime));
    REQUIRE(out.duration == Catch::Approx(in.duration));
}

TEST_CASE("SimulationParameters: missing keys throw", "[json][simulation][negative]")
{
    SECTION("Missing startTime")
    {
        const nlohmann::json j = nlohmann::json::parse(R"({"duration": 10.0})");
        SimulationParameters sp{};
        REQUIRE_THROWS_AS(Ser::from_json(j, sp), nlohmann::json::out_of_range);
    }

    SECTION("Missing duration")
    {
        const nlohmann::json j = nlohmann::json::parse(R"({"startTime": 0.0})");
        SimulationParameters sp{};
        REQUIRE_THROWS_AS(Ser::from_json(j, sp), nlohmann::json::out_of_range);
    }
}

TEST_CASE("SimulationParameters: wrong types throw", "[json][simulation][negative]")
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
      "startTime": "not-a-number",
      "duration":  10.0
    })");

    SimulationParameters sp{};
    REQUIRE_THROWS_AS(Ser::from_json(j, sp), nlohmann::json::type_error);
}