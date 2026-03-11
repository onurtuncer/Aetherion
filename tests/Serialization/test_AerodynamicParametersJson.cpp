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

#include "Aetherion/FlightDynamics/AerodynamicParameters.h"
#include "Aetherion/FlightDynamics/Serialization/AerodynamicParametersJson.h"

namespace Ser = Aetherion::FlightDynamics::Serialization;
using namespace Aetherion::FlightDynamics;

TEST_CASE("AerodynamicParameters: from_json parses expected values", "[json][aero]")
{
    const char* text = R"(
    {
      "S": 0.85,
      "CL": 0.52,
      "CD": 0.03,
      "CY": 0.01,
      "Cl": 0.002,
      "Cm": -0.04,
      "Cn": 0.005
    })";

    const nlohmann::json j = nlohmann::json::parse(text);

    AerodynamicParameters ap{};
    Ser::from_json(j, ap);

    REQUIRE(ap.S == Catch::Approx(0.85));
    REQUIRE(ap.CL == Catch::Approx(0.52));
    REQUIRE(ap.CD == Catch::Approx(0.03));
    REQUIRE(ap.CY == Catch::Approx(0.01));
    REQUIRE(ap.Cl == Catch::Approx(0.002));
    REQUIRE(ap.Cm == Catch::Approx(-0.04));
    REQUIRE(ap.Cn == Catch::Approx(0.005));
}

TEST_CASE("AerodynamicParameters: to_json emits expected keys and values", "[json][aero]")
{
    AerodynamicParameters ap{};
    ap.S = 1.2;
    ap.CL = 0.8;
    ap.CD = 0.05;
    ap.CY = 0.02;
    ap.Cl = -0.01;
    ap.Cm = 0.03;
    ap.Cn = -0.004;

    nlohmann::json j;
    Ser::to_json(j, ap);

    REQUIRE(j.at("S").get<double>() == Catch::Approx(1.2));
    REQUIRE(j.at("CL").get<double>() == Catch::Approx(0.8));
    REQUIRE(j.at("CD").get<double>() == Catch::Approx(0.05));
    REQUIRE(j.at("CY").get<double>() == Catch::Approx(0.02));
    REQUIRE(j.at("Cl").get<double>() == Catch::Approx(-0.01));
    REQUIRE(j.at("Cm").get<double>() == Catch::Approx(0.03));
    REQUIRE(j.at("Cn").get<double>() == Catch::Approx(-0.004));
}

TEST_CASE("AerodynamicParameters: JSON round-trip preserves values", "[json][aero]")
{
    AerodynamicParameters in{};
    in.S = 0.95;
    in.CL = 0.6;
    in.CD = 0.04;
    in.CY = 0.0;
    in.Cl = 0.001;
    in.Cm = -0.02;
    in.Cn = 0.003;

    nlohmann::json j;
    Ser::to_json(j, in);

    AerodynamicParameters out{};
    Ser::from_json(j, out);

    REQUIRE(out.S == Catch::Approx(in.S));
    REQUIRE(out.CL == Catch::Approx(in.CL));
    REQUIRE(out.CD == Catch::Approx(in.CD));
    REQUIRE(out.CY == Catch::Approx(in.CY));
    REQUIRE(out.Cl == Catch::Approx(in.Cl));
    REQUIRE(out.Cm == Catch::Approx(in.Cm));
    REQUIRE(out.Cn == Catch::Approx(in.Cn));
}

TEST_CASE("AerodynamicParameters: missing keys throw", "[json][aero][negative]")
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
      "S": 0.85,
      "CL": 0.52,
      "CD": 0.03
    })");

    AerodynamicParameters ap{};
    REQUIRE_THROWS_AS(Ser::from_json(j, ap), nlohmann::json::out_of_range);
}

TEST_CASE("AerodynamicParameters: wrong types throw", "[json][aero][negative]")
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
      "S": "invalid",
      "CL": 0.52,
      "CD": 0.03,
      "CY": 0.01,
      "Cl": 0.002,
      "Cm": -0.04,
      "Cn": 0.005
    })");

    AerodynamicParameters ap{};
    REQUIRE_THROWS_AS(Ser::from_json(j, ap), nlohmann::json::type_error);
}

TEST_CASE("AerodynamicParameters: basic physical sanity", "[json][aero][physics]")
{
    AerodynamicParameters ap{};
    ap.S = 1.0;
    ap.CL = 0.5;
    ap.CD = 0.03;

    // Reference area must be positive
    REQUIRE(ap.S > 0.0);

    // Drag coefficient should typically be non-negative
    REQUIRE(ap.CD >= 0.0);

    // No hard restriction on CL sign (can be negative at negative AoA)
}
