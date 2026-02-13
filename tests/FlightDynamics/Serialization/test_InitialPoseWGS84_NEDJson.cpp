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

#include "Aetherion/FlightDynamics/InitialPoseWGS84_NED.h"
#include "Aetherion/FlightDynamics/Serialization/InitialPoseWGS84_NEDJson.h"

using namespace Aetherion::FlightDynamics;
namespace Ser = Aetherion::FlightDynamics::Serialization;

TEST_CASE("InitialPoseWGS84_NED: from_json parses expected values", "[json][initial_pose]")
{
    const char* text = R"(
    {
      "lat_deg": 41.1055,
      "lon_deg": 29.0217,
      "alt_m": 120.0,
      "azimuth_deg": 90.0,
      "zenith_deg": 5.0,
      "roll_deg": 0.0
    })";

    nlohmann::json j = nlohmann::json::parse(text);

    InitialPoseWGS84_NED pose{};
    Ser::from_json(j, pose);

    REQUIRE(pose.lat_deg == Catch::Approx(41.1055));
    REQUIRE(pose.lon_deg == Catch::Approx(29.0217));
    REQUIRE(pose.alt_m == Catch::Approx(120.0));
    REQUIRE(pose.azimuth_deg == Catch::Approx(90.0));
    REQUIRE(pose.zenith_deg == Catch::Approx(5.0));
    REQUIRE(pose.roll_deg == Catch::Approx(0.0));
}

TEST_CASE("InitialPoseWGS84_NED: to_json emits expected keys and values", "[json][initial_pose]")
{
    InitialPoseWGS84_NED pose{};
    pose.lat_deg = 41.1055;
    pose.lon_deg = 29.0217;
    pose.alt_m = 120.0;
    pose.azimuth_deg = 90.0;
    pose.zenith_deg = 5.0;
    pose.roll_deg = 0.0;

    nlohmann::json j;
    Ser::to_json(j, pose);

    REQUIRE(j.at("lat_deg").get<double>() == Catch::Approx(41.1055));
    REQUIRE(j.at("lon_deg").get<double>() == Catch::Approx(29.0217));
    REQUIRE(j.at("alt_m").get<double>() == Catch::Approx(120.0));
    REQUIRE(j.at("azimuth_deg").get<double>() == Catch::Approx(90.0));
    REQUIRE(j.at("zenith_deg").get<double>() == Catch::Approx(5.0));
    REQUIRE(j.at("roll_deg").get<double>() == Catch::Approx(0.0));
}

TEST_CASE("InitialPoseWGS84_NED: round-trip JSON preserves values", "[json][initial_pose]")
{
    InitialPoseWGS84_NED in{};
    in.lat_deg = 41.1055;
    in.lon_deg = 29.0217;
    in.alt_m = 120.0;
    in.azimuth_deg = 90.0;
    in.zenith_deg = 5.0;
    in.roll_deg = -2.0;

    nlohmann::json j;
    Ser::to_json(j, in);

    InitialPoseWGS84_NED out{};
    Ser::from_json(j, out);

    REQUIRE(out.lat_deg == Catch::Approx(in.lat_deg));
    REQUIRE(out.lon_deg == Catch::Approx(in.lon_deg));
    REQUIRE(out.alt_m == Catch::Approx(in.alt_m));
    REQUIRE(out.azimuth_deg == Catch::Approx(in.azimuth_deg));
    REQUIRE(out.zenith_deg == Catch::Approx(in.zenith_deg));
    REQUIRE(out.roll_deg == Catch::Approx(in.roll_deg));
}
