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

#include "Aetherion/FlightDynamics/InertialParameters.h"
#include "Aetherion/FlightDynamics/Serialization/InertialParametersJson.h"

namespace Ser = Aetherion::FlightDynamics::Serialization;
using namespace Aetherion::FlightDynamics;

TEST_CASE("InertialParameters: from_json parses expected values", "[json][inertial]")
{
    const char* text = R"(
    {
      "mass_kg": 10.0,
      "inertia_kgm2": {
        "Ixx": 0.2,
        "Iyy": 0.3,
        "Izz": 0.4,
        "Ixy": 0.01,
        "Iyz": 0.02,
        "Ixz": 0.03
      },
      "body_origin_wrt_cog_m": {
        "x": 0.01,
        "y": -0.02,
        "z": 0.00
      }
    })";

    const nlohmann::json j = nlohmann::json::parse(text);

    InertialParameters ip{};
    Ser::from_json(j, ip);

    REQUIRE(ip.mass_kg == Catch::Approx(10.0));
    REQUIRE(ip.Ixx == Catch::Approx(0.2));
    REQUIRE(ip.Iyy == Catch::Approx(0.3));
    REQUIRE(ip.Izz == Catch::Approx(0.4));
    REQUIRE(ip.Ixy == Catch::Approx(0.01));
    REQUIRE(ip.Iyz == Catch::Approx(0.02));
    REQUIRE(ip.Ixz == Catch::Approx(0.03));
    REQUIRE(ip.xbar_m == Catch::Approx(0.01));
    REQUIRE(ip.ybar_m == Catch::Approx(-0.02));
    REQUIRE(ip.zbar_m == Catch::Approx(0.0));
}

TEST_CASE("InertialParameters: to_json emits expected nested structure", "[json][inertial]")
{
    InertialParameters ip{};
    ip.mass_kg = 12.5;

    ip.Ixx = 0.42;
    ip.Iyy = 0.58;
    ip.Izz = 0.61;
    ip.Ixy = 0.01;
    ip.Iyz = -0.02;
    ip.Ixz = 0.0;

    ip.xbar_m = 0.03;
    ip.ybar_m = -0.01;
    ip.zbar_m = 0.0;

    nlohmann::json j;
    Ser::to_json(j, ip);

    REQUIRE(j.at("mass_kg").get<double>() == Catch::Approx(12.5));

    const auto& I = j.at("inertia_kgm2");
    REQUIRE(I.at("Ixx").get<double>() == Catch::Approx(0.42));
    REQUIRE(I.at("Iyy").get<double>() == Catch::Approx(0.58));
    REQUIRE(I.at("Izz").get<double>() == Catch::Approx(0.61));
    REQUIRE(I.at("Ixy").get<double>() == Catch::Approx(0.01));
    REQUIRE(I.at("Iyz").get<double>() == Catch::Approx(-0.02));
    REQUIRE(I.at("Ixz").get<double>() == Catch::Approx(0.0));

    const auto& r = j.at("body_origin_wrt_cog_m");
    REQUIRE(r.at("x").get<double>() == Catch::Approx(0.03));
    REQUIRE(r.at("y").get<double>() == Catch::Approx(-0.01));
    REQUIRE(r.at("z").get<double>() == Catch::Approx(0.0));
}

TEST_CASE("InertialParameters: JSON round-trip preserves values", "[json][inertial]")
{
    InertialParameters in{};
    in.mass_kg = 7.25;

    in.Ixx = 0.31;
    in.Iyy = 0.27;
    in.Izz = 0.44;
    in.Ixy = -0.004;
    in.Iyz = 0.002;
    in.Ixz = 0.0005;

    in.xbar_m = 0.012;
    in.ybar_m = 0.0;
    in.zbar_m = -0.008;

    nlohmann::json j;
    Ser::to_json(j, in);

    InertialParameters out{};
    Ser::from_json(j, out);

    REQUIRE(out.mass_kg == Catch::Approx(in.mass_kg));
    REQUIRE(out.Ixx == Catch::Approx(in.Ixx));
    REQUIRE(out.Iyy == Catch::Approx(in.Iyy));
    REQUIRE(out.Izz == Catch::Approx(in.Izz));
    REQUIRE(out.Ixy == Catch::Approx(in.Ixy));
    REQUIRE(out.Iyz == Catch::Approx(in.Iyz));
    REQUIRE(out.Ixz == Catch::Approx(in.Ixz));
    REQUIRE(out.xbar_m == Catch::Approx(in.xbar_m));
    REQUIRE(out.ybar_m == Catch::Approx(in.ybar_m));
    REQUIRE(out.zbar_m == Catch::Approx(in.zbar_m));
}

TEST_CASE("InertialParameters: basic physical sanity (necessary conditions)", "[json][inertial][physics]")
{
    InertialParameters ip{};
    ip.mass_kg = 5.0;
    ip.Ixx = 0.30;
    ip.Iyy = 0.40;
    ip.Izz = 0.50;

    // Products of inertia can be non-zero; keep them small here
    ip.Ixy = 0.0;
    ip.Iyz = 0.0;
    ip.Ixz = 0.0;

    REQUIRE(ip.mass_kg > 0.0);

    // Principal moments should be positive for a physical rigid body
    REQUIRE(ip.Ixx > 0.0);
    REQUIRE(ip.Iyy > 0.0);
    REQUIRE(ip.Izz > 0.0);

    // Triangle inequalities (necessary, not sufficient)
    REQUIRE(ip.Ixx + ip.Iyy >= ip.Izz);
    REQUIRE(ip.Iyy + ip.Izz >= ip.Ixx);
    REQUIRE(ip.Izz + ip.Ixx >= ip.Iyy);
}
