// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// Reads InitialConditions from a mock JSON *string* and verifies fields.
// This assumes you have (or will add) a helper:
//   Json parse_json_string(std::string_view s);
//
// and a loader like:
//   InitialConditions load_initial_conditions(const Json& root);
//   (or load_launch_site_init / similar).
//
// If your loader currently expects a file path, just call parse_json_string()
// and then call the same inner loader that takes a Json root.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Aetherion/FlightDynamics/JsonConfig.h"         // Json + json_* API + InitialConditions
#include "Aetherion/FlightDynamics/JsonAdapter.h"        // parse_json_string declaration 
#include "Aetherion/FlightDynamics/InitialPoseWGS84_NED.h"  


// If your project doesn't have a public declaration yet, forward-declare for the test.
namespace Aetherion::FlightDynamics {
    Json parse_json_string(std::string_view s);

    InitialPoseWGS84_NED load_initial_pose(const Json& root);
}

namespace Aetherion::FlightDynamics {

   /* TEST_CASE("InitialConditions: parse from JSON string", "[flightdynamics][json][init]") {
        // NOTE: This mock matches the InitialConditions you showed:
        //   t0, pW (ECI position), qWB (wxyz), omegaB, vB, m
        //
        // Adjust key names ONLY if your loader uses different ones.
        const char* s = R"json(
    {
      "initial_conditions": {
        "t0": 1.5,

        "pW_m":    [ 100.0, 200.0, 300.0 ],
        "qWB_wxyz":[ 1.0,   0.0,   0.0,   0.0 ],

        "omegaB_radps": [ 0.01, 0.02, 0.03 ],
        "vB_mps":       [ 10.0, 20.0, 30.0 ],

        "m_kg": 12.25
      }
    }
    )json";

        const Json root = parse_json_string(s);

        REQUIRE(json_has(root, "initial_conditions"));
        const Json ic_json = json_at(root, "initial_conditions");

        const InitialPoseWGS84_NED ic = load_initial_conditions(ic_json);

        REQUIRE(ic.t0 == Catch::Approx(1.5));

        REQUIRE(ic.pW.x() == Catch::Approx(100.0));
        REQUIRE(ic.pW.y() == Catch::Approx(200.0));
        REQUIRE(ic.pW.z() == Catch::Approx(300.0));

        // QuatWxyz: [w, x, y, z]
        REQUIRE(ic.qWB.w() == Catch::Approx(1.0));
        REQUIRE(ic.qWB.x() == Catch::Approx(0.0));
        REQUIRE(ic.qWB.y() == Catch::Approx(0.0));
        REQUIRE(ic.qWB.z() == Catch::Approx(0.0));

        REQUIRE(ic.omegaB.x() == Catch::Approx(0.01));
        REQUIRE(ic.omegaB.y() == Catch::Approx(0.02));
        REQUIRE(ic.omegaB.z() == Catch::Approx(0.03));

        REQUIRE(ic.vB.x() == Catch::Approx(10.0));
        REQUIRE(ic.vB.y() == Catch::Approx(20.0));
        REQUIRE(ic.vB.z() == Catch::Approx(30.0));

        REQUIRE(ic.m == Catch::Approx(12.25));
    }

    TEST_CASE("InitialPose: missing required key throws", "[flightdynamics][json][init]") {
        const char* s = R"json(
    {
      "initial_conditions": {
        "t0": 0.0,
        "pW_m": [0,0,0]
        // qWB_wxyz missing on purpose
      }
    }
    )json";

        const Json root = parse_json_string(s);
        const Json ic_json = json_at(root, "initial_conditions");

        // Your loader should throw ConfigError (or similar) when required keys are missing.
        REQUIRE_THROWS(load_initial_conditions(ic_json));
    } */

} // namespace Aetherion::FlightDynamics
