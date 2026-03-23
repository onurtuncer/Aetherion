// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <vendor/nlohmann/json.hpp>  //TODO [Onur] this should be nlohhmann/json.hpp later on
#include "Aetherion/Serialization/BodyRatesJson.h"
#include "Aetherion/RigidBody/BodyRates.h"

namespace RigidBody = Aetherion::RigidBody;
namespace Ser = Aetherion::Serialization;

using namespace Catch::Matchers;

TEST_CASE("BodyRates JSON round-trip", "[serialization][RigidBody]")
{
    SECTION("to_json produces correct keys and values")
    {
        const RigidBody::BodyRates rot{ 0.1, 0.2, 0.3 };
        nlohmann::json j;
        Ser::to_json(j, rot);

        REQUIRE(j.contains("roll_rad_s"));
        REQUIRE(j.contains("pitch_rad_s"));
        REQUIRE(j.contains("yaw_rad_s"));

        CHECK_THAT(j["roll_rad_s"].get<double>(), WithinRel(0.1));
        CHECK_THAT(j["pitch_rad_s"].get<double>(), WithinRel(0.2));
        CHECK_THAT(j["yaw_rad_s"].get<double>(), WithinRel(0.3));
    }

    SECTION("from_json deserialises all fields correctly")
    {
        const nlohmann::json j = {
            {"roll_rad_s",  0.4},
            {"pitch_rad_s", 0.5},
            {"yaw_rad_s",   0.6}
        };

        RigidBody::BodyRates rot{};
        Ser::from_json(j, rot);

        CHECK_THAT(rot.roll_rad_s, WithinRel(0.4));
        CHECK_THAT(rot.pitch_rad_s, WithinRel(0.5));
        CHECK_THAT(rot.yaw_rad_s, WithinRel(0.6));
    }

    SECTION("round-trip serialization preserves all values")
    {
        const RigidBody::BodyRates original{ 1.1, -0.5, 0.0 };

        nlohmann::json j;
        Ser::to_json(j, original);

        RigidBody::BodyRates restored{};
        Ser::from_json(j, restored);

        CHECK_THAT(restored.roll_rad_s, WithinRel(original.roll_rad_s));
        CHECK_THAT(restored.pitch_rad_s, WithinRel(original.pitch_rad_s));
        // yaw is exactly 0.0 — use WithinAbs for zero-value check
        CHECK_THAT(restored.yaw_rad_s, WithinAbs(original.yaw_rad_s, 1e-12));
    }

    SECTION("default-constructed struct serialises to zero values")
    {
        const RigidBody::BodyRates rot{};
        nlohmann::json j;
        Ser::to_json(j, rot);

        CHECK_THAT(j["roll_rad_s"].get<double>(), WithinAbs(0.0, 1e-12));
        CHECK_THAT(j["pitch_rad_s"].get<double>(), WithinAbs(0.0, 1e-12));
        CHECK_THAT(j["yaw_rad_s"].get<double>(), WithinAbs(0.0, 1e-12));
    }

    SECTION("from_json throws on missing key")
    {
        const nlohmann::json j = {
            {"roll_rad_s", 0.1}
            // pitch_rad_s and yaw_rad_s deliberately omitted
        };

        RigidBody::BodyRates rot{};
        REQUIRE_THROWS_AS(Ser::from_json(j, rot), nlohmann::json::out_of_range);
    }
}
