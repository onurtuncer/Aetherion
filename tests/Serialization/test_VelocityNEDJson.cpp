// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vendor/nlohmann/json.hpp>

#include "Aetherion/RigidBody/VelocityNED.h"
#include "Aetherion/Serialization/VelocityNEDJson.h"

using namespace Aetherion::FlightDynamics;
namespace Ser = Aetherion::Serialization;

using namespace Catch::Matchers;

namespace Aetherion::Test::Serialization {
    
TEST_CASE("VelocityNED JSON round-trip", "[serialization][FlightDynamics]")
{
    SECTION("to_json produces correct keys and values")
    {
        const VelocityNED vel{ 10.0, -5.0, 2.5 };
        nlohmann::json j;
        Ser::to_json(j, vel);

        REQUIRE(j.contains("north_mps"));
        REQUIRE(j.contains("east_mps"));
        REQUIRE(j.contains("down_mps"));

        CHECK_THAT(j["north_mps"].get<double>(), WithinRel(10.0));
        CHECK_THAT(j["east_mps"].get<double>(), WithinRel(-5.0));
        CHECK_THAT(j["down_mps"].get<double>(), WithinRel(2.5));
    }

    SECTION("from_json deserialises all fields correctly")
    {
        const nlohmann::json j = {
            {"north_mps",  3.0},
            {"east_mps",  -1.5},
            {"down_mps",   0.8}
        };

        VelocityNED vel{};
        Ser::from_json(j, vel);

        CHECK_THAT(vel.north_mps, WithinRel(3.0));
        CHECK_THAT(vel.east_mps, WithinRel(-1.5));
        CHECK_THAT(vel.down_mps, WithinRel(0.8));
    }

    SECTION("round-trip serialization preserves all values")
    {
        const VelocityNED original{ 100.0, 0.0, -9.81 };

        nlohmann::json j;
        Ser::to_json(j, original);

        VelocityNED restored{};
        Ser::from_json(j, restored);

        CHECK_THAT(restored.north_mps, WithinRel(original.north_mps));
        // east is exactly 0.0 — use WithinAbs for zero-value check
        CHECK_THAT(restored.east_mps, WithinAbs(original.east_mps, 1e-12));
        CHECK_THAT(restored.down_mps, WithinRel(original.down_mps));
    }

    SECTION("default-constructed struct serialises to zero values")
    {
        const VelocityNED vel{};
        nlohmann::json j;
        Ser::to_json(j, vel);

        CHECK_THAT(j["north_mps"].get<double>(), WithinAbs(0.0, 1e-12));
        CHECK_THAT(j["east_mps"].get<double>(), WithinAbs(0.0, 1e-12));
        CHECK_THAT(j["down_mps"].get<double>(), WithinAbs(0.0, 1e-12));
    }

    SECTION("from_json throws on missing key")
    {
        const nlohmann::json j = {
            {"north_mps", 1.0}
            // east_mps and down_mps deliberately omitted
        };

        VelocityNED vel{};
        REQUIRE_THROWS_AS(Ser::from_json(j, vel), nlohmann::json::out_of_range);
    }

    SECTION("physically meaningful values: hover with slight drift")
    {
        // Hovering vehicle: near-zero horizontal, small positive down (sinking)
        const VelocityNED original{ 0.05, -0.03, 0.1 };

        nlohmann::json j;
        Ser::to_json(j, original);

        VelocityNED restored{};
        Ser::from_json(j, restored);

        CHECK_THAT(restored.north_mps, WithinRel(original.north_mps));
        CHECK_THAT(restored.east_mps, WithinRel(original.east_mps));
        CHECK_THAT(restored.down_mps, WithinRel(original.down_mps));
    }
}

} // namespace Aetherion::Test::Serialization