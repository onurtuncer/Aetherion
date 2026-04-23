// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Aetherion/Serialization/LoadConfigFromString.h>
#include <Aetherion/RigidBody/Config.h>

namespace {

// Minimal valid JSON that satisfies from_json(j, RigidBody::Config)
const std::string kValidJson = R"({
    "pose": {
        "lat_deg": 41.1, "lon_deg": 29.0, "alt_m": 100.0,
        "azimuth_deg": 90.0, "zenith_deg": 5.0, "roll_deg": 0.0
    },
    "velocityNED": { "north_mps": 0.0, "east_mps": 0.0, "down_mps": 0.0 },
    "bodyRates":   { "roll_rad_s": 0.0, "pitch_rad_s": 0.0, "yaw_rad_s": 0.0 },
    "inertialParameters": {
        "mass_kg": 10.0,
        "inertia_kgm2": {
            "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3,
            "Ixy": 0.0, "Iyz": 0.0, "Ixz": 0.0
        },
        "body_origin_wrt_cog_m": { "x": 0.0, "y": 0.0, "z": 0.0 }
    },
    "aerodynamicParameters": {
        "S": 0.1, "CL": 0.0, "CD": 0.0, "CY": 0.0,
        "Cl": 0.0, "Cm": 0.0, "Cn": 0.0
    }
})";

} // namespace

TEST_CASE("LoadConfigFromString: empty string throws runtime_error",
    "[serialization][LoadConfigFromString]")
{
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfigFromString(""),
        std::runtime_error);
}

TEST_CASE("LoadConfigFromString: empty string error message is descriptive",
    "[serialization][LoadConfigFromString]")
{
    using Catch::Matchers::ContainsSubstring;
    REQUIRE_THROWS_WITH(
        Aetherion::Serialization::LoadConfigFromString(""),
        ContainsSubstring("empty"));
}

TEST_CASE("LoadConfigFromString: malformed JSON throws runtime_error",
    "[serialization][LoadConfigFromString]")
{
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfigFromString("{ not valid json }"),
        std::runtime_error);
}

TEST_CASE("LoadConfigFromString: malformed JSON error message mentions syntax",
    "[serialization][LoadConfigFromString]")
{
    using Catch::Matchers::ContainsSubstring;
    REQUIRE_THROWS_WITH(
        Aetherion::Serialization::LoadConfigFromString("{bad:"),
        ContainsSubstring("syntax") || ContainsSubstring("JSON") || ContainsSubstring("parse"));
}

TEST_CASE("LoadConfigFromString: valid JSON returns populated config",
    "[serialization][LoadConfigFromString]")
{
    auto cfg = Aetherion::Serialization::LoadConfigFromString(kValidJson);

    CHECK(cfg.pose.lat_deg       == Catch::Approx(41.1));
    CHECK(cfg.pose.lon_deg       == Catch::Approx(29.0));
    CHECK(cfg.pose.alt_m         == Catch::Approx(100.0));
    CHECK(cfg.inertialParameters.mass_kg == Catch::Approx(10.0));
    CHECK(cfg.inertialParameters.Ixx     == Catch::Approx(0.1));
}

TEST_CASE("LoadConfigFromString: schema mismatch throws runtime_error",
    "[serialization][LoadConfigFromString]")
{
    // Valid JSON but missing required fields for RigidBody::Config
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfigFromString(R"({"unexpected_key": 42})"),
        std::runtime_error);
}
