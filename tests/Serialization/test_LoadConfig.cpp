// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>
#include <catch2/catch_approx.hpp>

#include <string>
#include <optional>
#include <fstream>
#include <cstdio>

#include "Aetherion/Serialization/LoadConfig.h"
#include "Aetherion/RigidBody/Config.h"
#include <vendor/nlohmann/json.hpp>

namespace {

    std::optional<std::string> g_configFile;

    // Minimal valid JSON satisfying from_json(j, RigidBody::Config)
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

// ─────────────────────────────────────────────────────────────────────────────
// LoadConfig(filename) error paths
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("LoadConfig(filename): throws for non-existent file", "[config][LoadConfig]")
{
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfig(std::string{"_nonexistent_file_xyz.json"}),
        std::runtime_error);
}

TEST_CASE("LoadConfig(filename): throws for malformed JSON", "[config][LoadConfig]")
{
    const std::string tmp = "_ae_test_bad.json";
    { std::ofstream f(tmp); f << "{ not : valid json }"; }
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfig(tmp),
        std::runtime_error);
    std::remove(tmp.c_str());
}

TEST_CASE("LoadConfig(filename): throws for schema mismatch", "[config][LoadConfig]")
{
    const std::string tmp = "_ae_test_schema.json";
    { std::ofstream f(tmp); f << R"({"unexpected_key": 42})"; }
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfig(tmp),
        std::runtime_error);
    std::remove(tmp.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// LoadConfig(j, source) overload
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("LoadConfig(j, source): happy path returns populated config", "[config][LoadConfig]")
{
    auto j = nlohmann::json::parse(kValidJson);
    auto cfg = Aetherion::Serialization::LoadConfig(j, "test_source");
    CHECK(cfg.inertialParameters.mass_kg == Catch::Approx(10.0));
    CHECK(cfg.pose.lat_deg == Catch::Approx(41.1));
}

TEST_CASE("LoadConfig(j, source): schema mismatch throws runtime_error", "[config][LoadConfig]")
{
    auto bad_j = nlohmann::json::parse(R"({"wrong_key": 42})");
    REQUIRE_THROWS_AS(
        Aetherion::Serialization::LoadConfig(bad_j, "test_bad_source"),
        std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// External-file test
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("SimulationConfig loads correctly from JSON file", "[config][external_file]")
{
    REQUIRE(g_configFile.has_value());

    auto cfg = Aetherion::Serialization::LoadConfig(*g_configFile);

    // Example sanity checks
    REQUIRE(cfg.inertialParameters.mass_kg > 0.0);

    REQUIRE(cfg.pose.alt_m >= 0.0);

    // Earth rotation sanity
    REQUIRE(cfg.bodyRates.roll_rad_s ==
        Catch::Approx(-7.292113023867705e-05));
}
