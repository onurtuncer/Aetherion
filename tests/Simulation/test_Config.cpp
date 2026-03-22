// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Snapshot1 CSV writer
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Simulation/Config.h>

using namespace Aetherion::Simulation;

// ─────────────────────────────────────────────────────────────
// Config tests
// ─────────────────────────────────────────────────────────────
TEST_CASE("Config has correct default values", "[Config]") {
    Config cfg;

    SECTION("default timeStep is 0.01") {
        REQUIRE(cfg.timeStep == Catch::Approx(0.01));
    }

    SECTION("default startTime is 0.0") {
        REQUIRE(cfg.startTime == Catch::Approx(0.0));
    }

    SECTION("default endTime is 1.0") {
        REQUIRE(cfg.endTime == Catch::Approx(1.0));
    }

    SECTION("default inputFileName is empty") {
        REQUIRE(cfg.inputFileName.empty());
    }

    SECTION("default outputFileName is output.txt") {
        REQUIRE(cfg.outputFileName == "output.txt");
    }
}

TEST_CASE("Config fields can be assigned", "[Config]") {
    Config cfg;
    cfg.timeStep = 0.005;
    cfg.startTime = 2.0;
    cfg.endTime = 10.0;
    cfg.inputFileName = "input.dat";
    cfg.outputFileName = "results.csv";

    REQUIRE(cfg.timeStep == Catch::Approx(0.005));
    REQUIRE(cfg.startTime == Catch::Approx(2.0));
    REQUIRE(cfg.endTime == Catch::Approx(10.0));
    REQUIRE(cfg.inputFileName == "input.dat");
    REQUIRE(cfg.outputFileName == "results.csv");
}