// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Snapshot1 CSV writer
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Simulation/Application.h>

using namespace Aetherion::Simulation;

// ─────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────
struct FakeArgv {
    std::vector<const char*> data;

    explicit FakeArgv(std::initializer_list<const char*> args) : data(args) {}

    int    argc() { return static_cast<int>(data.size()); }
    char** argv() { return const_cast<char**>(data.data()); }
};

// ─────────────────────────────────────────────────────────────
// Application – default values
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application uses default config when no args are passed", "[Application]") {
    FakeArgv args{ "test_program" };
    Application app(args.argc(), args.argv());
    const Config& cfg = app.getConfig();

    REQUIRE(cfg.timeStep == Catch::Approx(0.01));
    REQUIRE(cfg.startTime == Catch::Approx(0.0));
    REQUIRE(cfg.endTime == Catch::Approx(1.0));
    REQUIRE(cfg.outputFileName == "output.txt");
}

// ─────────────────────────────────────────────────────────────
// Application – timeStep
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --timeStep correctly", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "0.005" };
    Application app(args.argc(), args.argv());

    REQUIRE(app.getConfig().timeStep == Catch::Approx(0.005));
}

TEST_CASE("Application rejects non-positive --timeStep", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "-1.0" };
    REQUIRE_THROWS_AS(Application(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects non-numeric --timeStep", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "abc" };
    REQUIRE_THROWS_AS(Application(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Application – startTime
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --startTime correctly", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "2.5" };
    Application app(args.argc(), args.argv());

    REQUIRE(app.getConfig().startTime == Catch::Approx(2.5));
}

TEST_CASE("Application rejects non-numeric --startTime", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "xyz" };
    REQUIRE_THROWS_AS(Application(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Application – endTime
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --endTime correctly", "[Application]") {
    FakeArgv args{ "test_program", "--endTime", "5.0" };
    Application app(args.argc(), args.argv());

    REQUIRE(app.getConfig().endTime == Catch::Approx(5.0));
}

TEST_CASE("Application rejects --endTime <= --startTime", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "3.0", "--endTime", "1.0" };
    REQUIRE_THROWS_AS(Application(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects --endTime equal to --startTime", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "2.0", "--endTime", "2.0" };
    REQUIRE_THROWS_AS(Application(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects non-numeric --endTime", "[Application]") {
    FakeArgv args{ "test_program", "--endTime", "xyz" };
    REQUIRE_THROWS_AS(Application(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Application – inputFileName
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --inputFileName correctly", "[Application]") {
    FakeArgv args{ "test_program", "--inputFileName", "data.dat" };
    Application app(args.argc(), args.argv());

    REQUIRE(app.getConfig().inputFileName == "data.dat");
}

TEST_CASE("Application default inputFileName is empty", "[Application]") {
    FakeArgv args{ "test_program" };
    Application app(args.argc(), args.argv());

    REQUIRE(app.getConfig().inputFileName.empty());
}

// ─────────────────────────────────────────────────────────────
// Application – outputFileName
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --outputFileName correctly", "[Application]") {
    FakeArgv args{ "test_program", "--outputFileName", "sim.csv" };
    Application app(args.argc(), args.argv());

    REQUIRE(app.getConfig().outputFileName == "sim.csv");
}

// ─────────────────────────────────────────────────────────────
// Application – combined flags
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses all flags together correctly", "[Application]") {
    FakeArgv args{
        "test_program",
        "--timeStep",       "0.001",
        "--startTime",      "1.0",
        "--endTime",        "9.0",
        "--inputFileName",  "data.dat",
        "--outputFileName", "results.csv"
    };
    Application app(args.argc(), args.argv());
    const Config& cfg = app.getConfig();

    REQUIRE(cfg.timeStep == Catch::Approx(0.001));
    REQUIRE(cfg.startTime == Catch::Approx(1.0));
    REQUIRE(cfg.endTime == Catch::Approx(9.0));
    REQUIRE(cfg.inputFileName == "data.dat");
    REQUIRE(cfg.outputFileName == "results.csv");
}