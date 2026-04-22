// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Simulation::Config
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Simulation/Config.h>

#include <iostream>
#include <sstream>
#include <string>

using namespace Aetherion::Simulation;

// =============================================================================
// Default values
// =============================================================================

TEST_CASE("Simulation::Config has correct default values", "[Config][Simulation]") {
    const Config cfg{};

    SECTION("timeStep defaults to 0.01 s") {
        REQUIRE(cfg.timeStep == Catch::Approx(0.01));
    }
    SECTION("startTime defaults to 0.0 s") {
        REQUIRE(cfg.startTime == Catch::Approx(0.0));
    }
    SECTION("endTime defaults to 1.0 s") {
        REQUIRE(cfg.endTime == Catch::Approx(1.0));
    }
    SECTION("writeInterval defaults to 1") {
        REQUIRE(cfg.writeInterval == std::size_t{1});
    }
    SECTION("inputFileName defaults to empty string") {
        REQUIRE(cfg.inputFileName.empty());
    }
    SECTION("outputFileName defaults to output.txt") {
        REQUIRE(cfg.outputFileName == "output.txt");
    }
}

// =============================================================================
// Field assignment
// =============================================================================

TEST_CASE("Simulation::Config fields can be independently assigned", "[Config][Simulation]") {
    Config cfg;
    cfg.timeStep      = 0.005;
    cfg.startTime     = 2.0;
    cfg.endTime       = 10.0;
    cfg.writeInterval = 5;
    cfg.inputFileName  = "input.json";
    cfg.outputFileName = "results.csv";

    REQUIRE(cfg.timeStep      == Catch::Approx(0.005));
    REQUIRE(cfg.startTime     == Catch::Approx(2.0));
    REQUIRE(cfg.endTime       == Catch::Approx(10.0));
    REQUIRE(cfg.writeInterval == std::size_t{5});
    REQUIRE(cfg.inputFileName  == "input.json");
    REQUIRE(cfg.outputFileName == "results.csv");
}

TEST_CASE("Simulation::Config writeInterval stores large values", "[Config][Simulation]") {
    Config cfg;
    cfg.writeInterval = 1'000'000;
    REQUIRE(cfg.writeInterval == std::size_t{1'000'000});
}

// =============================================================================
// Copy and value semantics
// =============================================================================

TEST_CASE("Simulation::Config is copyable and copies are independent", "[Config][Simulation]") {
    Config a;
    a.timeStep = 0.1;
    a.endTime  = 50.0;
    a.writeInterval = 10;
    a.outputFileName = "a.csv";

    Config b = a;

    // Mutate b; a must be unchanged
    b.timeStep  = 0.2;
    b.endTime   = 100.0;
    b.writeInterval = 20;
    b.outputFileName = "b.csv";

    REQUIRE(a.timeStep      == Catch::Approx(0.1));
    REQUIRE(a.endTime       == Catch::Approx(50.0));
    REQUIRE(a.writeInterval == std::size_t{10});
    REQUIRE(a.outputFileName == "a.csv");

    REQUIRE(b.timeStep      == Catch::Approx(0.2));
    REQUIRE(b.endTime       == Catch::Approx(100.0));
    REQUIRE(b.writeInterval == std::size_t{20});
    REQUIRE(b.outputFileName == "b.csv");
}

// =============================================================================
// print() — coverage of the .cpp implementation
// =============================================================================

TEST_CASE("Simulation::Config::print does not throw", "[Config][Simulation]") {
    Config cfg;
    cfg.timeStep      = 0.001;
    cfg.startTime     = 0.0;
    cfg.endTime       = 5.0;
    cfg.writeInterval = 2;
    cfg.inputFileName  = "in.json";
    cfg.outputFileName = "out.csv";

    // Redirect stdout to a stringstream so the test is silent
    std::ostringstream oss;
    std::streambuf* orig = std::cout.rdbuf(oss.rdbuf());
    REQUIRE_NOTHROW(cfg.print());
    std::cout.rdbuf(orig);
}

TEST_CASE("Simulation::Config::print output contains all field values", "[Config][Simulation]") {
    Config cfg;
    cfg.timeStep      = 0.001;
    cfg.startTime     = 3.0;
    cfg.endTime       = 7.5;
    cfg.writeInterval = 4;
    cfg.inputFileName  = "scenario.json";
    cfg.outputFileName = "trajectory.csv";

    std::ostringstream oss;
    std::streambuf* orig = std::cout.rdbuf(oss.rdbuf());
    cfg.print();
    std::cout.rdbuf(orig);

    const std::string out = oss.str();
    REQUIRE(out.find("0.001")           != std::string::npos);
    REQUIRE(out.find("3")               != std::string::npos);
    REQUIRE(out.find("7.5")             != std::string::npos);
    REQUIRE(out.find("4")               != std::string::npos);
    REQUIRE(out.find("scenario.json")   != std::string::npos);
    REQUIRE(out.find("trajectory.csv")  != std::string::npos);
}

TEST_CASE("Simulation::Config::print on default config contains default values", "[Config][Simulation]") {
    const Config cfg{};

    std::ostringstream oss;
    std::streambuf* orig = std::cout.rdbuf(oss.rdbuf());
    cfg.print();
    std::cout.rdbuf(orig);

    const std::string out = oss.str();
    REQUIRE(out.find("0.01")       != std::string::npos);  // timeStep
    REQUIRE(out.find("1")          != std::string::npos);  // endTime or writeInterval
    REQUIRE(out.find("output.txt") != std::string::npos);  // outputFileName
}