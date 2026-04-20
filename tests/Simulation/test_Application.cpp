// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Application / Config / ArgumentParser
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <iostream>
#include <sstream>

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/ArgumentParser.h>

using namespace Aetherion::Simulation;

// ─────────────────────────────────────────────────────────────
// StubApplication — minimal concrete subclass for unit tests.
// Application is abstract; all pure-virtual hooks are no-ops.
// ─────────────────────────────────────────────────────────────
class StubApplication : public Application
{
public:
    using Application::Application;
protected:
    void prepareSimulation()                              const override {}
    void writeInitialSnapshot(std::ofstream&)             const override {}
    StepObservation stepAndRecord(std::ofstream&, double, bool) const override { return {}; }
    void logFinalSummary()                                const override {}
};

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
// Tests — default values
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application uses default config when no args are passed", "[Application]") {
    FakeArgv args{ "test_program" };
    StubApplication app(args.argc(), args.argv());
    const Config& cfg = app.getConfig();

    REQUIRE(cfg.timeStep == Catch::Approx(0.01));
    REQUIRE(cfg.startTime == Catch::Approx(0.0));
    REQUIRE(cfg.endTime == Catch::Approx(1.0));
    REQUIRE(cfg.outputFileName == "output.txt");
}

// ─────────────────────────────────────────────────────────────
// Tests — combined flags
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
    StubApplication app(args.argc(), args.argv());
    const Config& cfg = app.getConfig();

    REQUIRE(cfg.timeStep == Catch::Approx(0.001));
    REQUIRE(cfg.startTime == Catch::Approx(1.0));
    REQUIRE(cfg.endTime == Catch::Approx(9.0));
    REQUIRE(cfg.inputFileName == "data.dat");
    REQUIRE(cfg.outputFileName == "results.csv");
}

// ─────────────────────────────────────────────────────────────
// Tests — timeStep
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --timeStep correctly", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "0.05" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().timeStep == Catch::Approx(0.05));
}

TEST_CASE("Application rejects non-positive --timeStep", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "-1.0" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects zero --timeStep", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "0.0" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects non-numeric --timeStep", "[Application]") {
    FakeArgv args{ "test_program", "--timeStep", "abc" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Tests — startTime / endTime
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --startTime correctly", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "5.0", "--endTime", "10.0" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().startTime == Catch::Approx(5.0));
}

TEST_CASE("Application parses --endTime correctly", "[Application]") {
    FakeArgv args{ "test_program", "--endTime", "42.0" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().endTime == Catch::Approx(42.0));
}

TEST_CASE("Application rejects --endTime <= --startTime", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "3.0", "--endTime", "1.0" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects --endTime equal to --startTime", "[Application]") {
    FakeArgv args{ "test_program", "--startTime", "2.0", "--endTime", "2.0" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects non-numeric --endTime", "[Application]") {
    FakeArgv args{ "test_program", "--endTime", "xyz" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Tests — inputFileName / outputFileName
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application parses --inputFileName correctly", "[Application]") {
    FakeArgv args{ "test_program", "--inputFileName", "data.dat" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().inputFileName == "data.dat");
}

TEST_CASE("Application default inputFileName is empty", "[Application]") {
    FakeArgv args{ "test_program" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().inputFileName.empty());
}

TEST_CASE("Application parses --outputFileName correctly", "[Application]") {
    FakeArgv args{ "test_program", "--outputFileName", "sim.csv" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().outputFileName == "sim.csv");
}

// ─────────────────────────────────────────────────────────────
// Tests — writeInterval
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application default writeInterval is 1", "[Application]") {
    FakeArgv args{ "test_program" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().writeInterval == 1u);
}

TEST_CASE("Application parses --writeInterval correctly", "[Application]") {
    FakeArgv args{ "test_program", "--writeInterval", "10" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().writeInterval == 10u);
}

TEST_CASE("Application rejects --writeInterval of zero", "[Application]") {
    FakeArgv args{ "test_program", "--writeInterval", "0" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects negative --writeInterval", "[Application]") {
    FakeArgv args{ "test_program", "--writeInterval", "-5" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Tests — ArgumentParser error paths
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application rejects unknown flag", "[Application][ArgumentParser]") {
    FakeArgv args{ "test_program", "--unknownFlag", "value" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

TEST_CASE("Application rejects flag with no value", "[Application][ArgumentParser]") {
    // --timeStep at the end of argv with no following token
    FakeArgv args{ "test_program", "--timeStep" };
    REQUIRE_THROWS_AS(StubApplication(args.argc(), args.argv()), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────
// Tests — ArgumentParser::printUsage
// ─────────────────────────────────────────────────────────────
TEST_CASE("ArgumentParser::printUsage writes Usage header to cerr", "[ArgumentParser][printUsage]") {
    Aetherion::Simulation::ArgumentParser parser("my_program");
    parser.addArgument("--foo", "A foo argument", [](const std::string&) {});

    std::ostringstream buf;
    auto* old = std::cerr.rdbuf(buf.rdbuf());
    parser.printUsage();
    std::cerr.rdbuf(old);

    const std::string out = buf.str();
    CHECK(out.find("Usage:") != std::string::npos);
    CHECK(out.find("my_program") != std::string::npos);
    CHECK(out.find("--foo") != std::string::npos);
    CHECK(out.find("--help") != std::string::npos);
}