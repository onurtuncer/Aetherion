// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Application / Config / ArgumentParser
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstdio>
#include <iostream>
#include <sstream>

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/ArgumentParser.h>
#include <Aetherion/Simulation/Log.h>

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
    std::vector<std::string>  strings;
    std::vector<char*>        ptrs;
    explicit FakeArgv(std::initializer_list<const char*> args)
    {
        for (const char* s : args) {
            strings.emplace_back(s);
        }
        for (auto& s : strings) {
            ptrs.push_back(s.data());
        }
    }
    int    argc() const { return static_cast<int>(ptrs.size()); }
    char** argv()       { return ptrs.data(); }
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
    REQUIRE(app.getConfig().writeInterval == 1U);
}

TEST_CASE("Application parses --writeInterval correctly", "[Application]") {
    FakeArgv args{ "test_program", "--writeInterval", "10" };
    StubApplication app(args.argc(), args.argv());
    REQUIRE(app.getConfig().writeInterval == 10U);
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
// Sentinel thrown by the exit stub so --help tests stay in-process
// ─────────────────────────────────────────────────────────────
namespace { struct HelpExitCalled {}; }

// ─────────────────────────────────────────────────────────────
// Tests — ArgumentParser --help / -h
// ─────────────────────────────────────────────────────────────
TEST_CASE("ArgumentParser --help prints usage and triggers exit", "[ArgumentParser][help]") {
    ArgumentParser parser("my_prog",
        [](int) { throw HelpExitCalled{}; });
    parser.addArgument("--foo", "A foo argument", [](const std::string&) {});

    std::ostringstream buf;
    auto* old = std::cerr.rdbuf(buf.rdbuf());
    bool exited = false;
    try {
        FakeArgv fakeArgv{ "my_prog", "--help" };
        parser.parse(fakeArgv.argc(), fakeArgv.argv());
    }
    catch (const HelpExitCalled&) { exited = true; }
    std::cerr.rdbuf(old);

    REQUIRE(exited);
    CHECK(buf.str().find("Usage:") != std::string::npos);
    CHECK(buf.str().find("my_prog") != std::string::npos);
    CHECK(buf.str().find("--foo") != std::string::npos);
}

TEST_CASE("ArgumentParser -h is an alias for --help", "[ArgumentParser][help]") {
    ArgumentParser parser("my_prog",
        [](int) { throw HelpExitCalled{}; });

    FakeArgv fakeArgv{ "my_prog", "-h" };
    REQUIRE_THROWS_AS(
        parser.parse(fakeArgv.argc(), fakeArgv.argv()),
        HelpExitCalled);
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

// ─────────────────────────────────────────────────────────────
// RunableStub — concrete Application that can actually run().
// stepAndRecord advances simulation time by the given step h.
// failOnStep (1-based) optionally reports one non-convergent step.
// ─────────────────────────────────────────────────────────────
class RunableStub : public Application
{
public:
    explicit RunableStub(FakeArgv& args, int failOnStep = -1)
        : Application(args.argc(), args.argv())
        , failOnStep_(failOnStep)
        , time_(getConfig().startTime)
    {
    }

    int stepsCalled() const { return stepsCalled_; }

protected:
    void prepareSimulation()                  const override {}
    void writeInitialSnapshot(std::ofstream&) const override {}
    void logFinalSummary()                    const override {}

    StepObservation stepAndRecord(std::ofstream&, double h, bool) const override
    {
        ++stepsCalled_;
        time_ += h;
        StepObservation obs;
        obs.time_s    = time_;
        obs.converged = (stepsCalled_ != failOnStep_);
        obs.residual  = obs.converged ? 0.0 : 1.23e-4;
        return obs;
    }

private:
    int            failOnStep_;
    mutable int    stepsCalled_ = 0;
    mutable double time_        = 0.0;
};

// ─────────────────────────────────────────────────────────────
// Tests — Application::run()
// Covers: logSimulationParameters, openOutputCsv (happy path
// and error path), run, runTimeStepLoop (convergence warn,
// per-10-step progress log).
// ─────────────────────────────────────────────────────────────
TEST_CASE("Application::run completes a short simulation", "[Application][run]") {
    const std::string outFile = "_ae_run_test_output.csv";
    FakeArgv args{ "test_program",
        "--timeStep",       "0.1",
        "--endTime",        "0.3",
        "--outputFileName", outFile.c_str() };
    RunableStub app(args);

    REQUIRE_NOTHROW(app.run());
    CHECK(app.stepsCalled() == 3);

    std::remove(outFile.c_str());
}

TEST_CASE("Application::run continues past a non-converging step", "[Application][run]") {
    const std::string outFile = "_ae_run_test_nonconv.csv";
    FakeArgv args{ "test_program",
        "--timeStep",       "0.1",
        "--endTime",        "0.3",
        "--outputFileName", outFile.c_str() };
    RunableStub app(args, /*failOnStep=*/2);

    REQUIRE_NOTHROW(app.run());
    CHECK(app.stepsCalled() == 3);

    std::remove(outFile.c_str());
}

TEST_CASE("Application::run exercises per-10-step progress logging", "[Application][run]") {
    const std::string outFile = "_ae_run_test_10steps.csv";
    FakeArgv args{ "test_program",
        "--timeStep",       "0.1",
        "--endTime",        "2.0",
        "--outputFileName", outFile.c_str() };
    RunableStub app(args);

    REQUIRE_NOTHROW(app.run());
    CHECK(app.stepsCalled() == 20);

    std::remove(outFile.c_str());
}

TEST_CASE("Application::run throws std::runtime_error for unwritable output path",
          "[Application][run][openOutputCsv]") {
    FakeArgv args{ "test_program",
        "--outputFileName", "nonexistent_subdir_xyz/output.csv" };
    RunableStub app(args);

    REQUIRE_THROWS_AS(app.run(), std::runtime_error);
}

// ─────────────────────────────────────────────────────────────
// Tests — Log
// Log::Init() registers named spdlog loggers and may only be
// called once per process. All assertions are in a single
// TEST_CASE so Init() is invoked exactly once.
// ─────────────────────────────────────────────────────────────
TEST_CASE("Log: Init populates core and client loggers", "[Log]") {
    using namespace Aetherion::Simulation;
    Log::Init();

    REQUIRE(Log::GetCoreLogger() != nullptr);
    REQUIRE(Log::GetClientLogger() != nullptr);
    REQUIRE(Log::GetCoreLogger() == Log::GetCoreLogger());
    REQUIRE(Log::GetClientLogger() == Log::GetClientLogger());
}