// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_feedthrough.cpp
//
// Catch2 in-process integration test for the FeedThrough FMU.
// Tests the fmu4cpp FMI 2.0 Co-Simulation bindings end-to-end without loading
// a DLL: the model is compiled into the same executable alongside the fmu4cpp
// FMI 2.0 implementation objects (fmu4cpp_base + fmu4cpp_fmi2).
//
// Two test cases:
//   1. feed_through_values — set typed inputs, doStep, verify outputs match
//   2. reset_to_defaults   — confirm all outputs revert to 0/false/"empty"
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdarg>
#include <iostream>
#include <string>

#include "fmi2/fmi2Functions.h"
#include "FeedThrough.hpp"

// Required by model_identifier.cpp.in logic; this is the in-process equivalent
// of the FMU archive's model identifier string.
std::string fmu4cpp::model_identifier()
{
    return "FeedThrough_fmi2";
}

// ---------------------------------------------------------------------------
// FMI callback logger — forwards to stderr so CTest captures it
// ---------------------------------------------------------------------------
static void fmiLogger(fmi2Component,
                      fmi2String    instanceName,
                      fmi2Status    status,
                      fmi2String  /*category*/,
                      fmi2String    message, ...)
{
    const char* sstr = "OK";
    switch (status) {
        case fmi2Warning: sstr = "Warning"; break;
        case fmi2Error:   sstr = "Error";   break;
        case fmi2Fatal:   sstr = "Fatal";   break;
        default: break;
    }
    va_list args;
    va_start(args, message);
    char buf[512];
    std::vsnprintf(buf, sizeof(buf), message, args);
    va_end(args);
    std::cerr << sstr << ": [" << (instanceName ? instanceName : "") << "] " << buf << '\n';
}

// ---------------------------------------------------------------------------
// Helper: boot a slave through fmi2ExitInitializationMode
// ---------------------------------------------------------------------------
static fmi2Component boot(const std::string& guid,
                           const fmi2CallbackFunctions& cb)
{
    const auto c = fmi2Instantiate(
        "FeedThrough_test", fmi2CoSimulation,
        guid.c_str(), "", &cb, fmi2False, fmi2True);
    REQUIRE(c != nullptr);
    REQUIRE(fmi2SetupExperiment(c, fmi2False, 0.0, 0.0, fmi2False, 0.0) == fmi2OK);
    REQUIRE(fmi2EnterInitializationMode(c) == fmi2OK);
    REQUIRE(fmi2ExitInitializationMode(c) == fmi2OK);
    return c;
}

// ---------------------------------------------------------------------------
// Test 1 — Feed-through correctness
// ---------------------------------------------------------------------------
TEST_CASE("feed_through_values", "[FeedThrough][fmu4cpp]")
{
    // Inspect a temporary instance to discover GUIDs and value references
    FeedThrough probe({});
    const std::string guid = probe.guid();

    const fmi2ValueReference vrIntIn  = probe.get_int_variable("integerIn")->value_reference();
    const fmi2ValueReference vrRealIn = probe.get_real_variable("realIn")->value_reference();
    const fmi2ValueReference vrBoolIn = probe.get_bool_variable("booleanIn")->value_reference();
    const fmi2ValueReference vrStrIn  = probe.get_string_variable("stringIn")->value_reference();

    const fmi2ValueReference vrIntOut  = probe.get_int_variable("integerOut")->value_reference();
    const fmi2ValueReference vrRealOut = probe.get_real_variable("realOut")->value_reference();
    const fmi2ValueReference vrBoolOut = probe.get_bool_variable("booleanOut")->value_reference();
    const fmi2ValueReference vrStrOut  = probe.get_string_variable("stringOut")->value_reference();

    fmi2CallbackFunctions cb{};
    cb.logger = &fmiLogger;

    const auto c = boot(guid, cb);

    // ── Case 1: positive integer, non-zero real, true, non-empty string ──────
    {
        fmi2Integer intVal  = 42;
        fmi2Real    realVal = 3.14159;
        fmi2Boolean boolVal = fmi2True;
        fmi2String  strVal  = "aetherion";

        REQUIRE(fmi2SetInteger(c, &vrIntIn,  1, &intVal)  == fmi2OK);
        REQUIRE(fmi2SetReal   (c, &vrRealIn, 1, &realVal) == fmi2OK);
        REQUIRE(fmi2SetBoolean(c, &vrBoolIn, 1, &boolVal) == fmi2OK);
        REQUIRE(fmi2SetString (c, &vrStrIn,  1, &strVal)  == fmi2OK);

        REQUIRE(fmi2DoStep(c, 0.0, 0.1, fmi2True) == fmi2OK);

        fmi2Integer intOut{};
        fmi2Real    realOut{};
        fmi2Boolean boolOut{};
        fmi2String  strOut{};

        REQUIRE(fmi2GetInteger(c, &vrIntOut,  1, &intOut)  == fmi2OK);
        REQUIRE(fmi2GetReal   (c, &vrRealOut, 1, &realOut) == fmi2OK);
        REQUIRE(fmi2GetBoolean(c, &vrBoolOut, 1, &boolOut) == fmi2OK);
        REQUIRE(fmi2GetString (c, &vrStrOut,  1, &strOut)  == fmi2OK);

        CHECK(intOut  == 42);
        CHECK(realOut == Catch::Approx(3.14159));
        CHECK(boolOut == fmi2True);
        CHECK(std::string(strOut) == "aetherion");
    }

    // ── Case 2: negative integer, zero real, false, empty string ─────────────
    {
        fmi2Integer intVal  = -7;
        fmi2Real    realVal = 0.0;
        fmi2Boolean boolVal = fmi2False;
        fmi2String  strVal  = "";

        REQUIRE(fmi2SetInteger(c, &vrIntIn,  1, &intVal)  == fmi2OK);
        REQUIRE(fmi2SetReal   (c, &vrRealIn, 1, &realVal) == fmi2OK);
        REQUIRE(fmi2SetBoolean(c, &vrBoolIn, 1, &boolVal) == fmi2OK);
        REQUIRE(fmi2SetString (c, &vrStrIn,  1, &strVal)  == fmi2OK);

        REQUIRE(fmi2DoStep(c, 0.1, 0.1, fmi2True) == fmi2OK);

        fmi2Integer intOut{};
        fmi2Real    realOut{};
        fmi2Boolean boolOut{};
        fmi2String  strOut{};

        REQUIRE(fmi2GetInteger(c, &vrIntOut,  1, &intOut)  == fmi2OK);
        REQUIRE(fmi2GetReal   (c, &vrRealOut, 1, &realOut) == fmi2OK);
        REQUIRE(fmi2GetBoolean(c, &vrBoolOut, 1, &boolOut) == fmi2OK);
        REQUIRE(fmi2GetString (c, &vrStrOut,  1, &strOut)  == fmi2OK);

        CHECK(intOut  == -7);
        CHECK(realOut == Catch::Approx(0.0));
        CHECK(boolOut == fmi2False);
        CHECK(std::string(strOut) == "");
    }

    REQUIRE(fmi2Terminate(c) == fmi2OK);
    fmi2FreeInstance(c);
}

// ---------------------------------------------------------------------------
// Test 2 — Reset returns all outputs to defaults
// ---------------------------------------------------------------------------
TEST_CASE("reset_to_defaults", "[FeedThrough][fmu4cpp]")
{
    FeedThrough probe({});
    const std::string guid = probe.guid();

    const fmi2ValueReference vrIntIn  = probe.get_int_variable("integerIn")->value_reference();
    const fmi2ValueReference vrRealIn = probe.get_real_variable("realIn")->value_reference();
    const fmi2ValueReference vrBoolIn = probe.get_bool_variable("booleanIn")->value_reference();
    const fmi2ValueReference vrStrIn  = probe.get_string_variable("stringIn")->value_reference();

    const fmi2ValueReference vrIntOut  = probe.get_int_variable("integerOut")->value_reference();
    const fmi2ValueReference vrRealOut = probe.get_real_variable("realOut")->value_reference();
    const fmi2ValueReference vrBoolOut = probe.get_bool_variable("booleanOut")->value_reference();
    const fmi2ValueReference vrStrOut  = probe.get_string_variable("stringOut")->value_reference();

    fmi2CallbackFunctions cb{};
    cb.logger = &fmiLogger;

    const auto c = boot(guid, cb);

    // Drive non-default values through
    fmi2Integer intVal  = 99;
    fmi2Real    realVal = 2.718;
    fmi2Boolean boolVal = fmi2True;
    fmi2String  strVal  = "before_reset";

    REQUIRE(fmi2SetInteger(c, &vrIntIn,  1, &intVal)  == fmi2OK);
    REQUIRE(fmi2SetReal   (c, &vrRealIn, 1, &realVal) == fmi2OK);
    REQUIRE(fmi2SetBoolean(c, &vrBoolIn, 1, &boolVal) == fmi2OK);
    REQUIRE(fmi2SetString (c, &vrStrIn,  1, &strVal)  == fmi2OK);
    REQUIRE(fmi2DoStep(c, 0.0, 0.1, fmi2True) == fmi2OK);

    // Reset and re-initialise
    REQUIRE(fmi2Reset(c) == fmi2OK);
    REQUIRE(fmi2SetupExperiment(c, fmi2False, 0.0, 0.0, fmi2False, 0.0) == fmi2OK);
    REQUIRE(fmi2EnterInitializationMode(c) == fmi2OK);
    REQUIRE(fmi2ExitInitializationMode(c) == fmi2OK);
    REQUIRE(fmi2DoStep(c, 0.0, 0.1, fmi2True) == fmi2OK);

    fmi2Integer intOut{};
    fmi2Real    realOut{};
    fmi2Boolean boolOut{};
    fmi2String  strOut{};

    REQUIRE(fmi2GetInteger(c, &vrIntOut,  1, &intOut)  == fmi2OK);
    REQUIRE(fmi2GetReal   (c, &vrRealOut, 1, &realOut) == fmi2OK);
    REQUIRE(fmi2GetBoolean(c, &vrBoolOut, 1, &boolOut) == fmi2OK);
    REQUIRE(fmi2GetString (c, &vrStrOut,  1, &strOut)  == fmi2OK);

    CHECK(intOut  == 0);
    CHECK(realOut == Catch::Approx(0.0));
    CHECK(boolOut == fmi2False);
    CHECK(std::string(strOut) == "empty");

    REQUIRE(fmi2Terminate(c) == fmi2OK);
    fmi2FreeInstance(c);
}
