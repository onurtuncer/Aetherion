// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_feedthrough_ecos.cpp
//
// End-to-end smoke test for the packaged FeedThrough.fmu: loads the real .fmu
// archive (unzip, modelDescription.xml parsing, shared-library load) through
// ecos, rather than linking the fmu4cpp model objects in-process. This is the
// only test in the suite that exercises the actual FMU packaging pipeline.
//
// Three test cases:
//   1. feed_through_values_ecos       -- positive/non-zero/true/non-empty inputs pass through
//   2. feed_through_zero_values_ecos  -- negative/zero/false/empty inputs pass through
//   3. reset_to_defaults_ecos         -- confirm all outputs revert to 0/false/"empty"
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <ecos/algorithm/fixed_step_algorithm.hpp>
#include <ecos/simulation.hpp>
#include <ecos/structure/simulation_structure.hpp>

#include <filesystem>
#include <memory>
#include <numbers>

namespace
{
constexpr const char* kInstance = "feedthrough";
constexpr double      kStepSize = 0.1;

std::unique_ptr<ecos::simulation> boot()
{
    ecos::simulation_structure ss;
    ss.add_model(kInstance, std::filesystem::path(FEEDTHROUGH_FMU_PATH));

    auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(kStepSize));
    REQUIRE(sim != nullptr);
    sim->init();
    return sim;
}
} // namespace

// ---------------------------------------------------------------------------
// Test 1a -- Feed-through: positive integer, non-zero real, true, non-empty string
// ---------------------------------------------------------------------------
TEST_CASE("feed_through_values_ecos", "[FeedThrough][ecos]")
{
    auto sim = boot();

    sim->get_int_property("feedthrough::integerIn")->set_value(42);
    sim->get_real_property("feedthrough::realIn")->set_value(std::numbers::pi);
    sim->get_bool_property("feedthrough::booleanIn")->set_value(true);
    sim->get_string_property("feedthrough::stringIn")->set_value("aetherion");

    sim->step_for(kStepSize);

    CHECK(sim->get_int_property("feedthrough::integerOut")->get_value() == 42);
    CHECK(sim->get_real_property("feedthrough::realOut")->get_value() == Catch::Approx(std::numbers::pi));
    CHECK(sim->get_bool_property("feedthrough::booleanOut")->get_value() == true);
    CHECK(sim->get_string_property("feedthrough::stringOut")->get_value() == "aetherion");

    sim->terminate();
}

// ---------------------------------------------------------------------------
// Test 1b -- Feed-through: negative integer, zero real, false, empty string
// ---------------------------------------------------------------------------
TEST_CASE("feed_through_zero_values_ecos", "[FeedThrough][ecos]")
{
    auto sim = boot();

    sim->get_int_property("feedthrough::integerIn")->set_value(-7);
    sim->get_real_property("feedthrough::realIn")->set_value(0.0);
    sim->get_bool_property("feedthrough::booleanIn")->set_value(false);
    sim->get_string_property("feedthrough::stringIn")->set_value("");

    sim->step_for(kStepSize);

    CHECK(sim->get_int_property("feedthrough::integerOut")->get_value() == -7);
    CHECK(sim->get_real_property("feedthrough::realOut")->get_value() == Catch::Approx(0.0));
    CHECK(sim->get_bool_property("feedthrough::booleanOut")->get_value() == false);
    CHECK(sim->get_string_property("feedthrough::stringOut")->get_value().empty());

    sim->terminate();
}

// ---------------------------------------------------------------------------
// Test 2 -- Reset returns all outputs to defaults
// ---------------------------------------------------------------------------
TEST_CASE("reset_to_defaults_ecos", "[FeedThrough][ecos]")
{
    auto sim = boot();

    // Drive non-default values through
    sim->get_int_property("feedthrough::integerIn")->set_value(99);
    sim->get_real_property("feedthrough::realIn")->set_value(std::numbers::e);
    sim->get_bool_property("feedthrough::booleanIn")->set_value(true);
    sim->get_string_property("feedthrough::stringIn")->set_value("before_reset");
    sim->step_for(kStepSize);

    // Reset and re-initialise
    sim->reset();
    sim->init();
    sim->step_for(kStepSize);

    CHECK(sim->get_int_property("feedthrough::integerOut")->get_value() == 0);
    CHECK(sim->get_real_property("feedthrough::realOut")->get_value() == Catch::Approx(0.0));
    CHECK(sim->get_bool_property("feedthrough::booleanOut")->get_value() == false);
    CHECK(sim->get_string_property("feedthrough::stringOut")->get_value() == "empty");

    sim->terminate();
}
