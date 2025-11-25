// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <ode/ode_rk_4.h>
#include <type_traits>

TEST_CASE("libode headers are available and types exist", "[libode]") {
    // Just basic type sanity checks; no instantiation
    STATIC_REQUIRE(std::is_class_v<ode::OdeBase>);
    STATIC_REQUIRE(std::is_class_v<ode::OdeRK4>);
}

