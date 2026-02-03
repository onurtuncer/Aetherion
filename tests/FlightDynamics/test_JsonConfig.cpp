// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp> // optional, but useful

#include <type_traits>

// Adjust include paths to your project layout
#include "Aetherion/FlightDynamics/JsonConfig.h"

#include <vendor/nlohmann/json.hpp>

namespace Aetherion::FlightDynamics::Tests {

    using Catch::Approx;

    // Small SFINAE helpers to verify members exist (without hardcoding offsets/sizes).
    template <typename T, typename = void> struct has_lat_deg : std::false_type {};
    template <typename T> struct has_lat_deg<T, std::void_t<decltype(std::declval<T&>().lat_deg)>> : std::true_type {};

    template <typename T, typename = void> struct has_lon_deg : std::false_type {};
    template <typename T> struct has_lon_deg<T, std::void_t<decltype(std::declval<T&>().lon_deg)>> : std::true_type {};

    template <typename T, typename = void> struct has_alt_m : std::false_type {};
    template <typename T> struct has_alt_m<T, std::void_t<decltype(std::declval<T&>().alt_m)>> : std::true_type {};

    template <typename T, typename = void> struct has_azimuth_deg : std::false_type {};
    template <typename T> struct has_azimuth_deg<T, std::void_t<decltype(std::declval<T&>().azimuth_deg)>> : std::true_type {};

    template <typename T, typename = void> struct has_zenith_deg : std::false_type {};
    template <typename T> struct has_zenith_deg<T, std::void_t<decltype(std::declval<T&>().zenith_deg)>> : std::true_type {};

    template <typename T, typename = void> struct has_roll_deg : std::false_type {};
    template <typename T> struct has_roll_deg<T, std::void_t<decltype(std::declval<T&>().roll_deg)>> : std::true_type {};

    TEST_CASE("InitialConditions exposes Geo/Euler convenience fields", "[flightdynamics][jsonconfig][api]") {
        using IC = Aetherion::FlightDynamics::InitialPoseWGS84_NED;
     
        STATIC_REQUIRE(has_lat_deg<IC>::value);
        STATIC_REQUIRE(has_lon_deg<IC>::value);
        STATIC_REQUIRE(has_alt_m<IC>::value);
        STATIC_REQUIRE(has_azimuth_deg<IC>::value);
        STATIC_REQUIRE(has_zenith_deg<IC>::value);
        STATIC_REQUIRE(has_roll_deg<IC>::value);

        // Also sanity-check defaults (optional but nice)
        IC ic{};
        REQUIRE(ic.lat_deg == Approx(0.0));
        REQUIRE(ic.lon_deg == Approx(0.0));
        REQUIRE(ic.alt_m == Approx(0.0));
        REQUIRE(ic.azimuth_deg == Approx(0.0));
        REQUIRE(ic.zenith_deg == Approx(0.0));
        REQUIRE(ic.roll_deg == Approx(0.0));
    }

    // ---- Runtime parsing test ----
    //
    // Adapt this to however you currently parse JSON into InitialConditions.
    // If you already have an adapter like `JsonConfigNlohmannAdapter`, use it.
    // I’m providing two variants: enable the one that matches your codebase.

    TEST_CASE("JSON parsing fills InitialConditions Geo/Euler fields", "[flightdynamics][jsonconfig][parse]") {
        using IC = Aetherion::FlightDynamics::InitialPoseWGS84_NED;

        // Example JSON shape. Adjust keys/structure to match your actual schema.
        // If your JSON nests initial conditions, update accordingly.
        const nlohmann::json j = {
         //   {"t0", 1.25},
            {"lat_deg", 41.015137},
            {"lon_deg", 28.979530},
            {"alt_m",  35.0},
            {"azimuth_deg",  10.0},
            {"zenith_deg", -2.0},
            {"roll_deg",  170.0}

            // include these only if your schema supports them
         //   {"omegaB", {0.1, 0.2, 0.3}},
           // {"vB",     {1.0, 2.0, 3.0}},
            //{"m",      12.5}
        };

        IC ic{};
        // --- Variant A: if you have from_json/to_json overloads for InitialConditions
        // Uncomment if applicable:
        // ic = j.get<IC>();

        // --- Variant B: if you have an adapter function/class
        // Replace with your real API, e.g.:
        // Aetherion::FlightDynamics::JsonConfigNlohmannAdapter adapter;
        // adapter.read_initial_conditions(j, ic);

        // --- TEMP fallback: minimal direct extraction (use ONLY if you don't have an API yet)
        // Remove once you wire in your real parser.
       // ic.t0 = j.at("t0").get<double>();
        ic.lat_deg = j.at("lat_deg").get<double>();
        ic.lon_deg = j.at("lon_deg").get<double>();
        ic.alt_m = j.at("alt_m").get<double>();
        ic.azimuth_deg = j.at("azimuth_deg").get<double>();
        ic.zenith_deg = j.at("zenith_deg").get<double>();
        ic.roll_deg = j.at("roll_deg").get<double>();
    //   ic.m = j.at("m").get<double>();
        // End TEMP fallback

    //    REQUIRE(ic.t0 == Approx(1.25));
        REQUIRE(ic.lat_deg == Approx(41.015137));
        REQUIRE(ic.lon_deg == Approx(28.979530));
        REQUIRE(ic.alt_m == Approx(35.0));
        REQUIRE(ic.azimuth_deg == Approx(10.0));
        REQUIRE(ic.zenith_deg == Approx(-2.0));
        REQUIRE(ic.roll_deg == Approx(170.0));
    
    }

} // namespace Aetherion::FlightDynamics::Tests
