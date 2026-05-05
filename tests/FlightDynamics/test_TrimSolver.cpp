// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_TrimSolver.cpp
//
// Smoke tests for the Newton-Raphson trim solver using the F-16 DAVE-ML models.
//
// Reference flight condition: NASA TM-2015-218675, Check-Case 11
//   V = 565.685 ft/s  (335.15 KTAS)
//   h = 10 013 ft
//   α ≈ 2.643°,  δe ≈ −1.24°  (from reference initial state)
//   Body rates = 0,  β = 0
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16AeroPolicy.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16PropPolicy.h>

using namespace Aetherion::FlightDynamics;
using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// Injected at CMake configure time (see CMakeLists.txt)
#ifndef DAVEML_F16_AERO_FILE
#  define DAVEML_F16_AERO_FILE ""
#endif
#ifndef DAVEML_F16_PROP_FILE
#  define DAVEML_F16_PROP_FILE ""
#endif
static const std::string kAeroFile = DAVEML_F16_AERO_FILE;
static const std::string kPropFile = DAVEML_F16_PROP_FILE;

// ── Scenario-11 flight condition (English units) ──────────────────────────────
static constexpr double kVt_fps     = 565.685;   // 335.15 KTAS → ft/s
static constexpr double kAlt_ft     = 10013.0;
// Weight derived from CZ force at trim:  W = −CZ_aero / cos(α)
// Reference CZ_force = −20 423.7 lbf, α ≈ 2.643°
static constexpr double kWeight_lbf = 20447.0;   // [lbf]

// ── Reference trim values from NASA Atmos_11 ─────────────────────────────────
static constexpr double kRefAlpha_deg = 2.643;   // [deg] ± 0.1

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST_CASE("TrimSolver: model files are present", "[trim][smoke]")
{
    REQUIRE_FALSE(kAeroFile.empty());
    REQUIRE_FALSE(kPropFile.empty());
}

TEST_CASE("TrimSolver: converges for Scenario-11 flight condition", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);

    INFO("alpha=" << tp.alpha_deg << "  el=" << tp.el_deg
         << "  pwr=" << tp.pwr_pct << "  |r|=" << tp.residual_norm);

    CHECK(tp.converged);
    CHECK(tp.residual_norm < 0.1);  // residual in lbf
}

TEST_CASE("TrimSolver: trim alpha matches NASA reference", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);

    CHECK_THAT(tp.alpha_deg, WithinAbs(kRefAlpha_deg, 0.3));
}

TEST_CASE("TrimSolver: force balance at trim", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);
    const auto r = solver.residual(in, tp.alpha_deg, tp.el_deg, tp.pwr_pct);

    // Each force residual < 1 lbf (< 0.005% of weight)
    CHECK_THAT(r(0), WithinAbs(0.0, 1.0));  // body-X force [lbf]
    CHECK_THAT(r(1), WithinAbs(0.0, 1.0));  // body-Z force [lbf]
    CHECK_THAT(r(2), WithinAbs(0.0, 50.0)); // pitch moment [ft·lbf]
}

TEST_CASE("TrimSolver: throttle and elevator in physical range", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);

    CHECK(tp.pwr_pct >= 0.0);
    CHECK(tp.pwr_pct <= 100.0);
    CHECK(tp.el_deg  >= -25.0);
    CHECK(tp.el_deg  <=  25.0);
}

TEST_CASE("TrimSolver: F16AeroPolicy concept conformance", "[trim][concept]")
{
    static_assert(AeroPolicy<F16AeroPolicy>,
        "F16AeroPolicy must satisfy AeroPolicy");
}

TEST_CASE("TrimSolver: F16PropPolicy concept conformance", "[trim][concept]")
{
    static_assert(PropulsionPolicy<F16PropPolicy>,
        "F16PropPolicy must satisfy PropulsionPolicy");
}
