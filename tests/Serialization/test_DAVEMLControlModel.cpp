// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLControlModel.cpp
//
// Smoke tests for the F-16 DAVE-ML control / GNC model.
// Exercises DAVEMLControlModel::evaluate() and the underlying
// DAVEMLAeroModel::evaluateRaw() path.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLControlModel.h>

#include <cmath>

using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;

#ifndef DAVEML_CONTROL_FILE
#  define DAVEML_CONTROL_FILE ""
#endif
static const std::string kControlFile = DAVEML_CONTROL_FILE;

// ─────────────────────────────────────────────────────────────────────────────
// Guard
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLControlModel: control file found", "[daveml_ctrl][smoke]")
{
    REQUIRE_FALSE(kControlFile.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// Error path
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLControlModel: constructor throws for non-existent file", "[daveml_ctrl]")
{
    REQUIRE_THROWS_AS(DAVEMLControlModel("_nonexistent_ctrl.dml"), std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// Nominal outputs — all quantities finite
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLControlModel: evaluate at trim returns finite outputs", "[daveml_ctrl][smoke]")
{
    if (kControlFile.empty()) return;
    DAVEMLControlModel ctrl(kControlFile);

    DAVEMLControlModel::Inputs in;
    in.sasOn    = 1.0;
    in.apOn     = 0.0;
    in.altMsl_ft = 10013.0;
    in.Vequiv_kt = 300.0;
    in.alpha_deg =   2.65;

    auto out = ctrl.evaluate(in);

    CHECK(std::isfinite(out.el_deg));
    CHECK(std::isfinite(out.ail_deg));
    CHECK(std::isfinite(out.rdr_deg));
    CHECK(std::isfinite(out.pwr_pct));
}

// ─────────────────────────────────────────────────────────────────────────────
// Physical sanity
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLControlModel: SAS-off vs SAS-on differ in elevator output",
          "[daveml_ctrl][smoke]")
{
    if (kControlFile.empty()) return;
    DAVEMLControlModel ctrl(kControlFile);

    DAVEMLControlModel::Inputs base;
    base.apOn      = 0.0;      // isolate SAS from autopilot demands
    base.altMsl_ft = 10013.0;
    base.Vequiv_kt = 300.0;
    base.alpha_deg =   2.65;   // near trim — keeps elevator away from hard-stops
    base.qb_rad_s  =   0.05;   // nonzero pitch rate excites SAS

    auto off = base; off.sasOn = 0.0;
    auto on  = base; on.sasOn  = 1.0;

    auto out_off = ctrl.evaluate(off);
    auto out_on  = ctrl.evaluate(on);

    // SAS modifies elevator based on body rates — outputs should differ
    CHECK(out_off.el_deg != out_on.el_deg);
}

TEST_CASE("DAVEMLControlModel: autopilot altitude error drives elevator",
          "[daveml_ctrl][smoke]")
{
    if (kControlFile.empty()) return;
    DAVEMLControlModel ctrl(kControlFile);

    DAVEMLControlModel::Inputs base;
    base.sasOn         = 1.0;
    base.apOn          = 1.0;
    // Place the aircraft at the NASA reference trim point to avoid any
    // pilot-stick bias saturating the elevator before the AP term takes effect:
    base.altMsl_ft     = 10013.0;
    base.Vequiv_kt     = 300.0;
    base.keasCmd_kt    = 300.0;    // zero airspeed error
    base.alpha_deg     =   2.65;
    base.theta_deg     =   2.65;   // pitch ≈ AoA in level flight
    base.longStk_frac  = base.longStkTrim;  // pilot stick at trim — zero stick bias
    base.throttle_frac = base.throttleTrim; // throttle at trim

    auto hold    = base; hold.altCmd_ft    = 10013.0;  // on-altitude, zero error
    auto climb   = base; climb.altCmd_ft   = 10513.0;  // +500 ft command

    auto out_hold  = ctrl.evaluate(hold);
    auto out_climb = ctrl.evaluate(climb);

    // A positive altitude error should produce a different elevator command
    CHECK(out_hold.el_deg != out_climb.el_deg);
}
