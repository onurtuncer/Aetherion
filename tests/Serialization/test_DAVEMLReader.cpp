// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLReader.cpp
//
// Smoke tests for the minimal DAVE-ML reader.
//
// Reads the F-16 inertia file (F16_inertia.dml) and checks that:
//   1. All declared varIDs are accessible.
//   2. Constant values match the <initialValue> attributes.
//   3. The calculated DXCG is evaluated correctly via MathML.
//   4. getValueSI() applies the correct unit conversions.
//   5. setInput() re-evaluates calculated outputs.
//   6. LoadInertiaFromDAVEML maps to InertialParameters correctly.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLReader.h>
#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>

#include <string>

using namespace Aetherion::Serialization;
using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

// Path injected by CMake via -DDAVEML_TEST_FILE=...
#ifndef DAVEML_TEST_FILE
#  define DAVEML_TEST_FILE ""
#endif

static const std::string kFile = DAVEML_TEST_FILE;

// ── Unit conversion constants ────────────────────────────────────────────────
constexpr double kSlugFt2_to_kgm2 = 1.355817948329279;
constexpr double kSlug_to_kg       = 14.593902937206364;
constexpr double kFt_to_m          = 0.3048;

// ─────────────────────────────────────────────────────────────────────────────
// Guard: skip all tests if no file path is configured
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLReader: F-16 inertia file found", "[daveml][smoke]")
{
    REQUIRE_FALSE(kFile.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// Constant variableDef values
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLReader: constant varIDs are present", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    DAVEMLReader r(kFile);

    CHECK(r.hasVar("XIXX"));
    CHECK(r.hasVar("XIYY"));
    CHECK(r.hasVar("XIZZ"));
    CHECK(r.hasVar("XIZX"));
    CHECK(r.hasVar("XIXY"));
    CHECK(r.hasVar("XIYZ"));
    CHECK(r.hasVar("XMASS"));
    CHECK(r.hasVar("DYCG"));
    CHECK(r.hasVar("DZCG"));
    CHECK(r.hasVar("DXCG"));
}

TEST_CASE("DAVEMLReader: native values match Stevens & Lewis table", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    DAVEMLReader r(kFile);

    CHECK_THAT(r.getValue("XIXX"),  WithinRel(9496.0,  1e-9));
    CHECK_THAT(r.getValue("XIYY"),  WithinRel(55814.0, 1e-9));
    CHECK_THAT(r.getValue("XIZZ"),  WithinRel(63100.0, 1e-9));
    CHECK_THAT(r.getValue("XIZX"),  WithinRel(982.0,   1e-9));
    CHECK_THAT(r.getValue("XIXY"),  WithinAbs(0.0,     1e-15));
    CHECK_THAT(r.getValue("XIYZ"),  WithinAbs(0.0,     1e-15));
    CHECK_THAT(r.getValue("XMASS"), WithinRel(637.1595, 1e-6));
    CHECK_THAT(r.getValue("DYCG"),  WithinAbs(0.0, 1e-15));
    CHECK_THAT(r.getValue("DZCG"),  WithinAbs(0.0, 1e-15));
}

// ─────────────────────────────────────────────────────────────────────────────
// SI conversion
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLReader: getValueSI converts slugft2 to kg·m²", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    DAVEMLReader r(kFile);

    CHECK_THAT(r.getValueSI("XIXX"),
        WithinRel(9496.0 * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(r.getValueSI("XIYY"),
        WithinRel(55814.0 * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(r.getValueSI("XIZZ"),
        WithinRel(63100.0 * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(r.getValueSI("XIZX"),
        WithinRel(982.0 * kSlugFt2_to_kgm2, 1e-9));
}

TEST_CASE("DAVEMLReader: getValueSI converts slug to kg", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    DAVEMLReader r(kFile);
    CHECK_THAT(r.getValueSI("XMASS"),
        WithinRel(637.1595 * kSlug_to_kg, 1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────
// MathML calculation: DXCG = 0.01 * CBAR * (35 - CG_PCT_MAC)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLReader: DXCG = 0 at reference CG (35% MAC)", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    DAVEMLReader r(kFile);
    // Default CG_PCT_MAC = 35.0 → DXCG = 0.01 * 11.32 * (35 - 35) = 0
    CHECK_THAT(r.getValue("DXCG"), WithinAbs(0.0, 1e-12));
}

TEST_CASE("DAVEMLReader: DXCG re-evaluates after setInput", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    DAVEMLReader r(kFile);

    // Move CG forward by 5% MAC:  DXCG = 0.01 * 11.32 * (35 - 30) = 0.566 ft
    r.setInput("CG_PCT_MAC", 30.0);
    const double expected_ft = 0.01 * 11.32 * (35.0 - 30.0);  // 0.566 ft
    CHECK_THAT(r.getValue("DXCG"), WithinRel(expected_ft, 1e-9));

    // In SI
    CHECK_THAT(r.getValueSI("DXCG"),
        WithinRel(expected_ft * kFt_to_m, 1e-9));
}

// ─────────────────────────────────────────────────────────────────────────────
// LoadInertiaFromDAVEML — end-to-end
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("LoadInertiaFromDAVEML: F-16 inertia at reference CG", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    const auto ip = LoadInertiaFromDAVEML(kFile);

    CHECK_THAT(ip.mass_kg, WithinRel(637.1595 * kSlug_to_kg, 1e-6));
    CHECK_THAT(ip.Ixx,     WithinRel(9496.0   * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(ip.Iyy,     WithinRel(55814.0  * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(ip.Izz,     WithinRel(63100.0  * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(ip.Ixz,     WithinRel(982.0    * kSlugFt2_to_kgm2, 1e-9));
    CHECK_THAT(ip.Ixy,     WithinAbs(0.0, 1e-12));
    CHECK_THAT(ip.Iyz,     WithinAbs(0.0, 1e-12));
    CHECK_THAT(ip.xbar_m,  WithinAbs(0.0, 1e-12));  // at reference CG
    CHECK_THAT(ip.ybar_m,  WithinAbs(0.0, 1e-12));
    CHECK_THAT(ip.zbar_m,  WithinAbs(0.0, 1e-12));
}

TEST_CASE("LoadInertiaFromDAVEML: F-16 inertia at non-reference CG", "[daveml][smoke]")
{
    if (kFile.empty()) return;
    const auto ip = LoadInertiaFromDAVEML(kFile, {{"CG_PCT_MAC", 30.0}});

    // xbar_m = (35 - 30) * 0.01 * 11.32 ft → m (forward = positive)
    const double expected_xbar = 0.01 * 11.32 * (35.0 - 30.0) * kFt_to_m;
    CHECK_THAT(ip.xbar_m, WithinRel(expected_xbar, 1e-9));
}
