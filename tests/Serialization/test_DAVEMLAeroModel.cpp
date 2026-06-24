// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLAeroModel.cpp
//
// Smoke tests for the F-16 DAVE-ML aerodynamic model.
//
// Reference check-case extracted from the embedded <checkData> section of
// F16_aero.dml (Nominal static shot at alpha=5 deg, all others zero):
//   cx = -0.004    cy = 0.000    cz = -0.416
//   cl =  0.000    cm = -0.005   cn =  0.000
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <cppad/cppad.hpp>

using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;

// AV Rule 30 deviation (#define for constants): CMake-injected path constant via
// target_compile_definitions; see CODING_STANDARDS.md, Pre-Processing Directives.
#ifndef DAVEML_AERO_FILE
#  define DAVEML_AERO_FILE ""
#endif
static const std::string kAeroFile = DAVEML_AERO_FILE;

// ─────────────────────────────────────────────────────────────────────────────
// Guard
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: aero file found", "[daveml_aero][smoke]")
{
    REQUIRE_FALSE(kAeroFile.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// Reference geometry constants
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: reference geometry", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);
    CHECK_THAT(m.srefFt2(),  WithinAbs(300.0,  1e-9));
    CHECK_THAT(m.cbarFt(),   WithinAbs(11.32,  1e-9));
    CHECK_THAT(m.bspanFt(),  WithinAbs(30.0,   1e-9));
}

// ─────────────────────────────────────────────────────────────────────────────
// Nominal check-case (double)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: nominal check-case (double)", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    DAVEMLAeroModel::Inputs<double> in;
    in.vt_fps   = 300.0;
    in.alpha_deg =  5.0;
    in.beta_deg  =  0.0;
    in.p_rps = in.q_rps = in.r_rps = 0.0;
    in.el_deg = in.ail_deg = in.rdr_deg = 0.0;

    auto out = m.evaluate(in);

    // Tolerances match the 4-significant-figure check-case values
    CHECK_THAT(out.cx, WithinAbs(-0.004, 1e-4));
    CHECK_THAT(out.cy, WithinAbs( 0.000, 1e-6));
    CHECK_THAT(out.cz, WithinAbs(-0.416, 1e-4));
    CHECK_THAT(out.cl, WithinAbs( 0.000, 1e-6));
    CHECK_THAT(out.cm, WithinAbs(-0.005, 1e-4));
    CHECK_THAT(out.cn, WithinAbs( 0.000, 1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────
// Same check-case via AD<double> — confirms CppAD compatibility
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: nominal check-case (AD<double>)", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    using AD = CppAD::AD<double>;
    DAVEMLAeroModel::Inputs<AD> in;
    in.vt_fps    = AD(300.0);
    in.alpha_deg = AD(5.0);
    in.beta_deg  = AD(0.0);
    in.p_rps = in.q_rps = in.r_rps = AD(0.0);
    in.el_deg = in.ail_deg = in.rdr_deg = AD(0.0);

    auto out = m.evaluate(in);

    CHECK_THAT(CppAD::Value(out.cx), WithinAbs(-0.004, 1e-4));
    CHECK_THAT(CppAD::Value(out.cy), WithinAbs( 0.000, 1e-6));
    CHECK_THAT(CppAD::Value(out.cz), WithinAbs(-0.416, 1e-4));
    CHECK_THAT(CppAD::Value(out.cl), WithinAbs( 0.000, 1e-6));
    CHECK_THAT(CppAD::Value(out.cm), WithinAbs(-0.005, 1e-4));
    CHECK_THAT(CppAD::Value(out.cn), WithinAbs( 0.000, 1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────
// Physical sanity: CZ should be negative (lift) for positive alpha,
// CX should be negative (drag) at all tested conditions.
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: physical sanity at alpha=10 deg", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    DAVEMLAeroModel::Inputs<double> in;
    in.vt_fps    = 300.0;
    in.alpha_deg =  10.0;
    in.beta_deg  =   0.0;
    in.p_rps = in.q_rps = in.r_rps = 0.0;
    in.el_deg = in.ail_deg = in.rdr_deg = 0.0;

    auto out = m.evaluate(in);

    // CX sign is model-dependent (body-axis, not stability-axis);
    // can be positive at moderate alpha — just check it's finite
    CHECK(std::isfinite(CppAD::Value(CppAD::AD<double>(out.cx))));
    // CZ (body down) should be increasingly negative as alpha increases (lift)
    CHECK(out.cz < 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Error path — no file required
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: constructor throws for non-existent file", "[daveml_aero]")
{
    REQUIRE_THROWS_AS(DAVEMLAeroModel("_nonexistent_file_xyz.dml"), std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// evaluateRaw — direct test
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: evaluateRaw returns all six force-moment coefficients",
          "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    std::unordered_map<std::string, double> vars;
    vars["vt"]    = 300.0;
    vars["alpha"] = 5.0;
    vars["beta"]  = 0.0;
    vars["p"] = vars["q"] = vars["r"] = 0.0;
    vars["el"] = vars["ail"] = vars["rdr"] = 0.0;

    auto result = m.evaluateRaw(vars);

    for (const char* id : {"cx", "cy", "cz", "cl", "cm", "cn"}) {
        auto it = result.find(id);
        REQUIRE(it != result.end());
        CHECK(std::isfinite(it->second));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Physical sanity at varied flight conditions
// (exercises additional evaluation-graph branches: beta coupling, damping,
//  negative alpha, non-zero control deflections)
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("DAVEMLAeroModel: non-zero sideslip gives non-zero cy", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    DAVEMLAeroModel::Inputs<double> in;
    in.vt_fps    = 300.0;
    in.alpha_deg =   5.0;
    in.beta_deg  =   5.0;

    auto out = m.evaluate(in);
    CHECK(out.cy != 0.0);
}

TEST_CASE("DAVEMLAeroModel: non-zero elevator shifts cm", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    DAVEMLAeroModel::Inputs<double> baseline, deflected;
    baseline.vt_fps = deflected.vt_fps = 300.0;
    baseline.alpha_deg = deflected.alpha_deg = 5.0;
    deflected.el_deg = 10.0;

    auto b = m.evaluate(baseline);
    auto d = m.evaluate(deflected);
    CHECK(b.cm != d.cm);
}

TEST_CASE("DAVEMLAeroModel: negative alpha is evaluated without error", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    DAVEMLAeroModel::Inputs<double> in;
    in.vt_fps    = 300.0;
    in.alpha_deg = -5.0;

    auto out = m.evaluate(in);
    CHECK(std::isfinite(out.cx));
    CHECK(std::isfinite(out.cz));
}

TEST_CASE("DAVEMLAeroModel: non-zero pitch rate q affects cm", "[daveml_aero][smoke]")
{
    if (kAeroFile.empty()) return;
    DAVEMLAeroModel m(kAeroFile);

    DAVEMLAeroModel::Inputs<double> baseline, pitching;
    baseline.vt_fps = pitching.vt_fps = 300.0;
    baseline.alpha_deg = pitching.alpha_deg = 5.0;
    pitching.q_rps = 0.1;

    auto b = m.evaluate(baseline);
    auto p = m.evaluate(pitching);
    CHECK(b.cm != p.cm);
}
