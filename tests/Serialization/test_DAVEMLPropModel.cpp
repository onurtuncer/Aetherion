// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLPropModel.cpp
//
// Smoke tests for the F-16 DAVE-ML propulsion model.
// Check-case values extracted from the embedded <checkData> in F16_prop.dml.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <cppad/cppad.hpp>

using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

#ifndef DAVEML_PROP_FILE
#  define DAVEML_PROP_FILE ""
#endif
static const std::string kPropFile = DAVEML_PROP_FILE;

constexpr double kLbf_N = DAVEMLPropModel::kLbf_N;

TEST_CASE("DAVEMLPropModel: prop file found", "[daveml_prop][smoke]")
{
    REQUIRE_FALSE(kPropFile.empty());
}

// ── Check-case: idle at sea level, Mach 0 ─────────────────────────────────
// Expected: FEX = 1 060 lbf

TEST_CASE("DAVEMLPropModel: idle power, sea level, Mach 0", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 0.0, 0.0, 0.0 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(1060.0, 0.01));
    CHECK_THAT(out.fy_N,          WithinAbs(0.0, 1e-6));
    CHECK_THAT(out.fz_N,          WithinAbs(0.0, 1e-6));
}

// ── Check-case: MIL power (50%), sea level, Mach 0 ────────────────────────
// Expected: FEX = 12 680 lbf

TEST_CASE("DAVEMLPropModel: military power, sea level, Mach 0", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 50.0, 0.0, 0.0 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(12680.0, 0.01));
}

// ── Check-case: MAX power (100%), sea level, Mach 0 ──────────────────────
// Expected: FEX = 20 000 lbf

TEST_CASE("DAVEMLPropModel: max power, sea level, Mach 0", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 100.0, 0.0, 0.0 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(20000.0, 0.01));
}

// ── Check-case: MAX power, sea level, Mach 1.0 ───────────────────────────
// Expected: FEX = 28 885 lbf

TEST_CASE("DAVEMLPropModel: max power, sea level, Mach 1.0", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 100.0, 0.0, 1.0 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(28885.0, 0.01));
}

// ── Check-case: MAX power, 50 kft, Mach 1.0 ──────────────────────────────
// Expected: FEX = 5 057 lbf

TEST_CASE("DAVEMLPropModel: max power, 50000 ft, Mach 1.0", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 100.0, 50000.0, 1.0 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(5057.0, 0.01));
}

// ── Check-case: below-MIL at mid-envelope ────────────────────────────────
// PWR=42.3, ALT=23507, Mach=0.625  → FEX = 5319.3491 lbf (tol 0.001)

TEST_CASE("DAVEMLPropModel: below-MIL power, mid-envelope", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 42.3, 23507.0, 0.625 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(5319.3491, 0.002));
}

// ── Check-case: above-MIL at mid-envelope ────────────────────────────────
// PWR=88.3, ALT=33537, Mach=0.895  → FEX = 9298.8926 lbf (tol 0.001)

TEST_CASE("DAVEMLPropModel: above-MIL power, mid-envelope", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto out = m.evaluate(DAVEMLPropModel::Inputs<double>{ 88.3, 33537.0, 0.895 });
    CHECK_THAT(out.fx_N / kLbf_N, WithinAbs(9298.8926, 0.002));
}

// ── AD<double> path ───────────────────────────────────────────────────────

TEST_CASE("DAVEMLPropModel: MIL power via AD<double>", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    using AD = CppAD::AD<double>;
    auto out = m.evaluate(DAVEMLPropModel::Inputs<AD>{ AD(50.0), AD(0.0), AD(0.0) });
    CHECK_THAT(CppAD::Value(out.fx_N) / kLbf_N, WithinAbs(12680.0, 0.01));
}

// ── Physical sanity ───────────────────────────────────────────────────────

TEST_CASE("DAVEMLPropModel: thrust increases with throttle", "[daveml_prop][smoke]")
{
    if (kPropFile.empty()) return;
    DAVEMLPropModel m(kPropFile);

    auto idle = m.evaluate(DAVEMLPropModel::Inputs<double>{   0.0, 10000.0, 0.4 });
    auto mil  = m.evaluate(DAVEMLPropModel::Inputs<double>{  50.0, 10000.0, 0.4 });
    auto maxi = m.evaluate(DAVEMLPropModel::Inputs<double>{ 100.0, 10000.0, 0.4 });

    CHECK(idle.fx_N < mil.fx_N);
    CHECK(mil.fx_N  < maxi.fx_N);
}
