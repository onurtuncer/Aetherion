// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_atmosphere_offsets.cpp
//
// US1976Atmosphere with AtmosphereOffsets: ISA + dT and a sea-level pressure
// offset.  The offsets are re-integrated hydrostatically, not scaled, and the
// zero-offset day reproduces the standard function bit for bit.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cppad/cppad.hpp>

#include <Aetherion/Environment/Atmosphere.h>

#include <array>
#include <cmath>

using namespace Aetherion::Environment;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
    constexpr double kG0 = 9.80665;
    constexpr double kR  = 287.05287;
    constexpr double kRe = 6356766.0;   // geopotential reference radius

    // Geometric altitude of a geopotential layer base.
    double geometricOf(double H_m) { return H_m * kRe / (kRe - H_m); }
}

TEST_CASE("AtmosphereOffsets: zero offsets are bit-identical to the standard day", "[US1976][offsets]")
{
    const AtmosphereOffsets zero{};
    REQUIRE(zero.isStandard());

    const double probe[] = { 0.0, 500.0, 11019.0, 15000.0, 20063.0, 32162.0, 47350.0,
                             51413.0, 71802.0, 84852.0, 86000.0, 95000.0 };
    for (double h : probe) {
        const auto a = US1976Atmosphere(h);
        const auto b = US1976Atmosphere(h, zero);
        CHECK(a.T   == b.T);
        CHECK(a.p   == b.p);
        CHECK(a.rho == b.rho);
        CHECK(a.a   == b.a);
    }
    for (int i = 0; i < 200; ++i) {
        const double h = 425.0 * i;
        CHECK(US1976Atmosphere(h).p   == US1976Atmosphere(h, zero).p);
        CHECK(US1976Atmosphere(h).rho == US1976Atmosphere(h, zero).rho);
    }
}

TEST_CASE("AtmosphereOffsets: ISA + 15 K at sea level", "[US1976][offsets]")
{
    const auto s = US1976Atmosphere(0.0, AtmosphereOffsets{ 15.0, 0.0 });
    CHECK_THAT(s.T,   WithinAbs(303.15, 1e-9));
    CHECK_THAT(s.p,   WithinAbs(101325.0, 1e-6));
    CHECK_THAT(s.rho, WithinRel(101325.0 / (kR * 303.15), 1e-9));   // 1.1644 kg/m^3
    CHECK_THAT(s.a,   WithinRel(std::sqrt(1.4 * kR * 303.15), 1e-9));
}

TEST_CASE("AtmosphereOffsets: a 10 hPa low reads 84 m high on a standard-ISA barometer",
          "[US1976][offsets]")
{
    // p at the surface on the low-pressure day
    const double p_surface = US1976Atmosphere(0.0, AtmosphereOffsets{ 0.0, -1000.0 }).p;
    CHECK_THAT(p_surface, WithinAbs(100325.0, 1e-6));

    // Altitude at which the STANDARD atmosphere has that pressure: bisection.
    double lo = 0.0, hi = 300.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (US1976Atmosphere(mid).p > p_surface ? lo : hi) = mid;
    }
    const double h_pressure = 0.5 * (lo + hi);
    CHECK_THAT(h_pressure, WithinAbs(83.6, 0.5));
}

TEST_CASE("AtmosphereOffsets: pressure is hydrostatic for any offsets", "[US1976][offsets]")
{
    // dp/dh = -rho g0 (Re/(Re+h))^2, the last factor converting the geopotential
    // gradient to geometric altitude.  A scaled-instead-of-reintegrated pressure
    // fails this the moment deltaT_K is non-zero.
    const AtmosphereOffsets cases[] = {
        { 0.0, 0.0 }, { 15.0, 0.0 }, { 0.0, -1000.0 }, { -20.0, 800.0 }, { 30.0, -2500.0 } };
    const double alts[] = { 500.0, 5000.0, 15000.0, 25000.0, 40000.0, 49000.0, 60000.0, 80000.0 };

    for (const auto& off : cases) {
        for (double h : alts) {
            const double d = 0.5;
            const double dp_dh = (US1976Atmosphere(h + d, off).p - US1976Atmosphere(h - d, off).p) / (2.0 * d);
            const auto   s     = US1976Atmosphere(h, off);
            const double geo   = kRe / (kRe + h);
            const double expected = -s.rho * kG0 * geo * geo;
            INFO("dT=" << off.deltaT_K << " dP=" << off.deltaP_sl_Pa << " h=" << h);
            CHECK_THAT(dp_dh, WithinRel(expected, 1e-7));
        }
    }
}

TEST_CASE("AtmosphereOffsets: pressure is continuous across layer bases", "[US1976][offsets]")
{
    const AtmosphereOffsets cases[] = { { 15.0, -1000.0 }, { -30.0, 2000.0 } };
    const double H_bases[] = { 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0 };
    for (const auto& off : cases) {
        for (double H : H_bases) {
            const double h  = geometricOf(H);
            const double pl = US1976Atmosphere(h - 0.01, off).p;
            const double pu = US1976Atmosphere(h + 0.01, off).p;
            INFO("dT=" << off.deltaT_K << " H=" << H);
            CHECK_THAT(pu, WithinRel(pl, 1e-4));   // the published table is itself rounded to ~1e-6
        }
    }
}

TEST_CASE("AtmosphereOffsets: a hot day is thinner near the surface and denser aloft",
          "[US1976][offsets]")
{
    // With the sea-level pressure held, a warmer column has larger scale
    // heights, so its pressure decays more slowly with altitude.  Near the
    // surface the temperature wins and the hot day is thinner; a few km up the
    // higher pressure wins and the hot day is denser.  Both are physics, not a
    // bug, and both must hold.
    for (double h = 0.0; h <= 4000.0; h += 1000.0) {
        const double rho_std  = US1976Atmosphere(h).rho;
        const double rho_hot  = US1976Atmosphere(h, AtmosphereOffsets{ 20.0, 0.0 }).rho;
        const double rho_cold = US1976Atmosphere(h, AtmosphereOffsets{ -20.0, 0.0 }).rho;
        INFO("h = " << h);
        CHECK(rho_hot  < rho_std);
        CHECK(rho_cold > rho_std);
    }
    for (double h = 10000.0; h <= 30000.0; h += 5000.0) {
        const double rho_std  = US1976Atmosphere(h).rho;
        const double rho_hot  = US1976Atmosphere(h, AtmosphereOffsets{ 20.0, 0.0 }).rho;
        const double rho_cold = US1976Atmosphere(h, AtmosphereOffsets{ -20.0, 0.0 }).rho;
        INFO("h = " << h);
        CHECK(rho_hot  > rho_std);
        CHECK(rho_cold < rho_std);
    }
    // A pressure offset alone scales the whole column.
    const double ratio0 = US1976Atmosphere(0.0,     AtmosphereOffsets{ 0.0, 2000.0 }).p / US1976Atmosphere(0.0).p;
    const double ratio1 = US1976Atmosphere(10000.0, AtmosphereOffsets{ 0.0, 2000.0 }).p / US1976Atmosphere(10000.0).p;
    CHECK_THAT(ratio1, WithinRel(ratio0, 1e-9));
}

TEST_CASE("AtmosphereOffsets: memo returns the right table when offsets alternate", "[US1976][offsets]")
{
    const AtmosphereOffsets A{ 12.0, -300.0 };
    const AtmosphereOffsets B{ -7.0, 1200.0 };
    const double h = 8000.0;
    const double pA = US1976Atmosphere(h, A).p;
    const double pB = US1976Atmosphere(h, B).p;
    for (int i = 0; i < 5; ++i) {
        CHECK(US1976Atmosphere(h, A).p == pA);
        CHECK(US1976Atmosphere(h, B).p == pB);
        CHECK(US1976Atmosphere(h, AtmosphereOffsets{}).p == US1976Atmosphere(h).p);
    }
    CHECK(pA != pB);
}

TEST_CASE("AtmosphereOffsets: evaluates with CppAD::AD<double>", "[US1976][offsets][AD]")
{
    using AD = CppAD::AD<double>;
    const AtmosphereOffsets off{ 10.0, -500.0 };
    const AD h(5000.0);
    const auto s  = US1976Atmosphere(h, off);
    const auto sd = US1976Atmosphere(5000.0, off);
    CHECK_THAT(CppAD::Value(s.T),   WithinRel(sd.T,   1e-14));
    CHECK_THAT(CppAD::Value(s.p),   WithinRel(sd.p,   1e-14));
    CHECK_THAT(CppAD::Value(s.rho), WithinRel(sd.rho, 1e-14));
}
