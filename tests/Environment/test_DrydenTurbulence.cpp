// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DrydenTurbulence.cpp
//
// Statistics of the Dryden filter bank against the MIL-F-8785C spectra:
// variance, spectrum shape (including the sqrt(3) zero of the v/w channels),
// independence of the step size, determinism, and the low-altitude parameter
// rules.  The physical sign of the rotational gusts is tested against the
// F-16 model in tests/FlightDynamics/test_F16Environment.cpp.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Environment/DrydenTurbulence.h>

#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

using namespace Aetherion::Environment;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

    constexpr double kV  = 200.0;   // airspeed [m/s]
    constexpr double kL  = 533.4;   // scale length [m]
    constexpr double kPi = std::numbers::pi;

    DrydenParameters testParams(double su = 1.0, double sv = 2.0, double sw = 3.0)
    {
        DrydenParameters p{};
        p.sigma_u_mps = su; p.sigma_v_mps = sv; p.sigma_w_mps = sw;
        p.L_u_m = p.L_v_m = p.L_w_m = kL;
        p.wingspan_m = 9.144;
        return p;
    }

    struct Stats { double mean{}, var{}; };

    Stats meanVar(const std::vector<double>& x)
    {
        double m = 0.0;
        for (double v : x) m += v;
        m /= static_cast<double>(x.size());
        double s2 = 0.0;
        for (double v : x) s2 += (v - m) * (v - m);
        s2 /= static_cast<double>(x.size() - 1);
        return { m, s2 };
    }

    // MIL-F-8785C one-sided spectra, converted to temporal frequency omega [rad/s]
    // (Omega = omega / V): they integrate to sigma^2 over omega in [0, inf).
    double PhiU(double omega, double sigma) {
        const double x = kL * omega / kV;
        return sigma * sigma * (2.0 * kL / (kPi * kV)) / (1.0 + x * x);
    }
    double PhiW(double omega, double sigma) {
        const double x = kL * omega / kV;
        return sigma * sigma * (kL / (kPi * kV)) * (1.0 + 3.0 * x * x) / ((1.0 + x * x) * (1.0 + x * x));
    }

    // Averaged rectangular-window periodogram at the DFT bins k of segments of
    // length N:  Phi_hat(omega_k) = (dt / (pi N)) |X_k|^2, one-sided in omega.
    struct Periodogram {
        int N; double dt; std::vector<int> bins;
        std::vector<std::complex<double>> acc; std::vector<double> power; int nseg{ 0 }; int n{ 0 };
        Periodogram(int N_, double dt_, std::vector<int> bins_)
            : N(N_), dt(dt_), bins(std::move(bins_)), acc(bins.size()), power(bins.size(), 0.0) {}
        void push(double x) {
            for (std::size_t j = 0; j < bins.size(); ++j) {
                const double ang = -2.0 * kPi * bins[j] * n / static_cast<double>(N);
                acc[j] += x * std::complex<double>(std::cos(ang), std::sin(ang));
            }
            if (++n == N) {
                for (std::size_t j = 0; j < bins.size(); ++j) {
                    power[j] += std::norm(acc[j]);
                    acc[j] = 0.0;
                }
                n = 0; ++nseg;
            }
        }
        double omega(std::size_t j) const { return 2.0 * kPi * bins[j] / (N * dt); }
        double phi(std::size_t j)   const { return (dt / (kPi * N)) * power[j] / nseg; }
    };
}

TEST_CASE("DrydenTurbulence: zero intensity is calm", "[dryden]")
{
    DrydenTurbulence t(DrydenParameters{}, 7);
    for (int i = 0; i < 100; ++i) {
        const GustState g = t.step(0.01, kV);
        CHECK(g.u_mps == 0.0); CHECK(g.v_mps == 0.0); CHECK(g.w_mps == 0.0);
        CHECK(g.p_rad_s == 0.0); CHECK(g.q_rad_s == 0.0); CHECK(g.r_rad_s == 0.0);
    }
}

TEST_CASE("DrydenTurbulence: the same seed reproduces the same record", "[dryden]")
{
    DrydenTurbulence a(testParams(), 42), b(testParams(), 42), c(testParams(), 43);
    bool differsFromC = false;
    for (int i = 0; i < 2000; ++i) {
        const GustState ga = a.step(0.02, kV), gb = b.step(0.02, kV), gc = c.step(0.02, kV);
        REQUIRE(ga.u_mps == gb.u_mps); REQUIRE(ga.v_mps == gb.v_mps); REQUIRE(ga.w_mps == gb.w_mps);
        REQUIRE(ga.p_rad_s == gb.p_rad_s); REQUIRE(ga.q_rad_s == gb.q_rad_s); REQUIRE(ga.r_rad_s == gb.r_rad_s);
        differsFromC = differsFromC || (ga.u_mps != gc.u_mps);
    }
    CHECK(differsFromC);
}

TEST_CASE("DrydenTurbulence: variance of every linear channel is sigma^2, p matches Phi_p",
          "[dryden][statistics]")
{
    const auto prm = testParams(1.0, 2.0, 3.0);
    DrydenTurbulence t(prm, 1);
    const double dt = 0.02;
    const int    N  = 10'000'000;           // 2e5 s: ~3.7e4 correlation times of L/V

    std::vector<double> u, v, w, p;
    u.reserve(N); v.reserve(N); w.reserve(N); p.reserve(N);
    for (int i = 0; i < N; ++i) {
        const GustState g = t.step(dt, kV);
        u.push_back(g.u_mps); v.push_back(g.v_mps); w.push_back(g.w_mps); p.push_back(g.p_rad_s);
    }
    const Stats su = meanVar(u), sv = meanVar(v), sw = meanVar(w), sp = meanVar(p);

    // Standard error of a variance estimate here is under 1 %; 4 % is ~5 sigma.
    CHECK_THAT(su.var, WithinRel(1.0, 0.04));
    CHECK_THAT(sv.var, WithinRel(4.0, 0.04));
    CHECK_THAT(sw.var, WithinRel(9.0, 0.04));
    CHECK_THAT(su.mean, WithinAbs(0.0, 0.05));
    CHECK_THAT(sw.mean, WithinAbs(0.0, 0.10));

    // p: MIL-F-8785C Phi_p integrated over frequency.
    const double b = prm.wingspan_m;
    const double varP = 0.8 * 9.0 * kPi * kPi * std::cbrt(kPi * kL / (4.0 * b)) / (16.0 * b * kL);
    CHECK_THAT(sp.var, WithinRel(varP, 0.04));
}

TEST_CASE("DrydenTurbulence: spectrum matches MIL-F-8785C, including the sqrt(3) zero",
          "[dryden][statistics]")
{
    const auto prm = testParams(1.0, 2.0, 3.0);
    DrydenTurbulence t(prm, 3);
    const double dt = 0.02;
    const int    Nseg = 4096;                       // 81.9 s per segment, d_omega = 0.077 rad/s
    const std::vector<int> bins = { 2, 5, 13, 39 }; // 0.15, 0.38, 1.0, 3.0 rad/s (corner V/L = 0.375)
    Periodogram pu(Nseg, dt, bins), pw(Nseg, dt, bins);

    const int N = 2400 * Nseg;
    for (int i = 0; i < N; ++i) {
        const GustState g = t.step(dt, kV);
        pu.push(g.u_mps);
        pw.push(g.w_mps);
    }
    REQUIRE(pu.nseg == 2400);

    for (std::size_t j = 0; j < bins.size(); ++j) {
        const double om = pu.omega(j);
        INFO("omega = " << om << " rad/s");
        CHECK_THAT(pu.phi(j), WithinRel(PhiU(om, 1.0), 0.15));
        CHECK_THAT(pw.phi(j), WithinRel(PhiW(om, 3.0), 0.15));
    }

    // The w spectrum falls as 1/omega^2 where the u spectrum falls as 1/omega^2
    // too, but with the (1 + 3x^2) numerator it sits 3x higher in the tail
    // relative to its own low-frequency level.  Check the shape, not just the level.
    const double shapeU = pu.phi(3) / pu.phi(0);
    const double shapeW = pw.phi(3) / pw.phi(0);
    CHECK_THAT(shapeU, WithinRel(PhiU(pu.omega(3), 1.0) / PhiU(pu.omega(0), 1.0), 0.2));
    CHECK_THAT(shapeW, WithinRel(PhiW(pw.omega(3), 3.0) / PhiW(pw.omega(0), 3.0), 0.2));
    CHECK(shapeW > 2.0 * shapeU);
}

TEST_CASE("DrydenTurbulence: statistics do not depend on the step size", "[dryden][statistics]")
{
    const auto prm = testParams(1.0, 2.0, 3.0);
    const double T = 1.0e5;
    Stats s[2]; double vr[2];
    const double dts[2] = { 0.01, 0.04 };
    for (int k = 0; k < 2; ++k) {
        DrydenTurbulence t(prm, 11 + k);
        const int N = static_cast<int>(T / dts[k]);
        std::vector<double> w; w.reserve(N);
        std::vector<double> r; r.reserve(N);
        for (int i = 0; i < N; ++i) {
            const GustState g = t.step(dts[k], kV);
            w.push_back(g.w_mps);
            r.push_back(g.r_rad_s);
        }
        s[k]  = meanVar(w);
        vr[k] = meanVar(r).var;
    }
    CHECK_THAT(s[0].var, WithinRel(9.0, 0.05));
    CHECK_THAT(s[1].var, WithinRel(9.0, 0.05));
    CHECK_THAT(s[1].var, WithinRel(s[0].var, 0.06));
    CHECK_THAT(vr[1],    WithinRel(vr[0], 0.08));   // the lagged-derivative channel too
    CHECK(vr[0] > 0.0);
}

TEST_CASE("DrydenTurbulence: filter states round-trip through states()/setStates()", "[dryden]")
{
    DrydenTurbulence a(testParams(), 5);
    for (int i = 0; i < 500; ++i) a.step(0.02, kV);
    const auto x = a.states();

    DrydenTurbulence b(testParams(), 999);
    b.setStates(x);
    const GustState ga = a.current(kV), gb = b.current(kV);
    CHECK(ga.u_mps == gb.u_mps); CHECK(ga.v_mps == gb.v_mps); CHECK(ga.w_mps == gb.w_mps);
    CHECK(ga.p_rad_s == gb.p_rad_s); CHECK(ga.q_rad_s == gb.q_rad_s); CHECK(ga.r_rad_s == gb.r_rad_s);
}

TEST_CASE("DrydenLowAltitudeParameters: MIL-F-8785C low-altitude rules", "[dryden]")
{
    // h = 100 ft, W20 = 15 kt (light): sigma_w = 0.1 W20, denominator 0.177 + 0.0823.
    const double W20 = 15.0 * 0.514444;
    const auto p = DrydenLowAltitudeParameters(100.0 * 0.3048, W20);
    const double denom = 0.177 + 0.000823 * 100.0;
    CHECK_THAT(p.sigma_w_mps, WithinRel(0.1 * W20, 1e-12));
    CHECK_THAT(p.sigma_u_mps, WithinRel(0.1 * W20 / std::pow(denom, 0.4), 1e-12));
    CHECK(p.sigma_v_mps == p.sigma_u_mps);
    CHECK_THAT(p.L_w_m, WithinRel(100.0 * 0.3048, 1e-12));
    CHECK_THAT(p.L_u_m, WithinRel(100.0 / std::pow(denom, 1.2) * 0.3048, 1e-12));
    CHECK(p.L_v_m == p.L_u_m);
    // Height is clamped to the rule's range.
    CHECK(DrydenLowAltitudeParameters(5000.0, W20).L_w_m == DrydenLowAltitudeParameters(304.8, W20).L_w_m);
}
