// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cppad/cppad.hpp>

#include <Aetherion/Environment/Atmosphere.h>  

using Catch::Approx;
using namespace Aetherion;

TEST_CASE("US1976Atmosphere - sea level values (double)", "[US1976][values]")
{
    const double h = 0.0;
    const auto s = Environment::US1976Atmosphere(h);

    // Reference values from US1976 at sea level
    const double T_ref = 288.15;     // K
    const double p_ref = 101325.0;   // Pa
    const double rho_ref = 1.225;      // kg/m^3 (approx)
    const double a_ref = 340.29399;  // m/s (sqrt(1.4 * 287.05287 * 288.15))

    REQUIRE(s.T == Approx(T_ref).margin(1e-6));
    REQUIRE(s.p == Approx(p_ref).margin(0.5));       // 0.5 Pa tolerance
    REQUIRE(s.rho == Approx(rho_ref).epsilon(1e-3));   // ~0.1% relative
    REQUIRE(s.a == Approx(a_ref).epsilon(1e-5));
}

// Helper to compute finite-difference derivatives (central difference)
template <class Func>
auto central_difference(Func f, double x, double h)
{
    auto f_plus = f(x + h);
    auto f_minus = f(x - h);

    Environment::Us1976State<double> df{};
    df.T = (f_plus.T - f_minus.T) / (2.0 * h);
    df.p = (f_plus.p - f_minus.p) / (2.0 * h);
    df.rho = (f_plus.rho - f_minus.rho) / (2.0 * h);
    df.a = (f_plus.a - f_minus.a) / (2.0 * h);
    return df;
}

TEST_CASE("US1976Atmosphere - CppAD derivatives vs finite differences", "[US1976][CppAD][derivatives]")
{
    using ADScalar = CppAD::AD<double>;

    // Choose an altitude safely inside a single layer (avoid boundaries).
    // 5000 m is inside the 0–11 km gradient layer (Lb = -6.5 K/km).
    const double h0 = 5000.0;

    // ----- Build CppAD tape ---------------------------------------------------
    std::vector<ADScalar> ax(1);
    ax[0] = ADScalar(h0);

    CppAD::Independent(ax);

    auto s_ad = Environment::US1976Atmosphere(ax[0]);

    // y = [T, p, rho, a]
    std::vector<ADScalar> ay(4);
    ay[0] = s_ad.T;
    ay[1] = s_ad.p;
    ay[2] = s_ad.rho;
    ay[3] = s_ad.a;

    CppAD::ADFun<double> f(ax, ay);

    // ----- Evaluate Jacobian at h0 -------------------------------------------
    std::vector<double> x0(1);
    x0[0] = h0;

    // f: R -> R^4  => Jacobian is 4x1
    std::vector<double> J = f.Jacobian(x0);
    REQUIRE(J.size() == 4);

    const double dTdh_ad = J[0];
    const double dpdh_ad = J[1];
    const double drhodh_ad = J[2];
    const double dadh_ad = J[3];

    // ----- Finite-difference derivatives as reference ------------------------
    const double dh = 10.0;  // 10 m step, small enough but not too small

    auto fd = central_difference(
        [](double h) { return Environment::US1976Atmosphere(h); },
        h0,
        dh
    );

    const double dTdh_fd = fd.T;
    const double dpdh_fd = fd.p;
    const double drhodh_fd = fd.rho;
    const double dadh_fd = fd.a;

    // ----- Compare AD vs finite differences ----------------------------------
    // These are smooth in the interior of the layer; expect good agreement.
    REQUIRE(dTdh_ad == Approx(dTdh_fd).epsilon(1e-6));
    REQUIRE(dpdh_ad == Approx(dpdh_fd).epsilon(1e-5));
    REQUIRE(drhodh_ad == Approx(drhodh_fd).epsilon(1e-5));
    REQUIRE(dadh_ad == Approx(dadh_fd).epsilon(1e-6));
}

TEST_CASE("US1976Atmosphere - CppAD works in an isothermal layer", "[US1976][CppAD][isothermal]")
{
    using ADScalar = CppAD::AD<double>;

    // 15 km is inside the 11–20 km isothermal layer (Lb = 0).
    const double h0 = 15000.0;

    std::vector<ADScalar> ax(1);
    ax[0] = ADScalar(h0);
    CppAD::Independent(ax);

    auto s_ad = Environment::US1976Atmosphere(ax[0]);

    std::vector<ADScalar> ay(4);
    ay[0] = s_ad.T;
    ay[1] = s_ad.p;
    ay[2] = s_ad.rho;
    ay[3] = s_ad.a;

    CppAD::ADFun<double> f(ax, ay);

    std::vector<double> x0(1);
    x0[0] = h0;
    std::vector<double> J = f.Jacobian(x0);

    REQUIRE(J.size() == 4);

    const double dTdh_ad = J[0];
    const double dpdh_ad = J[1];
    const double drhodh_ad = J[2];
    const double dadh_ad = J[3];

    const double dh = 10.0;

    auto fd = central_difference(
        [](double h) { return Environment::US1976Atmosphere(h); },
        h0,
        dh
    );

    const double dTdh_fd = fd.T;
    const double dpdh_fd = fd.p;
    const double drhodh_fd = fd.rho;
    const double dadh_fd = fd.a;

    // In an isothermal layer, T is constant in H, so derivative T' should be ~0.
    REQUIRE(dTdh_ad == Approx(0.0).margin(1e-8));
    REQUIRE(dTdh_fd == Approx(0.0).margin(1e-6));

    // Other quantities should still have consistent derivatives.
    REQUIRE(dpdh_ad == Approx(dpdh_fd).epsilon(1e-5));
    REQUIRE(drhodh_ad == Approx(drhodh_fd).epsilon(1e-5));
    REQUIRE(dadh_ad == Approx(dadh_fd).epsilon(1e-6));
}

TEST_CASE("US1976Atmosphere - pressure/density monotonic and fields positive",
    "[US1976][sanity][monotonicity]")
{
    const double h_min = 0.0;
    const double h_max = 84000.0; // slightly below model top to avoid clamp artifacts
    const double dh = 1000.0;  // 1 km steps

    auto s_prev = Environment::US1976Atmosphere(h_min);

    REQUIRE(s_prev.p > 0.0);
    REQUIRE(s_prev.rho > 0.0);
    REQUIRE(s_prev.T > 0.0);
    REQUIRE(s_prev.a > 0.0);

    for (double h = h_min + dh; h <= h_max; h += dh) {
        auto s = Environment::US1976Atmosphere(h);

        // Positivity checks
        REQUIRE(s.p > 0.0);
        REQUIRE(s.rho > 0.0);
        REQUIRE(s.T > 0.0);
        REQUIRE(s.a > 0.0);

        // Pressure and density should strictly decrease with altitude in US1976
        CHECK(s.p < s_prev.p);
        CHECK(s.rho < s_prev.rho);

        s_prev = s;
    }
}
