// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cppad/cppad.hpp>
#include <vector>
#include <cmath>
#include <numbers>

#include <Aetherion/Environment/Gravity.h>

using Catch::Approx;
using Aetherion::Environment::CentralGravity;
using Aetherion::Environment::J2;
using Aetherion::Environment::Vec3;

// Helpers to extract doubles from scalars
inline static double ToDouble(double x) { return x; }
inline static double ToDouble(const CppAD::AD<double>& x) { return CppAD::Value(x); }

// Norm of a Vec3<double>
inline static double Norm(const Vec3<double>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

TEST_CASE("CentralGravity - magnitude at Earth's surface", "[environment][gravity]")
{
    const double mu = 3.986004418e14;
    const double Re = 6378137.0;

    Vec3<double> r{ Re, 0.0, 0.0 };
    auto g = CentralGravity(r, mu);

    const double g_mag = Norm(g);
    REQUIRE(g_mag == Approx(9.81).margin(0.1));
}

TEST_CASE("CentralGravity - direction is radial", "[environment][gravity]")
{
    const double mu = 3.986004418e14;
    const double Re = 6378137.0;

    // Position at 45° in equatorial plane
    const double angle = std::numbers::pi / 4.0;
    Vec3<double> r{
        Re * std::cos(angle),
        Re * std::sin(angle),
        0.0
    };
    auto g = CentralGravity(r, mu);

    // g should be opposite to r (up to scale)
    const double dot = r[0] * g[0] + r[1] * g[1] + r[2] * g[2];
    REQUIRE(dot < 0.0);
}

// ============================================================================
// 1. CentralGravity: surface magnitude & direction
// ============================================================================

TEST_CASE("CentralGravity - surface magnitude & direction", "[gravity][central]")
{
    const double mu = 3.986004418e14;  // [m^3/s^2]
    const double Re = 6378137.0;       // [m] WGS-84 equatorial radius

    Vec3<double> r{ Re, 0.0, 0.0 };
    auto g = CentralGravity(r, mu);

    const double g_mag = Norm(g);
    const double g_ref = mu / (Re * Re);

    // Magnitude should be mu / r^2
    REQUIRE(g_mag == Approx(g_ref).epsilon(1e-12));

    // Direction: should point exactly -x for this case
    REQUIRE(g[0] == Approx(-g_ref).epsilon(1e-12));
    REQUIRE(g[1] == Approx(0.0).margin(1e-14));
    REQUIRE(g[2] == Approx(0.0).margin(1e-14));
}

// ============================================================================
// 2. CentralGravity: scalar compatibility (double vs AD<double>)
// ============================================================================

TEMPLATE_TEST_CASE("CentralGravity - values for Scalar=double and AD<double>",
    "[gravity][central][template]",
    double, CppAD::AD<double>)
{
    using Scalar = TestType;

    const double mu = 3.986004418e14;

    // Generic off-axis position to avoid special symmetry
    Vec3<double> r_d{ 7000e3, -1000e3, 200e3 };

    // Reference (double) evaluation
    auto g_ref = CentralGravity(r_d, mu);

    // Scalar evaluation
    Vec3<Scalar> r_s{ Scalar(r_d[0]), Scalar(r_d[1]), Scalar(r_d[2]) };
    auto g_s = CentralGravity(r_s, Scalar(mu));

    for (int i = 0; i < 3; ++i) {
        REQUIRE(ToDouble(g_s[i]) == Approx(g_ref[i]).epsilon(1e-12));
    }
}

// ============================================================================
// 3. CentralGravity: CppAD Jacobian vs finite differences
// ============================================================================

static Vec3<double> CentralGravity_double(const Vec3<double>& r)
{
    return CentralGravity(r, 3.986004418e14);
}

// Finite-difference Jacobian: df/dr (3x3)
static std::array<std::array<double, 3>, 3>
central_gravity_fd_jacobian(const Vec3<double>& r0, double h)
{
    std::array<std::array<double, 3>, 3> J{};

    for (int j = 0; j < 3; ++j) {
        Vec3<double> r_plus = r0;
        Vec3<double> r_minus = r0;
        r_plus[j] += h;
        r_minus[j] -= h;

        auto g_plus = CentralGravity_double(r_plus);
        auto g_minus = CentralGravity_double(r_minus);

        for (int i = 0; i < 3; ++i) {
            J[i][j] = (g_plus[i] - g_minus[i]) / (2.0 * h);
        }
    }
    return J;
}

TEST_CASE("CentralGravity - CppAD Jacobian vs finite differences",
    "[gravity][central][CppAD][derivatives]")
{
    using ADScalar = CppAD::AD<double>;

    // Generic point away from the origin and poles
    Vec3<double> r0_d{ 7000e3, -1000e3, 200e3 };
    const double mu = 3.986004418e14;

    // Build AD tape: f : R^3 -> R^3
    std::vector<ADScalar> ax(3);
    for (int i = 0; i < 3; ++i) {
        ax[i] = ADScalar(r0_d[i]);
    }
    CppAD::Independent(ax);

    Vec3<ADScalar> r_ad{ ax[0], ax[1], ax[2] };
    auto g_ad = CentralGravity(r_ad, ADScalar(mu));

    std::vector<ADScalar> ay(3);
    ay[0] = g_ad[0];
    ay[1] = g_ad[1];
    ay[2] = g_ad[2];

    CppAD::ADFun<double> f(ax, ay);

    // AD Jacobian at r0
    std::vector<double> x0(3);
    for (int i = 0; i < 3; ++i) x0[i] = r0_d[i];

    std::vector<double> J_ad = f.Jacobian(x0);
    REQUIRE(J_ad.size() == 9);

    // FD Jacobian
    const double dh = 10.0; // 10 m
    auto J_fd = central_gravity_fd_jacobian(r0_d, dh);

    // Compare element-wise
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double Jij_ad = J_ad[i * 3 + j];
            const double Jij_fd = J_fd[i][j];
            REQUIRE(Jij_ad == Approx(Jij_fd).epsilon(1e-6));
        }
    }
}

// ============================================================================
// 4. J2: reduces to central gravity when J2 = 0
// ============================================================================

TEST_CASE("J2 - reduces to central gravity when J2 = 0", "[gravity][J2][central]")
{
    const double mu = 3.986004418e14;
    const double Re = 6378137.0;

    Vec3<double> r{ 7000e3, -200e3, 100e3 };

    auto g_central = CentralGravity(r, mu);
    auto g_j2_zero = J2(r, mu, Re, 0.0);

    for (int i = 0; i < 3; ++i) {
        REQUIRE(g_j2_zero[i] == Approx(g_central[i]).epsilon(1e-15));
    }
}

// ============================================================================
// 5. J2: perturbation magnitude sanity (small compared to central)
// ============================================================================

TEST_CASE("J2 - perturbation is small relative to central gravity",
    "[gravity][J2][sanity]")
{
    const double mu = 3.986004418e14;
    const double Re = 6378137.0;

    // 700 km altitude, off-equator position
    Vec3<double> r{ (Re + 700e3), 300e3, 500e3 };

    auto g_central = CentralGravity(r, mu);
    auto g_j2 = J2(r, mu, Re, 1.08262668e-3);

    Vec3<double> diff{
        g_j2[0] - g_central[0],
        g_j2[1] - g_central[1],
        g_j2[2] - g_central[2]
    };

    const double norm_central = Norm(g_central);
    const double norm_diff = Norm(diff);

    // J2 is typically a percent-level correction or less
    REQUIRE(norm_diff / norm_central < 1e-2);
}

// ============================================================================
// 6. J2: scalar compatibility (double vs AD<double>)
// ============================================================================

TEMPLATE_TEST_CASE("J2 - values for Scalar=double and AD<double>",
    "[gravity][J2][template]",
    double, CppAD::AD<double>)
{
    using Scalar = TestType;

    const double mu_d = 3.986004418e14;
    const double Re_d = 6378137.0;
    const double J2_d = 1.08262668e-3;

    Vec3<double> r_d{ (Re_d + 500e3), -400e3, 200e3 };

    // Reference (double)
    auto g_ref = J2(r_d, mu_d, Re_d, J2_d);

    // Scalar evaluation
    Vec3<Scalar> r_s{ Scalar(r_d[0]), Scalar(r_d[1]), Scalar(r_d[2]) };
    auto g_s = J2(r_s, Scalar(mu_d), Scalar(Re_d), Scalar(J2_d));

    for (int i = 0; i < 3; ++i) {
        REQUIRE(ToDouble(g_s[i]) == Approx(g_ref[i]).epsilon(1e-12));
    }
}

// ============================================================================
// 7. J2: CppAD Jacobian vs finite differences
// ============================================================================

static Vec3<double> J2_double(const Vec3<double>& r)
{
    return J2(r, 3.986004418e14, 6378137.0, 1.08262668e-3);
}

// Finite-difference Jacobian for J2: df/dr (3x3)
static std::array<std::array<double, 3>, 3>
j2_fd_jacobian(const Vec3<double>& r0, double h)
{
    std::array<std::array<double, 3>, 3> J{};

    for (int j = 0; j < 3; ++j) {
        Vec3<double> r_plus = r0;
        Vec3<double> r_minus = r0;
        r_plus[j] += h;
        r_minus[j] -= h;

        auto g_plus = J2_double(r_plus);
        auto g_minus = J2_double(r_minus);

        for (int i = 0; i < 3; ++i) {
            J[i][j] = (g_plus[i] - g_minus[i]) / (2.0 * h);
        }
    }
    return J;
}

TEST_CASE("J2 - CppAD Jacobian vs finite differences",
    "[gravity][J2][CppAD][derivatives]")
{
    using ADScalar = CppAD::AD<double>;

    Vec3<double> r0_d{ 6778e3, 500e3, 300e3 }; // ~400 km altitude
    const double mu = 3.986004418e14;
    const double Re = 6378137.0;
    const double J2_val = 1.08262668e-3;

    // Build AD tape: f : R^3 -> R^3
    std::vector<ADScalar> ax(3);
    for (int i = 0; i < 3; ++i) {
        ax[i] = ADScalar(r0_d[i]);
    }
    CppAD::Independent(ax);

    Vec3<ADScalar> r_ad{ ax[0], ax[1], ax[2] };
    auto g_ad = J2(r_ad, ADScalar(mu), ADScalar(Re), ADScalar(J2_val));

    std::vector<ADScalar> ay(3);
    ay[0] = g_ad[0];
    ay[1] = g_ad[1];
    ay[2] = g_ad[2];

    CppAD::ADFun<double> f(ax, ay);

    // AD Jacobian
    std::vector<double> x0(3);
    for (int i = 0; i < 3; ++i) x0[i] = r0_d[i];

    std::vector<double> J_ad = f.Jacobian(x0);
    REQUIRE(J_ad.size() == 9);

    // FD Jacobian
    const double dh = 10.0;
    auto J_fd = j2_fd_jacobian(r0_d, dh);

    // Compare element-wise (J2 field is smooth away from r=0)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double Jij_ad = J_ad[i * 3 + j];
            const double Jij_fd = J_fd[i][j];
            REQUIRE(Jij_ad == Approx(Jij_fd).epsilon(1e-5));
        }
    }
}
