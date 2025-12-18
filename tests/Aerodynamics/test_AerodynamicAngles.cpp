// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <vector>

#include <cppad/cppad.hpp>

// Adjust include path to your project layout:
#include <Aetherion/Aerodynamics/AerodynamicAngles.h>

using Catch::Approx;

namespace Aero = Aetherion::Aerodynamics;

TEST_CASE("SpeedFromVelocity: 3-4-0 gives 5 (with tiny eps effect)", "[Aerodynamics][Angles]")
{
    const Aero::Vec3<double> v{ 3.0, 4.0, 0.0 };
    const double eps = 1e-12;

    const double speed = Aero::SpeedFromVelocity(v, eps);
    const double expected = std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + eps * eps);

    REQUIRE(speed == Approx(expected).margin(1e-15));
}

TEST_CASE("AnglesFromVelocityBody: pure +x velocity gives alpha=0, beta=0", "[Aerodynamics][Angles]")
{
    const Aero::Vec3<double> v{ 10.0, 0.0, 0.0 };

    const auto ang = Aero::AnglesFromVelocityBody(v);

    REQUIRE(ang.alpha_rad == Approx(0.0).margin(1e-15));
    REQUIRE(ang.beta_rad == Approx(0.0).margin(1e-15));
    REQUIRE(ang.speed_m_s == Approx(10.0).margin(1e-12));

    REQUIRE(ang.u == Approx(10.0).margin(1e-15));
    REQUIRE(ang.v == Approx(0.0).margin(1e-15));
    REQUIRE(ang.w == Approx(0.0).margin(1e-15));
}

TEST_CASE("AnglesFromVelocityBody: alpha = atan2(w,u)", "[Aerodynamics][Angles]")
{
    // u=1, w=1 -> alpha = 45 deg = pi/4
    const Aero::Vec3<double> v{ 1.0, 0.0, 1.0 };
    const auto ang = Aero::AnglesFromVelocityBody(v);

    REQUIRE(ang.alpha_rad == Approx(std::atan2(1.0, 1.0)).margin(1e-15));
    REQUIRE(ang.beta_rad == Approx(0.0).margin(1e-15));
}

TEST_CASE("AnglesFromVelocityBody: beta = atan2(v, sqrt(u^2+w^2))", "[Aerodynamics][Angles]")
{
    const double u = 3.0;
    const double vside = 4.0;
    const double w = 0.0;

    const Aero::Vec3<double> vb{ u, vside, w };
    const double eps = 1e-12;

    const auto ang = Aero::AnglesFromVelocityBody(vb, eps);

    const double uw = std::sqrt(u * u + w * w + eps * eps);
    const double expected_beta = std::atan2(vside, uw);

    REQUIRE(ang.beta_rad == Approx(expected_beta).margin(1e-15));
    REQUIRE(ang.alpha_rad == Approx(std::atan2(w, u)).margin(1e-15));
}

TEST_CASE("Cos/Sin helpers match std for doubles", "[Aerodynamics][Angles]")
{
    const double a = 0.3;
    const double b = -0.7;

    REQUIRE(Aero::CosAlpha(a) == Approx(std::cos(a)).margin(1e-15));
    REQUIRE(Aero::SinAlpha(a) == Approx(std::sin(a)).margin(1e-15));
    REQUIRE(Aero::CosBeta(b) == Approx(std::cos(b)).margin(1e-15));
    REQUIRE(Aero::SinBeta(b) == Approx(std::sin(b)).margin(1e-15));
}

TEST_CASE("CppAD: alpha and beta derivatives wrt velocity components (analytic checks)", "[Aerodynamics][Angles][CppAD]")
{
    using AD = CppAD::AD<double>;

    const double eps = 1e-12;

    // Independent variables: [u, v, w]
    std::vector<AD> x(3);
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;

    CppAD::Independent(x);

    Aero::Vec3<AD> vb{ x[0], x[1], x[2] };
    const auto ang = Aero::AnglesFromVelocityBody(vb, AD(eps));

    // Outputs: [alpha, beta]
    std::vector<AD> Y(2);
    Y[0] = ang.alpha_rad;
    Y[1] = ang.beta_rad;

    CppAD::ADFun<double> f(x, Y);

    // Test point (avoid singularities): u>0, v small, w nonzero
    const double u = 10.0;
    const double v = 2.0;
    const double w = 3.0;

    const std::vector<double> xin{ u, v, w };
    const std::vector<double> jac = f.Jacobian(xin);
    // jac layout: 2 rows x 3 cols (row-major): [dY0/du, dY0/dv, dY0/dw, dY1/du, dY1/dv, dY1/dw]
    REQUIRE(jac.size() == 6);

    // alpha = atan2(w,u)
    // d alpha / du = -w/(u^2 + w^2)
    // d alpha / dv = 0
    // d alpha / dw =  u/(u^2 + w^2)
    const double denom_a = (u * u + w * w);
    const double dalpha_du = -w / denom_a;
    const double dalpha_dv = 0.0;
    const double dalpha_dw = u / denom_a;

    REQUIRE(jac[0] == Approx(dalpha_du).margin(1e-12));
    REQUIRE(jac[1] == Approx(dalpha_dv).margin(1e-12));
    REQUIRE(jac[2] == Approx(dalpha_dw).margin(1e-12));

    // beta = atan2(v, s), s = sqrt(u^2 + w^2 + eps^2)
    // Let t = v/s. beta = atan(t).
    // d beta / dv =  s / (s^2 + v^2)
    // d beta / ds = -v / (s^2 + v^2)
    // ds/du = u/s, ds/dw = w/s
    const double s = std::sqrt(u * u + w * w + eps * eps);
    const double denom_b = (s * s + v * v);

    const double dbeta_dv = s / denom_b;
    const double dbeta_ds = -v / denom_b;

    const double ds_du = u / s;
    const double ds_dw = w / s;

    const double dbeta_du = dbeta_ds * ds_du;
    const double dbeta_dw = dbeta_ds * ds_dw;

    REQUIRE(jac[3] == Approx(dbeta_du).margin(1e-12));
    REQUIRE(jac[4] == Approx(dbeta_dv).margin(1e-12));
    REQUIRE(jac[5] == Approx(dbeta_dw).margin(1e-12));
}

TEST_CASE("CppAD: speed derivative equals v_i / speed (smooth)", "[Aerodynamics][Angles][CppAD]")
{
    using AD = CppAD::AD<double>;

    const double eps = 1e-12;

    std::vector<AD> x(3);
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;

    CppAD::Independent(x);

    Aero::Vec3<AD> vb{ x[0], x[1], x[2] };
    const AD speed = Aero::SpeedFromVelocity(vb, AD(eps));

    std::vector<AD> Y(1);
    Y[0] = speed;

    CppAD::ADFun<double> f(x, Y);

    const std::vector<double> xin{ 30.0, 40.0, 0.0 };
    const std::vector<double> jac = f.Jacobian(xin);

    const double u = xin[0], v = xin[1], w = xin[2];
    const double sp = std::sqrt(u * u + v * v + w * w + eps * eps);

    REQUIRE(jac.size() == 3);
    REQUIRE(jac[0] == Approx(u / sp).margin(1e-12));
    REQUIRE(jac[1] == Approx(v / sp).margin(1e-12));
    REQUIRE(jac[2] == Approx(w / sp).margin(1e-12));
}
