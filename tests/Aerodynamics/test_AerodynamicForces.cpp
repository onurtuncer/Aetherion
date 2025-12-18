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

#include <cmath>
#include <vector>

#include <cppad/cppad.hpp>

#include <Aetherion/Aerodynamics/AerodynamicForces.h>

using Catch::Approx;

namespace Aero = Aetherion::Aerodynamics;

static inline double Norm3(const Aero::Vec3<double>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

TEST_CASE("DynamicPressure: matches 0.5*rho*V^2", "[Aerodynamics][Force]")
{
    const double rho = 1.225;
    const double V = 100.0;

    const double q = Aero::DynamicPressure(rho, V);
    REQUIRE(q == Approx(0.5 * rho * V * V).margin(1e-12));
}

TEST_CASE("WindToBodyDCM: alpha=0,beta=0 is identity", "[Aerodynamics][Force]")
{
    const auto C = Aero::WindToBodyDCM(0.0, 0.0);

    REQUIRE(C[0][0] == Approx(1.0).margin(1e-15));
    REQUIRE(C[0][1] == Approx(0.0).margin(1e-15));
    REQUIRE(C[0][2] == Approx(0.0).margin(1e-15));

    REQUIRE(C[1][0] == Approx(0.0).margin(1e-15));
    REQUIRE(C[1][1] == Approx(1.0).margin(1e-15));
    REQUIRE(C[1][2] == Approx(0.0).margin(1e-15));

    REQUIRE(C[2][0] == Approx(0.0).margin(1e-15));
    REQUIRE(C[2][1] == Approx(0.0).margin(1e-15));
    REQUIRE(C[2][2] == Approx(1.0).margin(1e-15));
}

TEST_CASE("AerodynamicForceBodyFromCLCDCY_AlphaBeta: alpha=0,beta=0 maps wind forces directly", "[Aerodynamics][Force]")
{
    const double rho = 1.0;
    const double V = 10.0;
    const double S = 2.0;

    const double CL = 3.0;
    const double CD = 5.0;
    const double CY = 7.0;

    const double q = 0.5 * rho * V * V;

    const double D = q * S * CD;
    const double L = q * S * CL;
    const double Y = q * S * CY;

    // alpha=beta=0 => C_b_w = I => F_b == F_w = [-D, +Y, -L]
    const auto Fb = Aero::AerodynamicForceBodyFromCLCDCY_AlphaBeta(
        0.0, 0.0, rho, V, S, CL, CD, CY);

    REQUIRE(Fb[0] == Approx(-D).margin(1e-12));
    REQUIRE(Fb[1] == Approx(Y).margin(1e-12));
    REQUIRE(Fb[2] == Approx(-L).margin(1e-12));
}

TEST_CASE("AerodynamicForceBodyFromCLCDCY: alpha=0,beta=0 when v_body along +x", "[Aerodynamics][Force]")
{
    const Aero::Vec3<double> v_body{ 10.0, 0.0, 0.0 };

    const double rho = 1.0;
    const double S = 2.0;
    const double CL = 3.0;
    const double CD = 5.0;
    const double CY = 7.0;

    const double q = 0.5 * rho * 10.0 * 10.0;

    const double D = q * S * CD;
    const double L = q * S * CL;
    const double Y = q * S * CY;

    const auto Fb = Aero::AerodynamicForceBodyFromCLCDCY(
        v_body, rho, S, CL, CD, CY);

    REQUIRE(Fb[0] == Approx(-D).margin(1e-12));
    REQUIRE(Fb[1] == Approx(Y).margin(1e-12));
    REQUIRE(Fb[2] == Approx(-L).margin(1e-12));
}

TEST_CASE("AerodynamicForceBodyFromCLCDCY_AlphaBeta: pure drag only has magnitude q*S*CD (rotation preserves norm)", "[Aerodynamics][Force]")
{
    const double rho = 1.2;
    const double V = 50.0;
    const double S = 0.8;

    const double CL = 0.0;
    const double CY = 0.0;
    const double CD = 0.4;

    const double alpha = 0.3;
    const double beta = -0.2;

    const double q = 0.5 * rho * V * V;
    const double D = q * S * CD;

    const auto Fb = Aero::AerodynamicForceBodyFromCLCDCY_AlphaBeta(
        alpha, beta, rho, V, S, CL, CD, CY);

    REQUIRE(Norm3(Fb) == Approx(D).margin(1e-9));
}

TEST_CASE("AerodynamicForceBodyFromCxyz: matches q*S*[Cx,Cy,Cz]", "[Aerodynamics][Force]")
{
    const Aero::Vec3<double> v_body{ 3.0, 4.0, 0.0 }; // speed = 5
    const double rho = 2.0;
    const double S = 1.5;

    const double Cx = -0.1;
    const double Cy = 0.2;
    const double Cz = 0.3;

    const double V = 5.0;
    const double q = 0.5 * rho * V * V;

    const auto Fb = Aero::AerodynamicForceBodyFromCxyz(v_body, rho, S, Cx, Cy, Cz);

    REQUIRE(Fb[0] == Approx(q * S * Cx).margin(1e-12));
    REQUIRE(Fb[1] == Approx(q * S * Cy).margin(1e-12));
    REQUIRE(Fb[2] == Approx(q * S * Cz).margin(1e-12));
}

TEST_CASE("CppAD: AerodynamicForceBodyFromCxyz Jacobian wrt velocity matches analytic scaling", "[Aerodynamics][Force][CppAD]")
{
    using AD = CppAD::AD<double>;

    const double rho = 1.3;
    const double S = 2.2;

    const double Cx = 0.5;
    const double Cy = -0.25;
    const double Cz = 0.75;

    const double eps = 1e-12;

    // Independent: u,v,w
    std::vector<AD> x(3);
    x[0] = 0.0; x[1] = 0.0; x[2] = 0.0;
    CppAD::Independent(x);

    Aero::Vec3<AD> vb{ x[0], x[1], x[2] };

    const auto Fb = Aero::AerodynamicForceBodyFromCxyz(
        vb, AD(rho), AD(S), AD(Cx), AD(Cy), AD(Cz), AD(eps));

    std::vector<AD> Y(3);
    Y[0] = Fb[0]; Y[1] = Fb[1]; Y[2] = Fb[2];

    CppAD::ADFun<double> f(x, Y);

    const std::vector<double> xin{ 30.0, 40.0, 0.0 };
    const double u = xin[0], v = xin[1], w = xin[2];

    const std::vector<double> jac = f.Jacobian(xin);
    REQUIRE(jac.size() == 9);

    // Fb_i = q*S*C_i, q=0.5*rho*V^2, V = sqrt(u^2+v^2+w^2+eps^2)
    // dFb_i/du = S*C_i * 0.5*rho * d(V^2)/du = S*C_i * 0.5*rho * 2u = S*C_i*rho*u
    // similarly for v,w (since V^2 = u^2+v^2+w^2+eps^2)
    const auto dF_du = [&](double Ci) { return S * Ci * rho * u; };
    const auto dF_dv = [&](double Ci) { return S * Ci * rho * v; };
    const auto dF_dw = [&](double Ci) { return S * Ci * rho * w; };

    // Row 0 (Fb[0]) derivatives
    REQUIRE(jac[0] == Approx(dF_du(Cx)).margin(1e-8));
    REQUIRE(jac[1] == Approx(dF_dv(Cx)).margin(1e-8));
    REQUIRE(jac[2] == Approx(dF_dw(Cx)).margin(1e-8));

    // Row 1 (Fb[1]) derivatives
    REQUIRE(jac[3] == Approx(dF_du(Cy)).margin(1e-8));
    REQUIRE(jac[4] == Approx(dF_dv(Cy)).margin(1e-8));
    REQUIRE(jac[5] == Approx(dF_dw(Cy)).margin(1e-8));

    // Row 2 (Fb[2]) derivatives
    REQUIRE(jac[6] == Approx(dF_du(Cz)).margin(1e-8));
    REQUIRE(jac[7] == Approx(dF_dv(Cz)).margin(1e-8));
    REQUIRE(jac[8] == Approx(dF_dw(Cz)).margin(1e-8));
}
