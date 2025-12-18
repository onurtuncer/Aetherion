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

#include <Aetherion/Aerodynamics/AerodynamicMoments.h>

using Catch::Approx;

namespace Aero = Aetherion::Aerodynamics;

TEST_CASE("Cross: right-hand rule basic check", "[Aerodynamics][Moment]")
{
    const Aero::Vec3<double> ex{ 1.0, 0.0, 0.0 };
    const Aero::Vec3<double> ey{ 0.0, 1.0, 0.0 };

    const auto ez = Aero::Cross(ex, ey);

    REQUIRE(ez[0] == Approx(0.0).margin(1e-15));
    REQUIRE(ez[1] == Approx(0.0).margin(1e-15));
    REQUIRE(ez[2] == Approx(1.0).margin(1e-15));
}

TEST_CASE("DynamicPressure: matches 0.5*rho*V^2", "[Aerodynamics][Moment]")
{
    const double rho = 1.225;
    const double V = 100.0;

    const double q = Aero::DynamicPressure(rho, V);
    REQUIRE(q == Approx(0.5 * rho * V * V).margin(1e-12));
}

TEST_CASE("AerodynamicMomentBodyFromClCmCn: matches q*S*[b*Cl, cbar*Cm, b*Cn]", "[Aerodynamics][Moment]")
{
    const Aero::Vec3<double> v_body{ 3.0, 4.0, 0.0 }; // speed ~ 5
    const double eps = 1e-12;

    const double rho = 1.2;
    const double S = 0.8;
    const double b = 2.0;
    const double cbar = 0.5;

    const double Cl = 0.10;
    const double Cm = -0.20;
    const double Cn = 0.30;

    const double V = std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + eps * eps);
    const double q = 0.5 * rho * V * V;

    const auto M = Aero::AerodynamicMomentBodyFromClCmCn(v_body, rho, S, b, cbar, Cl, Cm, Cn, eps);

    REQUIRE(M[0] == Approx(q * S * b * Cl).margin(1e-12)); // L
    REQUIRE(M[1] == Approx(q * S * cbar * Cm).margin(1e-12)); // M
    REQUIRE(M[2] == Approx(q * S * b * Cn).margin(1e-12)); // N
}

TEST_CASE("AerodynamicMomentBodyFromClCmCn_Speed: agrees with velocity-based version", "[Aerodynamics][Moment]")
{
    const Aero::Vec3<double> v_body{ 10.0, 2.0, -1.0 };
    const double eps = 1e-12;

    const double rho = 1.1;
    const double S = 1.3;
    const double b = 1.9;
    const double cbar = 0.6;

    const double Cl = -0.05;
    const double Cm = 0.07;
    const double Cn = 0.02;

    const auto M1 = Aero::AerodynamicMomentBodyFromClCmCn(v_body, rho, S, b, cbar, Cl, Cm, Cn, eps);

    const double V = std::sqrt(v_body[0] * v_body[0] + v_body[1] * v_body[1] + v_body[2] * v_body[2] + eps * eps);
    const auto M2 = Aero::AerodynamicMomentBodyFromClCmCn_Speed(V, rho, S, b, cbar, Cl, Cm, Cn);

    REQUIRE(M1[0] == Approx(M2[0]).margin(1e-12));
    REQUIRE(M1[1] == Approx(M2[1]).margin(1e-12));
    REQUIRE(M1[2] == Approx(M2[2]).margin(1e-12));
}

TEST_CASE("AerodynamicMomentBodyFromCPForce: M = r x F", "[Aerodynamics][Moment]")
{
    // r = [0, 0, 1] m, F = [10, 0, 0] N => M = [0, 10, 0] N*m
    const Aero::Vec3<double> r{ 0.0, 0.0, 1.0 };
    const Aero::Vec3<double> F{ 10.0, 0.0, 0.0 };

    const auto M = Aero::AerodynamicMomentBodyFromCPForce(r, F);

    REQUIRE(M[0] == Approx(0.0).margin(1e-15));
    REQUIRE(M[1] == Approx(10.0).margin(1e-15));
    REQUIRE(M[2] == Approx(0.0).margin(1e-15));
}

TEST_CASE("CppAD: Jacobian of AerodynamicMomentBodyFromClCmCn_Speed wrt speed", "[Aerodynamics][Moment][CppAD]")
{
    using AD = CppAD::AD<double>;

    const double rho = 1.2;
    const double S = 0.9;
    const double b = 2.2;
    const double cbar = 0.7;

    const double Cl = 0.1;
    const double Cm = -0.2;
    const double Cn = 0.05;

    // Independent variable: speed
    std::vector<AD> x(1);
    x[0] = 0.0;
    CppAD::Independent(x);

    const AD V = x[0];

    const auto M = Aero::AerodynamicMomentBodyFromClCmCn_Speed(
        V, AD(rho), AD(S), AD(b), AD(cbar), AD(Cl), AD(Cm), AD(Cn));

    std::vector<AD> Y(3);
    Y[0] = M[0];
    Y[1] = M[1];
    Y[2] = M[2];

    CppAD::ADFun<double> f(x, Y);

    const double V0 = 123.0;
    const std::vector<double> jac = f.Jacobian(std::vector<double>{V0});

    // M_i = q*S*ref*Ci, q = 0.5*rho*V^2
    // dM_i/dV = (0.5*rho*2V)*S*ref*Ci = rho*V*S*ref*Ci
    const double dL = rho * V0 * S * b * Cl;
    const double dM = rho * V0 * S * cbar * Cm;
    const double dN = rho * V0 * S * b * Cn;

    REQUIRE(jac.size() == 3);
    REQUIRE(jac[0] == Approx(dL).margin(1e-9));
    REQUIRE(jac[1] == Approx(dM).margin(1e-9));
    REQUIRE(jac[2] == Approx(dN).margin(1e-9));
}

TEST_CASE("CppAD: Jacobian of AerodynamicMomentBodyFromCPForce wrt force components", "[Aerodynamics][Moment][CppAD]")
{
    using AD = CppAD::AD<double>;

    // r fixed, F variable
    const Aero::Vec3<double> r_d{ 1.0, 2.0, 3.0 };
    const double rx = r_d[0], ry = r_d[1], rz = r_d[2];

    std::vector<AD> x(3);
    x[0] = 0.0; x[1] = 0.0; x[2] = 0.0;
    CppAD::Independent(x);

    const Aero::Vec3<AD> r{ AD(rx), AD(ry), AD(rz) };
    const Aero::Vec3<AD> F{ x[0], x[1], x[2] };

    const auto M = Aero::AerodynamicMomentBodyFromCPForce(r, F);

    std::vector<AD> Y(3);
    Y[0] = M[0];
    Y[1] = M[1];
    Y[2] = M[2];

    CppAD::ADFun<double> f(x, Y);

    const std::vector<double> F0{ 10.0, 20.0, 30.0 };
    const std::vector<double> jac = f.Jacobian(F0);
    REQUIRE(jac.size() == 9);

    // M = r x F
    // Mx = ry*Fz - rz*Fy
    // My = rz*Fx - rx*Fz
    // Mz = rx*Fy - ry*Fx
    //
    // Jacobian wrt [Fx,Fy,Fz]:
    // dMx = [ 0, -rz,  ry ]
    // dMy = [ rz,  0, -rx ]
    // dMz = [ -ry, rx,  0 ]
    REQUIRE(jac[0] == Approx(0.0).margin(1e-12));
    REQUIRE(jac[1] == Approx(-rz).margin(1e-12));
    REQUIRE(jac[2] == Approx(ry).margin(1e-12));

    REQUIRE(jac[3] == Approx(rz).margin(1e-12));
    REQUIRE(jac[4] == Approx(0.0).margin(1e-12));
    REQUIRE(jac[5] == Approx(-rx).margin(1e-12));

    REQUIRE(jac[6] == Approx(-ry).margin(1e-12));
    REQUIRE(jac[7] == Approx(rx).margin(1e-12));
    REQUIRE(jac[8] == Approx(0.0).margin(1e-12));
}
