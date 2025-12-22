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

// Adjust include path to your tree:
#include <Aetherion/Aerodynamics/AerodynamicWrench.h>

using Catch::Approx;

namespace Aero = Aetherion::Aerodynamics;

TEST_CASE("AerodynamicWrenchBodyFromCoefficients: equals force + moment components", "[Aerodynamics][Wrench]")
{
    const Aero::Vec3<double> v_body{ 10.0, 0.0, 0.0 }; // alpha=0,beta=0
    const double rho = 1.0;
    const double S = 2.0;
    const double b = 3.0;
    const double cbar = 0.5;

    const double CL = 3.0;
    const double CD = 5.0;
    const double CY = 7.0;

    const double Cl = 0.1;
    const double Cm = -0.2;
    const double Cn = 0.3;

    const auto w = Aero::AerodynamicWrenchBodyFromCoefficients(
        v_body, rho, S, b, cbar, CL, CD, CY, Cl, Cm, Cn);

    const auto F = Aero::AerodynamicForceBodyFromCLCDCY(v_body, rho, S, CL, CD, CY);
    const auto M = Aero::AerodynamicMomentBodyFromClCmCn(v_body, rho, S, b, cbar, Cl, Cm, Cn);

    REQUIRE(w.F_body_N[0] == Approx(F[0]).margin(1e-12));
    REQUIRE(w.F_body_N[1] == Approx(F[1]).margin(1e-12));
    REQUIRE(w.F_body_N[2] == Approx(F[2]).margin(1e-12));

    REQUIRE(w.M_body_Nm[0] == Approx(M[0]).margin(1e-12));
    REQUIRE(w.M_body_Nm[1] == Approx(M[1]).margin(1e-12));
    REQUIRE(w.M_body_Nm[2] == Approx(M[2]).margin(1e-12));
}

TEST_CASE("AerodynamicWrenchBodyFromCoefficientsAndCP: moment equals r x F", "[Aerodynamics][Wrench]")
{
    const Aero::Vec3<double> v_body{ 10.0, 0.0, 0.0 }; // alpha=0,beta=0
    const double rho = 1.1;
    const double S = 1.7;

    const double CL = 1.2;
    const double CD = 0.4;
    const double CY = -0.3;

    const Aero::Vec3<double> r_cp_minus_cg{ 0.0, 0.0, 1.0 };

    const auto w = Aero::AerodynamicWrenchBodyFromCoefficientsAndCP(
        v_body, rho, S, CL, CD, CY, r_cp_minus_cg);

    const auto F = Aero::AerodynamicForceBodyFromCLCDCY(v_body, rho, S, CL, CD, CY);
    const auto M = Aero::AerodynamicMomentBodyFromCPForce(r_cp_minus_cg, F);

    REQUIRE(w.F_body_N[0] == Approx(F[0]).margin(1e-12));
    REQUIRE(w.F_body_N[1] == Approx(F[1]).margin(1e-12));
    REQUIRE(w.F_body_N[2] == Approx(F[2]).margin(1e-12));

    REQUIRE(w.M_body_Nm[0] == Approx(M[0]).margin(1e-12));
    REQUIRE(w.M_body_Nm[1] == Approx(M[1]).margin(1e-12));
    REQUIRE(w.M_body_Nm[2] == Approx(M[2]).margin(1e-12));
}

TEST_CASE("CppAD: Jacobian of AerodynamicWrenchBodyFromCoefficientsAndCP wrt CL,CD,CY matches linearity", "[Aerodynamics][Wrench][CppAD]")
{
    using AD = CppAD::AD<double>;

    // Keep alpha=beta=0 so force mapping is simple and analytic:
    // v_body along +x => F_body = [-qS*CD, +qS*CY, -qS*CL]
    // with q = 0.5*rho*V^2
    const Aero::Vec3<double> v_body_d{ 20.0, 0.0, 0.0 };
    const double rho = 1.3;
    const double S = 2.1;

    const double V = v_body_d[0];
    const double q = 0.5 * rho * V * V;

    const Aero::Vec3<double> r_cp_minus_cg_d{ 0.0, 0.0, 1.0 };

    // Independent vars: [CL, CD, CY]
    std::vector<AD> x(3);
    x[0] = 0.0; x[1] = 0.0; x[2] = 0.0;
    CppAD::Independent(x);

    const Aero::Vec3<AD> v_body{ AD(v_body_d[0]), AD(v_body_d[1]), AD(v_body_d[2]) };
    const Aero::Vec3<AD> r_cp_minus_cg{ AD(r_cp_minus_cg_d[0]), AD(r_cp_minus_cg_d[1]), AD(r_cp_minus_cg_d[2]) };

    const auto w = Aero::AerodynamicWrenchBodyFromCoefficientsAndCP(
        v_body, AD(rho), AD(S), x[0], x[1], x[2], r_cp_minus_cg);

    // Outputs: [Fx,Fy,Fz,Mx,My,Mz]
    std::vector<AD> Y(6);
    Y[0] = w.F_body_N[0];
    Y[1] = w.F_body_N[1];
    Y[2] = w.F_body_N[2];
    Y[3] = w.M_body_Nm[0];
    Y[4] = w.M_body_Nm[1];
    Y[5] = w.M_body_Nm[2];

    CppAD::ADFun<double> f(x, Y);

    const std::vector<double> xin{ 0.7, 0.2, -0.1 }; // CL,CD,CY
    const std::vector<double> jac = f.Jacobian(xin);
    REQUIRE(jac.size() == 6 * 3);

    // Force sensitivities (alpha=beta=0):
    // Fx = -qS*CD
    // Fy = +qS*CY
    // Fz = -qS*CL
    const double dFx_dCL = 0.0;
    const double dFx_dCD = -q * S;
    const double dFx_dCY = 0.0;

    const double dFy_dCL = 0.0;
    const double dFy_dCD = 0.0;
    const double dFy_dCY = +q * S;

    const double dFz_dCL = -q * S;
    const double dFz_dCD = 0.0;
    const double dFz_dCY = 0.0;

    // Moment M = r x F, with r=[0,0,1]:
    // Mx = -Fy
    // My = +Fx
    // Mz = 0
    const double dMx_dCL = -dFy_dCL;
    const double dMx_dCD = -dFy_dCD;
    const double dMx_dCY = -dFy_dCY;

    const double dMy_dCL = +dFx_dCL;
    const double dMy_dCD = +dFx_dCD;
    const double dMy_dCY = +dFx_dCY;

    const double dMz_dCL = 0.0;
    const double dMz_dCD = 0.0;
    const double dMz_dCY = 0.0;

    // Row-major: row i, col j => jac[i*3 + j]
    auto J = [&](int row, int col) { return jac[static_cast<size_t>(row) * 3u + static_cast<size_t>(col)]; };

    REQUIRE(J(0, 0) == Approx(dFx_dCL).margin(1e-9));
    REQUIRE(J(0, 1) == Approx(dFx_dCD).margin(1e-9));
    REQUIRE(J(0, 2) == Approx(dFx_dCY).margin(1e-9));

    REQUIRE(J(1, 0) == Approx(dFy_dCL).margin(1e-9));
    REQUIRE(J(1, 1) == Approx(dFy_dCD).margin(1e-9));
    REQUIRE(J(1, 2) == Approx(dFy_dCY).margin(1e-9));

    REQUIRE(J(2, 0) == Approx(dFz_dCL).margin(1e-9));
    REQUIRE(J(2, 1) == Approx(dFz_dCD).margin(1e-9));
    REQUIRE(J(2, 2) == Approx(dFz_dCY).margin(1e-9));

    REQUIRE(J(3, 0) == Approx(dMx_dCL).margin(1e-9));
    REQUIRE(J(3, 1) == Approx(dMx_dCD).margin(1e-9));
    REQUIRE(J(3, 2) == Approx(dMx_dCY).margin(1e-9));

    REQUIRE(J(4, 0) == Approx(dMy_dCL).margin(1e-9));
    REQUIRE(J(4, 1) == Approx(dMy_dCD).margin(1e-9));
    REQUIRE(J(4, 2) == Approx(dMy_dCY).margin(1e-9));

    REQUIRE(J(5, 0) == Approx(dMz_dCL).margin(1e-9));
    REQUIRE(J(5, 1) == Approx(dMz_dCD).margin(1e-9));
    REQUIRE(J(5, 2) == Approx(dMz_dCY).margin(1e-9));
}
