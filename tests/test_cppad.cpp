// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_all.hpp>
#include <cppad/cppad.hpp>
#include <cmath>

TEST_CASE("CppAD Jacobian test", "[cppad]") {

    // Independent variables
    CPPAD_TESTVECTOR(double) x0(2);
    x0[0] = 1.5;
    x0[1] = 2.0;

    // Wrap them as independent variables
    CppAD::ADFun<double> f;
    {
        CPPAD_TESTVECTOR(CppAD::AD<double>) ax(2);
        ax[0] = x0[0];
        ax[1] = x0[1];

        CppAD::Independent(ax);

        // Define y = f(x)
        CPPAD_TESTVECTOR(CppAD::AD<double>) ay(2);
        ay[0] = ax[0] * ax[0] + ax[1];
        ay[1] = CppAD::sin(ax[0]);

        f.Dependent(ax, ay);
    }

    // Evaluate Jacobian at x0
    CPPAD_TESTVECTOR(double) jac = f.Jacobian(x0);

    // Jacobian matrix is:
    // [ 2*x1    1  ]
    // [ cos(x1) 0  ]
    REQUIRE(jac.size() == 4);

    double j11 = 2.0 * x0[0];
    double j12 = 1.0;
    double j21 = std::cos(x0[0]);
    double j22 = 0.0;

    REQUIRE(jac[0] == Approx(j11)); // row 0 col 0
    REQUIRE(jac[1] == Approx(j12)); // row 0 col 1
    REQUIRE(jac[2] == Approx(j21)); // row 1 col 0
    REQUIRE(jac[3] == Approx(j22)); // row 1 col 1
}
