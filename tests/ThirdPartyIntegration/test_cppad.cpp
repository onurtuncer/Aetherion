// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <vector>
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <cppad/cppad.hpp>

TEST_CASE("CppAD: derivative of x^2 at x = 3", "[cppad]") {
    using CppAD::AD;

    // Independent variable vector: x (size 1)
    std::vector<AD<double>> x(1);
    x[0] = 3.0;

    // Start recording
    CppAD::Independent(x);

    // Dependent variable vector: y = x^2
    std::vector<AD<double>> y(1);
    y[0] = x[0] * x[0];

    // Create function f : x -> y
    CppAD::ADFun<double> f(x, y);

    // Point where we evaluate derivative
    std::vector<double> x0(1);
    x0[0] = 3.0;
    f.Forward(0, x0); // set the zero-order value

    // First-order forward mode: dx = 1 => dy = f'(x0)
    std::vector<double> dx(1);
    dx[0] = 1.0;

    auto dy = f.Forward(1, dx);

    REQUIRE(dy.size() == 1);

    const double expected = 6.0;        // d/dx (x^2) at x=3
    const double tol = 1e-10;
    REQUIRE(std::fabs(dy[0] - expected) < tol);
}


