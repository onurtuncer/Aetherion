// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Catch2 tests for: Aetherion/ODE/RKMK/Core/Newton.h

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Newton.h>

namespace {
    using Catch::Approx;
    namespace Core = Aetherion::ODE::RKMK::Core;

    inline bool finite_vec(const Eigen::VectorXd& v) {
        for (int i = 0; i < v.size(); ++i) {
            if (!std::isfinite(v(i))) return false;
        }
        return true;
    }

    struct SqrtSystemFunctor {
        template<class S>
        Eigen::Matrix<S, Eigen::Dynamic, 1>
            operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const {
            Eigen::Matrix<S, Eigen::Dynamic, 1> r(2);
            r(0) = x(0) * x(0) - S(2);
            r(1) = x(1) * x(1) - S(3);
            return r;
        }
    };
} // namespace

TEST_CASE("NewtonSolve: solves scalar x^2 - 2 = 0", "[newton]") {
    Eigen::VectorXd x(1);
    x(0) = 1.0;

    auto eval = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
        r.resize(1);
        J.resize(1, 1);
        r(0) = xin(0) * xin(0) - 2.0;
        J(0, 0) = 2.0 * xin(0);
        };

    Core::NewtonOptions opt;
    opt.max_iters = 30;
    opt.abs_tol = 1e-14;
    opt.rel_tol = 1e-14;
    opt.step_tol = 1e-14;
    opt.use_damping = true;

    const auto res = Core::NewtonSolve(eval, x, opt);

    REQUIRE(res.converged);
    REQUIRE(std::isfinite(x(0)));
    REQUIRE(x(0) == Approx(std::sqrt(2.0)).margin(1e-12));
    REQUIRE(res.r_norm == Approx(0.0).margin(1e-12));
}

TEST_CASE("NewtonSolve: solves a linear 2x2 system in one Newton step", "[newton]") {
    Eigen::Matrix2d A;
    A << 3.0, 1.0,
        1.0, 2.0;
    Eigen::Vector2d b;
    b << 1.0, 0.0;

    Eigen::VectorXd x(2);
    x << 0.0, 0.0;

    auto eval = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
        r = A * xin - b;
        J = A;
        };

    Core::NewtonOptions opt;
    opt.max_iters = 5;
    opt.abs_tol = 1e-14;
    opt.rel_tol = 0.0;
    opt.use_damping = false;

    const auto res = Core::NewtonSolve(eval, x, opt);

    const Eigen::Vector2d x_ref = A.fullPivLu().solve(b);

    REQUIRE(res.converged);
    REQUIRE((x - x_ref).norm() == Approx(0.0).margin(1e-13));
}

TEST_CASE("NewtonSolveRJ: wrapper works (same linear system)", "[newton]") {
    Eigen::Matrix2d A;
    A << 4.0, -1.0,
        -2.0, 3.0;

    Eigen::Vector2d b;
    b << 2.0, -1.0;

    Eigen::VectorXd x(2);
    x << 10.0, -10.0;

    auto residual = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r) { r = A * xin - b; };
    auto jacobian = [&](const Eigen::VectorXd&, Eigen::MatrixXd& J) { J = A; };

    Core::NewtonOptions opt;
    opt.max_iters = 10;
    opt.abs_tol = 1e-14;
    opt.rel_tol = 0.0;
    opt.use_damping = true;

    const auto res = Core::NewtonSolveRJ(residual, jacobian, x, opt);

    const Eigen::Vector2d x_ref = A.fullPivLu().solve(b);

    REQUIRE(res.converged);
    REQUIRE((x - x_ref).norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("NewtonSolve: line-search fails when residual cannot decrease", "[newton]") {
    // r(x) = 1 (constant), J(x) = 1 -> ||r|| never decreases for any step.
    Eigen::VectorXd x(1);
    x(0) = 0.0;

    auto eval = [&](const Eigen::VectorXd&, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
        r.resize(1);
        J.resize(1, 1);
        r(0) = 1.0;
        J(0, 0) = 1.0;
        };

    Core::NewtonOptions opt;
    opt.max_iters = 10;
    opt.use_damping = true;
    opt.sufficient_decrease = 0.9;
    opt.damping_init = 1.0;
    opt.damping_shrink = 0.5;

    // CRITICAL: make damping_min high so the line-search aborts immediately
    // after the first shrink (1.0 -> 0.5 < 0.9), triggering early failure.
    opt.damping_min = 0.9;
    opt.max_line_search_iters = 10;
    opt.throw_on_fail = false;

    const auto res = Core::NewtonSolve(eval, x, opt);

    REQUIRE_FALSE(res.converged);
    REQUIRE(res.iters == 1); // now matches the implementation’s early-return path
}


TEST_CASE("NewtonSolve: reaches max_iters without throwing when throw_on_fail=false", "[newton]") {
    // No real root: x^2 + 1 = 0
    Eigen::VectorXd x(1);
    x(0) = 0.1;

    auto eval = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
        r.resize(1);
        J.resize(1, 1);
        r(0) = xin(0) * xin(0) + 1.0;
        J(0, 0) = 2.0 * xin(0);
        };

    Core::NewtonOptions opt;
    opt.max_iters = 3;
    opt.use_damping = false;
    opt.throw_on_fail = false;

    const auto res = Core::NewtonSolve(eval, x, opt);

    REQUIRE_FALSE(res.converged);
    REQUIRE(res.iters == opt.max_iters);
    REQUIRE(std::isfinite(res.r_norm));
}

TEST_CASE("NewtonSolve: throws on max_iters when throw_on_fail=true", "[newton]") {
    Eigen::VectorXd x(1);
    x(0) = 0.1;

    auto eval = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
        r.resize(1);
        J.resize(1, 1);
        r(0) = xin(0) * xin(0) + 1.0;
        J(0, 0) = 2.0 * xin(0);
        };

    Core::NewtonOptions opt;
    opt.max_iters = 2;
    opt.use_damping = false;
    opt.throw_on_fail = true;

    REQUIRE_THROWS_AS(Core::NewtonSolve(eval, x, opt), std::runtime_error);
}

TEST_CASE("CppADResidualJacobian: NewtonSolve converges using AD Jacobian", "[newton][cppad]") {
    Core::CppADResidualJacobian eval(SqrtSystemFunctor{}, /*n=*/2);

    Eigen::VectorXd x(2);
    x << 1.4, 1.8;

    Core::NewtonOptions opt;
    opt.max_iters = 30;
    opt.abs_tol = 1e-14;
    opt.rel_tol = 1e-14;
    opt.step_tol = 1e-14;
    opt.use_damping = true;
    opt.throw_on_fail = false;

    const auto res = Core::NewtonSolve(eval, x, opt);

    REQUIRE(res.converged);
    REQUIRE(finite_vec(x));
    REQUIRE(x(0) == Approx(std::sqrt(2.0)).margin(1e-12));
    REQUIRE(x(1) == Approx(std::sqrt(3.0)).margin(1e-12));
}
