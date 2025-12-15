// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/Newton.h
//
// Damped Newton solver for implicit RK stage systems.
//
// Design goals:
//  - Works with Eigen vectors/matrices (double).
//  - AD-friendly: optional CppAD-based evaluator that provides residual + Jacobian
//    from a templated residual functor (R^n -> R^n).
//  - Value semantics: all helpers are small, copyable, and header-only.
//
// Notes:
//  - Convergence decisions are performed in double (NewtonOptions uses double).
//  - For CppAD: we "tape once", then reuse Forward/Jacobian each iteration.
//
#pragma once

#include <utility>
#include <stdexcept>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::ODE::RKMK::Core {

    // ----------------------------------------------------------------------------
    // Result
    // ----------------------------------------------------------------------------

    struct NewtonResult final {
        bool   converged = false;
        int    iters = 0;
        int    line_search_iters = 0;

        double r0_norm = 0.0;
        double r_norm = 0.0;
        double dx_norm = 0.0;

        double last_lambda = 1.0;
    };

    // ----------------------------------------------------------------------------
    // Concepts-lite (keep header independent from Concepts.h to avoid include cycles)
    // ----------------------------------------------------------------------------

    template<class Eval>
    concept ResidualJacobianEvaluator =
        requires(Eval eval,
    const Eigen::VectorXd & x,
        Eigen::VectorXd & r,
        Eigen::MatrixXd & J) {
            { eval(x, r, J) } -> std::same_as<void>;
    };

    // ----------------------------------------------------------------------------
    // Utilities
    // ----------------------------------------------------------------------------

    [[nodiscard]] inline double norm2(const Eigen::VectorXd& v) noexcept {
        return v.norm();
    }

    // ----------------------------------------------------------------------------
    // Newton: uses an evaluator that provides residual r(x) and Jacobian J(x)
    // ----------------------------------------------------------------------------

    template<ResidualJacobianEvaluator Eval>
    inline NewtonResult NewtonSolve(Eval&& eval,
        Eigen::VectorXd& x,
        const NewtonOptions& opt = {}) {
        NewtonResult res{};

        const int n = static_cast<int>(x.size());
        if (n <= 0) {
            if (opt.throw_on_fail) throw std::invalid_argument("NewtonSolve: x.size() must be > 0");
            return res;
        }

        Eigen::VectorXd r(n);
        Eigen::MatrixXd J(n, n);

        // initial residual
        eval(x, r, J);
        res.r0_norm = norm2(r);
        res.r_norm = res.r0_norm;

        // early exit
        if (newton_converged(res.r_norm, res.r0_norm, 0.0, norm2(x), opt)) {
            res.converged = true;
            res.iters = 0;
            return res;
        }

        for (int k = 0; k < opt.max_iters; ++k) {
            // Recompute at current x
            eval(x, r, J);
            const double r_norm = norm2(r);

            // Solve J dx = -r
            // Use a robust QR (works for rank-deficient-ish cases better than LDLT).
            Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-r);

            const double dx_norm = norm2(dx);
            const double x_norm = norm2(x);

            res.iters = k + 1;
            res.r_norm = r_norm;
            res.dx_norm = dx_norm;

            if (newton_converged(r_norm, res.r0_norm, dx_norm, x_norm, opt)) {
                res.converged = true;
                res.last_lambda = 1.0;
                return res;
            }

            // Damping / line search
            double lambda = opt.use_damping ? opt.damping_init : 1.0;

            if (!opt.use_damping) {
                x.noalias() += dx;
                res.last_lambda = 1.0;
                continue;
            }

            // Try to ensure residual decreases
            bool accepted = false;
            Eigen::VectorXd x_trial(n);
            Eigen::VectorXd r_trial(n);
            Eigen::MatrixXd J_trial(n, n); // not strictly needed, but evaluator fills it

            for (int ls = 0; ls < opt.max_line_search_iters; ++ls) {
                x_trial = x + lambda * dx;

                eval(x_trial, r_trial, J_trial);
                const double r_new = norm2(r_trial);

                if (r_new <= opt.sufficient_decrease * r_norm) {
                    x = x_trial;
                    accepted = true;
                    res.line_search_iters += (ls + 1);
                    res.last_lambda = lambda;
                    break;
                }

                lambda *= opt.damping_shrink;
                if (lambda < opt.damping_min) break;
            }

            if (!accepted) {
                // Fall back: take the smallest step we tried (or no step if lambda underflowed)
                if (lambda >= opt.damping_min) {
                    x.noalias() += lambda * dx;
                    res.last_lambda = lambda;
                }
                else {
                    // Can't find a decent step; declare failure
                    res.converged = false;
                    if (opt.throw_on_fail) {
                        throw std::runtime_error("NewtonSolve: line search failed to find decreasing step");
                    }
                    return res;
                }
            }
        }

        // max iters reached
        res.converged = false;
        if (opt.throw_on_fail) {
            throw std::runtime_error("NewtonSolve: max_iters reached without convergence");
        }
        return res;
    }

    // ----------------------------------------------------------------------------
    // Convenience wrapper: residual-only + analytic Jacobian callback
    // ----------------------------------------------------------------------------

    template<class Residual, class Jacobian>
    inline NewtonResult NewtonSolveRJ(Residual&& residual,
        Jacobian&& jacobian,
        Eigen::VectorXd& x,
        const NewtonOptions& opt = {}) {
        auto eval = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
            residual(xin, r);
            jacobian(xin, J);
            };
        return NewtonSolve(eval, x, opt);
    }

    // ----------------------------------------------------------------------------
    // CppAD evaluator: tape once, then provide r(x) and J(x)
    // ----------------------------------------------------------------------------
    //
    // Requirements on Functor F:
    //   template<class S>
    //   Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const;
    //
    // and it must return a vector of the same size as x (square system).
    //
    template<class Functor>
    class CppADResidualJacobian final {
    public:
        explicit CppADResidualJacobian(Functor f, int n)
            : functor_(std::move(f)), n_(n) {
            if (n_ <= 0) throw std::invalid_argument("CppADResidualJacobian: n must be > 0");
            tape_ = build_tape_();
        }

        void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
            if (x.size() != n_) throw std::invalid_argument("CppADResidualJacobian: x has wrong size");

            // Forward(0) returns y = f(x)
            const auto y = tape_.Forward(0, to_cppad_(x));
            r = from_cppad_vec_(y);

            // Jacobian: row-major vector length n*n
            const auto jac = tape_.Jacobian(to_cppad_(x));
            J = from_cppad_jac_(jac);
        }

        [[nodiscard]] int size() const noexcept { return n_; }

    private:
        Functor functor_;
        int n_{ 0 };
        CppAD::ADFun<double> tape_;

        static CppAD::vector<double> to_cppad_(const Eigen::VectorXd& x) {
            CppAD::vector<double> v(static_cast<size_t>(x.size()));
            for (int i = 0; i < x.size(); ++i) v[static_cast<size_t>(i)] = x(i);
            return v;
        }

        Eigen::VectorXd from_cppad_vec_(const CppAD::vector<double>& v) const {
            Eigen::VectorXd out(n_);
            for (int i = 0; i < n_; ++i) out(i) = v[static_cast<size_t>(i)];
            return out;
        }

        Eigen::MatrixXd from_cppad_jac_(const CppAD::vector<double>& jac) const {
            Eigen::MatrixXd J(n_, n_);
            // CppAD Jacobian is in row-major order by default: dy_i/dx_j stored at i*n + j
            for (int i = 0; i < n_; ++i) {
                for (int j = 0; j < n_; ++j) {
                    J(i, j) = jac[static_cast<size_t>(i * n_ + j)];
                }
            }
            return J;
        }

        CppAD::ADFun<double> build_tape_() const {
            using AD = CppAD::AD<double>;

            CppAD::vector<AD> ax(static_cast<size_t>(n_));
            for (int i = 0; i < n_; ++i) ax[static_cast<size_t>(i)] = AD(0.0);

            CppAD::Independent(ax);

            Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
            for (int i = 0; i < n_; ++i) x_ad(i) = ax[static_cast<size_t>(i)];

            Eigen::Matrix<AD, Eigen::Dynamic, 1> y_ad = functor_(x_ad);

            if (y_ad.size() != n_) {
                throw std::invalid_argument("CppADResidualJacobian: functor must return vector of same size as x");
            }

            CppAD::vector<AD> ay(static_cast<size_t>(n_));
            for (int i = 0; i < n_; ++i) ay[static_cast<size_t>(i)] = y_ad(i);

            CppAD::ADFun<double> f;
            f.Dependent(ax, ay);
            return f;
        }
    };

} // namespace Aetherion::ODE::RKMK::Core
