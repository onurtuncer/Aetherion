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
// Two overload families are provided:
//
//   Fixed-size  (N as compile-time template parameter)
//   -------------------------------------------------------
//   NewtonSolve<N>(eval, x, opt)
//   NewtonSolveRJ<N>(residual, jacobian, x, opt)
//   FixedCppADResidualJacobian<N, Functor>       -- zero heap allocation
//   MakeCppADResidualJacobian<N>(functor)        -- factory helper
//
//   All internal Eigen objects are stack-resident. Use in the hot path
//   (e.g. RadauIIA_RKMK_SE3) where N = 6*Stages is known at compile time.
//
//   Dynamic  (runtime n, Eigen::VectorXd / MatrixXd)
//   -------------------------------------------------------
//   NewtonSolve(eval, x, opt)               -- eval takes VectorXd/MatrixXd
//   NewtonSolveRJ(residual, jacobian, x, opt)
//   CppADResidualJacobian(functor, n)       -- functor returns Dynamic vector
//
//   Used by tests and any caller whose dimension is not a compile-time constant.
//
// Allocation policy
// -----------------
// Fixed-size path: zero heap allocations in the Newton loop. The only heap
// traffic is the two std::vector round-trips in CppAD's Forward()/Jacobian()
// API inside FixedCppADResidualJacobian, which are structurally unavoidable.
// Dynamic path: allocates as needed (VectorXd/MatrixXd).
//
// Notes:
//  - CppAD::ADFun::Forward() and Jacobian() are non-const; tape_ is mutable.
//  - Both paths share the same newton_converged() criterion.
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

    // -------------------------------------------------------------------------
    // Result
    // -------------------------------------------------------------------------

    struct NewtonResult final {
        bool   converged = false;
        int    iters = 0;
        int    line_search_iters = 0;

        double r0_norm = 0.0;
        double r_norm = 0.0;
        double dx_norm = 0.0;

        double last_lambda = 1.0;
    };

    // -------------------------------------------------------------------------
    // Shared Newton iteration loop
    //
    // Templated over vector/matrix types so the same logic serves both the
    // fixed-size (VecN/MatN) and dynamic (VectorXd/MatrixXd) paths.
    // MakeVec(n) must return a default-constructed vector of size n.
    // -------------------------------------------------------------------------

    namespace detail {

        template<class VecT, class MatT, class EvalT, class MakeVec>
        inline NewtonResult newton_loop(EvalT&& eval,
            VecT& x,
            const NewtonOptions& opt,
            MakeVec              make_vec) {
            NewtonResult res{};
            const int n = static_cast<int>(x.size());

            VecT r = make_vec(n);
            MatT J(n, n);

            eval(x, r, J);
            res.r0_norm = r.norm();
            res.r_norm = res.r0_norm;

            if (newton_converged(res.r_norm, res.r0_norm, 0.0, x.norm(), opt)) {
                res.converged = true;
                return res;
            }

            for (int k = 0; k < opt.max_iters; ++k) {
                eval(x, r, J);
                const double r_norm = r.norm();
                const double x_norm = x.norm();
                const VecT   dx = J.colPivHouseholderQr().solve(-r);
                const double dx_norm = dx.norm();

                res.iters = k + 1;
                res.r_norm = r_norm;
                res.dx_norm = dx_norm;

                if (newton_converged(r_norm, res.r0_norm, dx_norm, x_norm, opt)) {
                    res.converged = true;
                    res.last_lambda = 1.0;
                    return res;
                }

                if (!opt.use_damping) {
                    x.noalias() += dx;
                    res.last_lambda = 1.0;
                    continue;
                }

                double lambda = opt.damping_init;
                bool accepted = false;

                VecT x_trial = make_vec(n);
                VecT r_trial = make_vec(n);
                MatT J_trial(n, n);

                for (int ls = 0; ls < opt.max_line_search_iters; ++ls) {
                    x_trial = x + lambda * dx;
                    eval(x_trial, r_trial, J_trial);

                    if (r_trial.norm() <= opt.sufficient_decrease * r_norm) {
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
                    if (lambda >= opt.damping_min) {
                        x.noalias() += lambda * dx;
                        res.last_lambda = lambda;
                    }
                    else {
                        res.converged = false;
                        if (opt.throw_on_fail)
                            throw std::runtime_error(
                                "NewtonSolve: line search failed to find a decreasing step");
                        return res;
                    }
                }
            }

            res.converged = false;
            if (opt.throw_on_fail)
                throw std::runtime_error(
                    "NewtonSolve: max_iters reached without convergence");
            return res;
        }

    } // namespace detail

    // =========================================================================
    // FIXED-SIZE PATH  (N known at compile time)
    // =========================================================================

    template<int N, class Eval>
    inline NewtonResult NewtonSolve(Eval&& eval,
        Eigen::Matrix<double, N, 1>& x,
        const NewtonOptions& opt = {}) {
        static_assert(N > 0, "NewtonSolve<N> requires N > 0 at compile time");
        using VecN = Eigen::Matrix<double, N, 1>;
        using MatN = Eigen::Matrix<double, N, N>;
        return detail::newton_loop<VecN, MatN>(
            std::forward<Eval>(eval), x, opt, [](int) { return VecN{}; });
    }

    template<int N, class Residual, class Jacobian>
    inline NewtonResult NewtonSolveRJ(Residual&& residual,
        Jacobian&& jacobian,
        Eigen::Matrix<double, N, 1>& x,
        const NewtonOptions& opt = {}) {
        using VecN = Eigen::Matrix<double, N, 1>;
        using MatN = Eigen::Matrix<double, N, N>;
        auto eval = [&](const VecN& xin, VecN& r, MatN& J) {
            residual(xin, r);
            jacobian(xin, J);
            };
        return NewtonSolve<N>(eval, x, opt);
    }

    // -------------------------------------------------------------------------
    // FixedCppADResidualJacobian<N, Functor>
    //
    // Fixed-size evaluator for the hot path. All Eigen objects are
    // stack-resident. tape_ is mutable because ADFun::Forward/Jacobian
    // are non-const.
    //
    // Functor must provide:
    //   template<class S>
    //   Eigen::Matrix<S, N, 1> operator()(const Eigen::Matrix<S, N, 1>&) const;
    // -------------------------------------------------------------------------

    template<int N, class Functor>
    class FixedCppADResidualJacobian final {
    public:
        using VecN = Eigen::Matrix<double, N, 1>;
        using MatN = Eigen::Matrix<double, N, N>;

        explicit FixedCppADResidualJacobian(Functor f)
            : functor_(std::move(f)) {
            static_assert(N > 0, "FixedCppADResidualJacobian requires N > 0");
            tape_ = build_tape_();
        }

        void operator()(const VecN& x, VecN& r, MatN& J) const {
            const std::vector<double> xd = to_std_(x);
            const std::vector<double> yd = tape_.Forward(0, xd);
            for (int i = 0; i < N; ++i) r(i) = yd[(std::size_t)i];
            const std::vector<double> jd = tape_.Jacobian(xd);
            for (int rr = 0; rr < N; ++rr)
                for (int cc = 0; cc < N; ++cc)
                    J(rr, cc) = jd[(std::size_t)(rr * N + cc)];
        }

    private:
        static std::vector<double> to_std_(const VecN& x) {
            std::vector<double> v(N);
            for (int i = 0; i < N; ++i) v[(std::size_t)i] = x(i);
            return v;
        }

        CppAD::ADFun<double> build_tape_() const {
            using AD = CppAD::AD<double>;
            using VecAD = Eigen::Matrix<AD, N, 1>;

            CppAD::vector<AD> ax(N);
            for (int i = 0; i < N; ++i) ax[(std::size_t)i] = AD(0.0);
            CppAD::Independent(ax);

            VecAD x_ad;
            for (int i = 0; i < N; ++i) x_ad(i) = ax[(std::size_t)i];
            const VecAD y_ad = functor_.template operator() < AD > (x_ad);

            CppAD::vector<AD> ay(N);
            for (int i = 0; i < N; ++i) ay[(std::size_t)i] = y_ad(i);

            CppAD::ADFun<double> f;
            f.Dependent(ax, ay);
            return f;
        }

        Functor                      functor_;
        mutable CppAD::ADFun<double> tape_;
    };

    // Factory: deduces Functor, N must be specified explicitly.
    //   auto eval = MakeCppADResidualJacobian<18>(my_residual);
    template<int N, class Functor>
    [[nodiscard]] auto MakeCppADResidualJacobian(Functor&& f)
        -> FixedCppADResidualJacobian<N, std::decay_t<Functor>> {
        return FixedCppADResidualJacobian<N, std::decay_t<Functor>>(
            std::forward<Functor>(f));
    }

    // =========================================================================
    // DYNAMIC PATH  (n known only at runtime)
    // =========================================================================

    template<class Eval>
    inline NewtonResult NewtonSolve(Eval&& eval,
        Eigen::VectorXd& x,
        const NewtonOptions& opt = {}) {
        const int n = static_cast<int>(x.size());
        if (n <= 0) {
            if (opt.throw_on_fail)
                throw std::invalid_argument("NewtonSolve: x.size() must be > 0");
            return {};
        }
        return detail::newton_loop<Eigen::VectorXd, Eigen::MatrixXd>(
            std::forward<Eval>(eval), x, opt,
            [](int sz) { return Eigen::VectorXd(sz); });
    }

    template<class Residual, class Jacobian>
    inline NewtonResult NewtonSolveRJ(Residual&& residual,
        Jacobian&& jacobian,
        Eigen::VectorXd& x,
        const NewtonOptions& opt = {}) {
        auto eval = [&](const Eigen::VectorXd& xin,
            Eigen::VectorXd& r,
            Eigen::MatrixXd& J) {
                residual(xin, r);
                jacobian(xin, J);
            };
        return NewtonSolve(eval, x, opt);
    }

    // -------------------------------------------------------------------------
    // CppADResidualJacobian (dynamic)
    //
    // Preserves the existing test API:  CppADResidualJacobian eval(functor, n);
    //
    // Functor must provide:
    //   template<class S>
    //   Eigen::Matrix<S, Eigen::Dynamic, 1>
    //     operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>&) const;
    //
    // tape_ is mutable because ADFun::Forward/Jacobian are non-const.
    // -------------------------------------------------------------------------

    template<class Functor>
    class CppADResidualJacobian final {
    public:
        explicit CppADResidualJacobian(Functor f, int n)
            : functor_(std::move(f)), n_(n) {
            if (n_ <= 0) throw std::invalid_argument(
                "CppADResidualJacobian: n must be > 0");
            tape_ = build_tape_();
        }

        void operator()(const Eigen::VectorXd& x,
            Eigen::VectorXd& r,
            Eigen::MatrixXd& J) const {
            const std::vector<double> xd = to_std_(x);
            const std::vector<double> yd = tape_.Forward(0, xd);
            r.resize(n_);
            for (int i = 0; i < n_; ++i) r(i) = yd[(std::size_t)i];
            const std::vector<double> jd = tape_.Jacobian(xd);
            J.resize(n_, n_);
            for (int rr = 0; rr < n_; ++rr)
                for (int cc = 0; cc < n_; ++cc)
                    J(rr, cc) = jd[(std::size_t)(rr * n_ + cc)];
        }

        [[nodiscard]] int size() const noexcept { return n_; }

    private:
        std::vector<double> to_std_(const Eigen::VectorXd& x) const {
            std::vector<double> v((std::size_t)n_);
            for (int i = 0; i < n_; ++i) v[(std::size_t)i] = x(i);
            return v;
        }

        CppAD::ADFun<double> build_tape_() const {
            using AD = CppAD::AD<double>;

            CppAD::vector<AD> ax((std::size_t)n_);
            for (int i = 0; i < n_; ++i) ax[(std::size_t)i] = AD(0.0);
            CppAD::Independent(ax);

            Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
            for (int i = 0; i < n_; ++i) x_ad(i) = ax[(std::size_t)i];
            const auto y_ad = functor_.template operator() < AD > (x_ad);

            CppAD::vector<AD> ay((std::size_t)n_);
            for (int i = 0; i < n_; ++i) ay[(std::size_t)i] = y_ad(i);

            CppAD::ADFun<double> f;
            f.Dependent(ax, ay);
            return f;
        }

        Functor              functor_;
        int                  n_{ 0 };
        mutable CppAD::ADFun<double> tape_;
    };

} // namespace Aetherion::ODE::RKMK::Core

//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// Copyright(c) 2025, Onur Tuncer, PhD,
//// Istanbul Technical University
////
//// SPDX - License - Identifier: MIT
//// License - Filename: LICENSE
//// ------------------------------------------------------------------------------
////
//// File: Aetherion/ODE/RKMK/Core/Newton.h
////
//// Damped Newton solver for implicit RK stage systems.
////
//// Design goals:
////  - Fixed-size Eigen vectors/matrices throughout: all internal objects are
////    stack-resident for N known at compile time.
////  - AD-friendly: CppADResidualJacobian provides residual + Jacobian from a
////    templated residual functor (R^N -> R^N) with fixed-size types end-to-end.
////  - Value semantics: all helpers are small, copyable, and header-only.
////
//// Allocation policy
//// -----------------
//// NewtonSolve<N> and CppADResidualJacobian<N,...> allocate nothing on the heap.
//// The only unavoidable heap traffic is the std::vector round-trips required by
//// CppAD's Forward()/Jacobian() API inside CppADResidualJacobian::operator().
////
//// Notes:
////  - Convergence decisions are performed in double (NewtonOptions uses double).
////  - CppAD tape is recorded once at construction and reused each iteration.
////
//#pragma once
//
//#include <utility>
//#include <stdexcept>
//#include <vector>
//#include <cmath>
//
//#include <Eigen/Dense>
//#include <cppad/cppad.hpp>
//
//#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
//
//namespace Aetherion::ODE::RKMK::Core {
//
//    // -------------------------------------------------------------------------
//    // Result
//    // -------------------------------------------------------------------------
//
//    struct NewtonResult final {
//        bool   converged = false;
//        int    iters = 0;
//        int    line_search_iters = 0;
//
//        double r0_norm = 0.0;
//        double r_norm = 0.0;
//        double dx_norm = 0.0;
//
//        double last_lambda = 1.0;
//    };
//
//    // -------------------------------------------------------------------------
//    // Concept: fixed-size ResidualJacobianEvaluator
//    //
//    // Eval must provide:
//    //   void operator()(const Eigen::Matrix<double,N,1>&  x,
//    //                         Eigen::Matrix<double,N,1>&  r,
//    //                         Eigen::Matrix<double,N,N>&  J) const;
//    // -------------------------------------------------------------------------
//
//    template<class Eval, int N>
//    concept ResidualJacobianEvaluator =
//        requires(Eval                              eval,
//    const Eigen::Matrix<double, N, 1>&x,
//        Eigen::Matrix<double, N, 1>&r,
//        Eigen::Matrix<double, N, N>&J) {
//            { eval(x, r, J) } -> std::same_as<void>;
//    };
//
//    // -------------------------------------------------------------------------
//    // NewtonSolve<N>
//    //
//    // All internal objects are fixed-size: VecN and MatN are stack-resident.
//    // The line-search trial vector x_trial is also fixed-size.
//    // -------------------------------------------------------------------------
//
//    template<int N, class Eval>
//        requires ResidualJacobianEvaluator<Eval, N>
//    inline NewtonResult NewtonSolve(Eval&& eval,
//        Eigen::Matrix<double, N, 1>& x,
//        const NewtonOptions& opt = {}) {
//        static_assert(N > 0, "NewtonSolve requires N > 0 at compile time");
//
//        using VecN = Eigen::Matrix<double, N, 1>;
//        using MatN = Eigen::Matrix<double, N, N>;
//
//        NewtonResult res{};
//
//        VecN r;
//        MatN J;
//
//        // Initial residual
//        eval(x, r, J);
//        res.r0_norm = r.norm();
//        res.r_norm = res.r0_norm;
//
//        // Early exit
//        if (newton_converged(res.r_norm, res.r0_norm, 0.0, x.norm(), opt)) {
//            res.converged = true;
//            res.iters = 0;
//            return res;
//        }
//
//        for (int k = 0; k < opt.max_iters; ++k) {
//            eval(x, r, J);
//            const double r_norm = r.norm();
//            const double x_norm = x.norm();
//
//            // Solve J dx = -r via column-pivoting QR (robust for near-singular J)
//            const VecN dx = J.colPivHouseholderQr().solve(-r);
//            const double dx_norm = dx.norm();
//
//            res.iters = k + 1;
//            res.r_norm = r_norm;
//            res.dx_norm = dx_norm;
//
//            if (newton_converged(r_norm, res.r0_norm, dx_norm, x_norm, opt)) {
//                res.converged = true;
//                res.last_lambda = 1.0;
//                return res;
//            }
//
//            // ------------------------------------------------------------------
//            // Damping / line search
//            // ------------------------------------------------------------------
//
//            if (!opt.use_damping) {
//                x.noalias() += dx;
//                res.last_lambda = 1.0;
//                continue;
//            }
//
//            double lambda = opt.damping_init;
//            bool accepted = false;
//
//            VecN x_trial;
//            VecN r_trial;
//            MatN J_trial;   // evaluator fills J_trial; we only inspect r_trial
//
//            for (int ls = 0; ls < opt.max_line_search_iters; ++ls) {
//                x_trial = x + lambda * dx;
//                eval(x_trial, r_trial, J_trial);
//
//                if (r_trial.norm() <= opt.sufficient_decrease * r_norm) {
//                    x = x_trial;
//                    accepted = true;
//                    res.line_search_iters += (ls + 1);
//                    res.last_lambda = lambda;
//                    break;
//                }
//
//                lambda *= opt.damping_shrink;
//                if (lambda < opt.damping_min) break;
//            }
//
//            if (!accepted) {
//                if (lambda >= opt.damping_min) {
//                    x.noalias() += lambda * dx;
//                    res.last_lambda = lambda;
//                }
//                else {
//                    res.converged = false;
//                    if (opt.throw_on_fail)
//                        throw std::runtime_error(
//                            "NewtonSolve: line search failed to find a decreasing step");
//                    return res;
//                }
//            }
//        }
//
//        // Max iterations reached
//        res.converged = false;
//        if (opt.throw_on_fail)
//            throw std::runtime_error("NewtonSolve: max_iters reached without convergence");
//        return res;
//    }
//
//    // -------------------------------------------------------------------------
//    // Convenience wrapper: separate residual + analytic Jacobian callbacks
//    // -------------------------------------------------------------------------
//
//    template<int N, class Residual, class Jacobian>
//    inline NewtonResult NewtonSolveRJ(Residual&& residual,
//        Jacobian&& jacobian,
//        Eigen::Matrix<double, N, 1>& x,
//        const NewtonOptions& opt = {}) {
//        using VecN = Eigen::Matrix<double, N, 1>;
//        using MatN = Eigen::Matrix<double, N, N>;
//
//        auto eval = [&](const VecN& xin, VecN& r, MatN& J) {
//            residual(xin, r);
//            jacobian(xin, J);
//            };
//        return NewtonSolve<N>(eval, x, opt);
//    }
//
//    // -------------------------------------------------------------------------
//    // CppADResidualJacobian<N, Functor>
//    //
//    // Wraps a templated residual functor and provides operator()(x, r, J) for
//    // use with NewtonSolve<N>. All Eigen objects are fixed-size (stack-resident).
//    // The two std::vector copies are the irreducible cost of CppAD's API.
//    //
//    // Functor must provide:
//    //   template<class S>
//    //   Eigen::Matrix<S, N, 1> operator()(const Eigen::Matrix<S, N, 1>& x) const;
//    // -------------------------------------------------------------------------
//
//    template<int N, class Functor>
//    class CppADResidualJacobian final {
//    public:
//        using VecN = Eigen::Matrix<double, N, 1>;
//        using MatN = Eigen::Matrix<double, N, N>;
//
//        explicit CppADResidualJacobian(Functor f)
//            : functor_(std::move(f)) {
//            static_assert(N > 0, "CppADResidualJacobian requires N > 0 at compile time");
//            tape_ = build_tape_();
//        }
//
//        void operator()(const VecN& x, VecN& r, MatN& J) const {
//            // CppAD's Forward/Jacobian require std::vector -- unavoidable copies.
//            const std::vector<double> xd = to_std_(x);
//
//            const std::vector<double> yd = tape_.Forward(0, xd);
//            for (int i = 0; i < N; ++i) r(i) = yd[(std::size_t)i];
//
//            const std::vector<double> jd = tape_.Jacobian(xd);
//            for (int rr = 0; rr < N; ++rr)
//                for (int cc = 0; cc < N; ++cc)
//                    J(rr, cc) = jd[(std::size_t)(rr * N + cc)];
//        }
//
//    private:
//        static std::vector<double> to_std_(const VecN& x) {
//            std::vector<double> v(N);
//            for (int i = 0; i < N; ++i) v[(std::size_t)i] = x(i);
//            return v;
//        }
//
//        CppAD::ADFun<double> build_tape_() const {
//            using AD = CppAD::AD<double>;
//            using VecAD = Eigen::Matrix<AD, N, 1>;
//
//            CppAD::vector<AD> ax(N);
//            for (int i = 0; i < N; ++i) ax[(std::size_t)i] = AD(0.0);
//            CppAD::Independent(ax);
//
//            VecAD x_ad;
//            for (int i = 0; i < N; ++i) x_ad(i) = ax[(std::size_t)i];
//
//            const VecAD y_ad = functor_.template operator() < AD > (x_ad);
//
//            CppAD::vector<AD> ay(N);
//            for (int i = 0; i < N; ++i) ay[(std::size_t)i] = y_ad(i);
//
//            CppAD::ADFun<double> f;
//            f.Dependent(ax, ay);
//            return f;
//        }
//
//        Functor              functor_;
//        CppAD::ADFun<double> tape_;
//    };
//
//} // namespace Aetherion::ODE::RKMK::Core

//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// Copyright(c) 2025, Onur Tuncer, PhD,
//// Istanbul Technical University
////
//// SPDX - License - Identifier: MIT
//// License - Filename: LICENSE
//// ------------------------------------------------------------------------------
////
//// File: Aetherion/ODE/RKMK/Core/Newton.h
////
//// Damped Newton solver for implicit RK stage systems.
////
//// Design goals:
////  - Works with Eigen vectors/matrices (double).
////  - AD-friendly: optional CppAD-based evaluator that provides residual + Jacobian
////    from a templated residual functor (R^n -> R^n).
////  - Value semantics: all helpers are small, copyable, and header-only.
////
//// Notes:
////  - Convergence decisions are performed in double (NewtonOptions uses double).
////  - For CppAD: we "tape once", then reuse Forward/Jacobian each iteration.
////
//#pragma once
//
//#include <utility>
//#include <stdexcept>
//#include <vector>
//#include <cmath>
//
//#include <Eigen/Dense>
//#include <cppad/cppad.hpp>
//
//#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
//
//namespace Aetherion::ODE::RKMK::Core {
//
//    // ----------------------------------------------------------------------------
//    // Result
//    // ----------------------------------------------------------------------------
//
//    struct NewtonResult final {
//        bool   converged = false;
//        int    iters = 0;
//        int    line_search_iters = 0;
//
//        double r0_norm = 0.0;
//        double r_norm = 0.0;
//        double dx_norm = 0.0;
//
//        double last_lambda = 1.0;
//    };
//
//    // ----------------------------------------------------------------------------
//    // Concepts-lite (keep header independent from Concepts.h to avoid include cycles)
//    // ----------------------------------------------------------------------------
//
//    template<class Eval>
//    concept ResidualJacobianEvaluator =
//        requires(Eval eval,
//    const Eigen::VectorXd & x,
//        Eigen::VectorXd & r,
//        Eigen::MatrixXd & J) {
//            { eval(x, r, J) } -> std::same_as<void>;
//    };
//
//    // ----------------------------------------------------------------------------
//    // Utilities
//    // ----------------------------------------------------------------------------
//
//    [[nodiscard]] inline double norm2(const Eigen::VectorXd& v) noexcept {
//        return v.norm();
//    }
//
//    // ----------------------------------------------------------------------------
//    // Newton: uses an evaluator that provides residual r(x) and Jacobian J(x)
//    // ----------------------------------------------------------------------------
//
//    template<ResidualJacobianEvaluator Eval>
//    inline NewtonResult NewtonSolve(Eval&& eval,
//        Eigen::VectorXd& x,
//        const NewtonOptions& opt = {}) {
//        NewtonResult res{};
//
//        const int n = static_cast<int>(x.size());
//        if (n <= 0) {
//            if (opt.throw_on_fail) throw std::invalid_argument("NewtonSolve: x.size() must be > 0");
//            return res;
//        }
//
//        Eigen::VectorXd r(n);
//        Eigen::MatrixXd J(n, n);
//
//        // initial residual
//        eval(x, r, J);
//        res.r0_norm = norm2(r);
//        res.r_norm = res.r0_norm;
//
//        // early exit
//        if (newton_converged(res.r_norm, res.r0_norm, 0.0, norm2(x), opt)) {
//            res.converged = true;
//            res.iters = 0;
//            return res;
//        }
//
//        for (int k = 0; k < opt.max_iters; ++k) {
//            // Recompute at current x
//            eval(x, r, J);
//            const double r_norm = norm2(r);
//
//            // Solve J dx = -r
//            // Use a robust QR (works for rank-deficient-ish cases better than LDLT).
//            Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-r);
//
//            const double dx_norm = norm2(dx);
//            const double x_norm = norm2(x);
//
//            res.iters = k + 1;
//            res.r_norm = r_norm;
//            res.dx_norm = dx_norm;
//
//            if (newton_converged(r_norm, res.r0_norm, dx_norm, x_norm, opt)) {
//                res.converged = true;
//                res.last_lambda = 1.0;
//                return res;
//            }
//
//            // Damping / line search
//            double lambda = opt.use_damping ? opt.damping_init : 1.0;
//
//            if (!opt.use_damping) {
//                x.noalias() += dx;
//                res.last_lambda = 1.0;
//                continue;
//            }
//
//            // Try to ensure residual decreases
//            bool accepted = false;
//            Eigen::VectorXd x_trial(n);
//            Eigen::VectorXd r_trial(n);
//            Eigen::MatrixXd J_trial(n, n); // not strictly needed, but evaluator fills it
//
//            for (int ls = 0; ls < opt.max_line_search_iters; ++ls) {
//                x_trial = x + lambda * dx;
//
//                eval(x_trial, r_trial, J_trial);
//                const double r_new = norm2(r_trial);
//
//                if (r_new <= opt.sufficient_decrease * r_norm) {
//                    x = x_trial;
//                    accepted = true;
//                    res.line_search_iters += (ls + 1);
//                    res.last_lambda = lambda;
//                    break;
//                }
//
//                lambda *= opt.damping_shrink;
//                if (lambda < opt.damping_min) break;
//            }
//
//            if (!accepted) {
//                // Fall back: take the smallest step we tried (or no step if lambda underflowed)
//                if (lambda >= opt.damping_min) {
//                    x.noalias() += lambda * dx;
//                    res.last_lambda = lambda;
//                }
//                else {
//                    // Can't find a decent step; declare failure
//                    res.converged = false;
//                    if (opt.throw_on_fail) {
//                        throw std::runtime_error("NewtonSolve: line search failed to find decreasing step");
//                    }
//                    return res;
//                }
//            }
//        }
//
//        // max iters reached
//        res.converged = false;
//        if (opt.throw_on_fail) {
//            throw std::runtime_error("NewtonSolve: max_iters reached without convergence");
//        }
//        return res;
//    }
//
//    // ----------------------------------------------------------------------------
//    // Convenience wrapper: residual-only + analytic Jacobian callback
//    // ----------------------------------------------------------------------------
//
//    template<class Residual, class Jacobian>
//    inline NewtonResult NewtonSolveRJ(Residual&& residual,
//        Jacobian&& jacobian,
//        Eigen::VectorXd& x,
//        const NewtonOptions& opt = {}) {
//        auto eval = [&](const Eigen::VectorXd& xin, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
//            residual(xin, r);
//            jacobian(xin, J);
//            };
//        return NewtonSolve(eval, x, opt);
//    }
//
//    // ----------------------------------------------------------------------------
//    // CppAD evaluator: tape once, then provide r(x) and J(x)
//    // ----------------------------------------------------------------------------
//    //
//    // Requirements on Functor F:
//    //   template<class S>
//    //   Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const;
//    //
//    // and it must return a vector of the same size as x (square system).
//    //
//    template<class Functor>
//    class CppADResidualJacobian final {
//    public:
//        explicit CppADResidualJacobian(Functor f, int n)
//            : functor_(std::move(f)), n_(n) {
//            if (n_ <= 0) throw std::invalid_argument("CppADResidualJacobian: n must be > 0");
//            tape_ = build_tape_();
//        }
//
//        void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J) {
//            if (x.size() != n_) throw std::invalid_argument("CppADResidualJacobian: x has wrong size");
//
//            // Forward(0) returns y = f(x)
//            const auto y = tape_.Forward(0, to_cppad_(x));
//            r = from_cppad_vec_(y);
//
//            // Jacobian: row-major vector length n*n
//            const auto jac = tape_.Jacobian(to_cppad_(x));
//            J = from_cppad_jac_(jac);
//        }
//
//        [[nodiscard]] int size() const noexcept { return n_; }
//
//    private:
//        Functor functor_;
//        int n_{ 0 };
//        CppAD::ADFun<double> tape_;
//
//        static CppAD::vector<double> to_cppad_(const Eigen::VectorXd& x) {
//            CppAD::vector<double> v(static_cast<size_t>(x.size()));
//            for (int i = 0; i < x.size(); ++i) v[static_cast<size_t>(i)] = x(i);
//            return v;
//        }
//
//        Eigen::VectorXd from_cppad_vec_(const CppAD::vector<double>& v) const {
//            Eigen::VectorXd out(n_);
//            for (int i = 0; i < n_; ++i) out(i) = v[static_cast<size_t>(i)];
//            return out;
//        }
//
//        Eigen::MatrixXd from_cppad_jac_(const CppAD::vector<double>& jac) const {
//            Eigen::MatrixXd J(n_, n_);
//            // CppAD Jacobian is in row-major order by default: dy_i/dx_j stored at i*n + j
//            for (int i = 0; i < n_; ++i) {
//                for (int j = 0; j < n_; ++j) {
//                    J(i, j) = jac[static_cast<size_t>(i * n_ + j)];
//                }
//            }
//            return J;
//        }
//
//        CppAD::ADFun<double> build_tape_() const {
//            using AD = CppAD::AD<double>;
//
//            CppAD::vector<AD> ax(static_cast<size_t>(n_));
//            for (int i = 0; i < n_; ++i) ax[static_cast<size_t>(i)] = AD(0.0);
//
//            CppAD::Independent(ax);
//
//            Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
//            for (int i = 0; i < n_; ++i) x_ad(i) = ax[static_cast<size_t>(i)];
//
//            Eigen::Matrix<AD, Eigen::Dynamic, 1> y_ad = functor_(x_ad);
//
//            if (y_ad.size() != n_) {
//                throw std::invalid_argument("CppADResidualJacobian: functor must return vector of same size as x");
//            }
//
//            CppAD::vector<AD> ay(static_cast<size_t>(n_));
//            for (int i = 0; i < n_; ++i) ay[static_cast<size_t>(i)] = y_ad(i);
//
//            CppAD::ADFun<double> f;
//            f.Dependent(ax, ay);
//            return f;
//        }
//    };
//
//} // namespace Aetherion::ODE::RKMK::Core
