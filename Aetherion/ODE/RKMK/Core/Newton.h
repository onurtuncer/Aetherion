// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
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

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <stdexcept>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Core/TapeParameters.h>

namespace Aetherion::ODE::RKMK::Core {

    // -------------------------------------------------------------------------
    // Result
    // -------------------------------------------------------------------------

    struct NewtonResult final
    {
        bool converged = false;
        int iters = 0;
        int line_search_iters = 0;

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

        // An evaluator may offer its two halves separately:
        //
        //   eval.residual(x, r)   -- the residual at x;
        //   eval.jacobian(x, J)   -- the Jacobian at x, the point of the last
        //                            residual() call.
        //
        // The loop then asks for a Jacobian only where it uses one. A line
        // search judges a trial point on its residual, so a rejected trial
        // costs no Jacobian. An evaluator without the two members is called
        // in full, eval(x, r, J), as it always was.
        template <class EvalT, class VecT, class MatT>
        concept SplitResidualJacobian = requires(EvalT& eval, const VecT& x, VecT& r, MatT& J) {
            eval.residual(x, r);
            eval.jacobian(x, J);
        };

        // Residual at x; true when J was filled as well (evaluator called in full).
        template <class VecT, class MatT, class EvalT>
        inline bool newton_residual(EvalT& eval, const VecT& x, VecT& r, MatT& J)
        {
            if constexpr (SplitResidualJacobian<EvalT, VecT, MatT>) {
                eval.residual(x, r);
                return false;
            } else {
                eval(x, r, J);
                return true;
            }
        }

        // Jacobian at x, whose residual r is already known.
        template <class VecT, class MatT, class EvalT>
        inline void newton_jacobian(EvalT& eval, const VecT& x, VecT& r, MatT& J)
        {
            if constexpr (SplitResidualJacobian<EvalT, VecT, MatT>)
                eval.jacobian(x, J);
            else
                eval(x, r, J);
        }

        // What a line search left behind.
        enum class LineSearchOutcome : std::uint8_t
        {
            Accepted, // x is a trial point that decreased the residual; r (and J if has_J) are evaluated there
            Damped,   // no trial was accepted; x moved by the last damped step and is not evaluated
            Failed    // the step length fell below damping_min
        };

        // Backtracking line search along dx from x, whose residual norm is
        // r_norm. Trial points are judged on their residual; the buffers are
        // the caller's so that nothing is allocated per iteration.
        template <class VecT, class MatT, class EvalT>
        inline LineSearchOutcome newton_line_search(EvalT& eval,
            const NewtonOptions& opt,
            const VecT& dx,
            double r_norm,
            VecT& x,
            VecT& r,
            MatT& J,
            bool& has_J,
            VecT& x_trial,
            VecT& r_trial,
            MatT& J_trial,
            NewtonResult& res)
        {
            double lambda = opt.damping_init;
            for (int ls = 0; ls < opt.max_line_search_iters; ++ls) {
                x_trial = x + lambda * dx;
                const bool trial_has_J = newton_residual<VecT, MatT>(eval, x_trial, r_trial, J_trial);

                if (r_trial.norm() <= opt.sufficient_decrease * r_norm) {
                    x = x_trial;
                    r = r_trial;
                    if (trial_has_J) J = J_trial;
                    has_J = trial_has_J;
                    res.line_search_iters += (ls + 1);
                    res.last_lambda = lambda;
                    return LineSearchOutcome::Accepted;
                }

                lambda *= opt.damping_shrink;
                if (lambda < opt.damping_min) break;
            }

            if (lambda < opt.damping_min) return LineSearchOutcome::Failed;
            x.noalias() += lambda * dx;
            res.last_lambda = lambda;
            return LineSearchOutcome::Damped;
        }

        template <class VecT, class MatT, class EvalT, class MakeVec>
        inline NewtonResult newton_loop(EvalT&& eval, // NOLINT(cppcoreguidelines-missing-std-forward): reused as an lvalue
            VecT& x,
            const NewtonOptions& opt,
            MakeVec make_vec)
        {
            NewtonResult res{};
            const int n = static_cast<int>(x.size());

            VecT r = make_vec(n);
            MatT J(n, n);

            // have_r / have_J: r, J are the evaluation at the current x. They
            // are after the first evaluation, and after a line search accepted
            // a trial point, which was evaluated to be judged; the iteration
            // then starts from that evaluation instead of repeating it at the
            // same point. The evaluator is a function of x (for the taped
            // evaluators below, a second call at the point of the previous one
            // replays the same tape), so the iterates are those of a loop that
            // evaluates again, bit for bit; only the number of calls differs.
            bool have_J = newton_residual<VecT, MatT>(eval, x, r, J);
            bool have_r = true;
            res.r0_norm = r.norm();
            res.r_norm = res.r0_norm;

            if (newton_converged(res.r_norm, res.r0_norm, 0.0, x.norm(), opt)) {
                res.converged = true;
                return res;
            }

            VecT x_trial = make_vec(n);
            VecT r_trial = make_vec(n);
            MatT J_trial(n, n);

            for (int k = 0; k < opt.max_iters; ++k) {
                if (!have_r) have_J = newton_residual<VecT, MatT>(eval, x, r, J);
                have_r = false;
                const double r_norm = r.norm();
                const double x_norm = x.norm();

                // The residual criterion alone settles convergence here, and
                // the Newton step would be computed only to report its norm.
                if (!opt.report_step_norm && r_norm <= opt.abs_tol + opt.rel_tol * res.r0_norm) {
                    res.iters = k + 1;
                    res.r_norm = r_norm;
                    res.dx_norm = std::numeric_limits<double>::quiet_NaN();
                    res.converged = true;
                    res.last_lambda = 1.0;
                    return res;
                }

                if (!have_J) newton_jacobian<VecT, MatT>(eval, x, r, J);
                have_J = false;
                const VecT dx = J.colPivHouseholderQr().solve(-r);
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

                const LineSearchOutcome outcome = newton_line_search<VecT, MatT>(
                    eval, opt, dx, r_norm, x, r, J, have_J, x_trial, r_trial, J_trial, res);
                have_r = (outcome == LineSearchOutcome::Accepted);
                if (outcome == LineSearchOutcome::Failed) {
                    res.converged = false;
                    if (opt.throw_on_fail)
                        throw std::runtime_error(
                            "NewtonSolve: line search failed to find a decreasing step");
                    return res;
                }
            }

            // The loop tests convergence before each correction, never after
            // the last one it is allowed to take. Evaluate the iterate that is
            // actually returned, so a root reached on the final iteration is
            // reported as such and r_norm / dx_norm describe x, not its
            // predecessor.
            if (!have_r) have_J = newton_residual<VecT, MatT>(eval, x, r, J);
            if (!have_J) newton_jacobian<VecT, MatT>(eval, x, r, J);
            res.r_norm = r.norm();
            res.dx_norm = J.colPivHouseholderQr().solve(-r).norm();
            if (newton_converged(res.r_norm, res.r0_norm, res.dx_norm, x.norm(), opt)) {
                res.converged = true;
                return res;
            }

            res.converged = false;
            if (opt.throw_on_fail)
                throw std::runtime_error(
                    "NewtonSolve: max_iters reached without convergence");
            return res;
        }

        // Zero-order Forward of a tape kept across calls, re-recorded at xd when
        // a comparison on it went the other way there.
        //
        // The kept tape may be stale at xd: its arithmetic is that of the region
        // it was recorded in, which need not be defined at xd (sqrt(-x) recorded
        // for x < 0, asked about x > 0). Evaluated with CppAD's NaN check on --
        // the default without NDEBUG -- the stale branch aborted the run before
        // compare_change_number() could say the tape was stale, so the recovery
        // never ran (second pre-DOI audit, N08). The first pass therefore runs
        // with the check off. If a comparison changed, the result is discarded
        // and the fresh tape is evaluated with the check as configured; if none
        // changed, the tape is valid at xd and a non-finite value is the
        // function's own -- it is evaluated again with the check on, so CppAD
        // reports it exactly as before.
        template <class BuildTape>
        inline std::vector<double> forward_or_retape(CppAD::ADFun<double>& tape,
            const std::vector<double>& xd,
            BuildTape&& build,
            int& retapes)
        {
            const bool check = tape.check_for_nan();
            tape.check_for_nan(false);
            std::vector<double> yd = tape.Forward(0, xd);
            tape.check_for_nan(check);
            if (tape.compare_change_number() > 0) {
                tape = std::forward<BuildTape>(build)(xd);
                ++retapes;
                return tape.Forward(0, xd);
            }
            if (check && !std::all_of(yd.begin(), yd.end(), [](double v) { return std::isfinite(v); }))
                yd = tape.Forward(0, xd);
            return yd;
        }

        // The two callbacks of NewtonSolveRJ as one evaluator that offers its
        // halves separately (see SplitResidualJacobian).
        template <class Residual, class Jacobian>
        struct ResidualJacobianPair
        {
            std::reference_wrapper<std::remove_reference_t<Residual>> residual_fn;
            std::reference_wrapper<std::remove_reference_t<Jacobian>> jacobian_fn;

            template <class VecT>
            void residual(const VecT& x, VecT& r)
            {
                residual_fn(x, r);
            }
            template <class VecT, class MatT>
            void jacobian(const VecT& x, MatT& J)
            {
                jacobian_fn(x, J);
            }
            template <class VecT, class MatT>
            void operator()(const VecT& x, VecT& r, MatT& J)
            {
                residual_fn(x, r);
                jacobian_fn(x, J);
            }
        };

        // Dense Jacobian of a tape at xd, row-major, the tape's zero-order
        // coefficients being those of xd (a zero-order sweep at xd was the
        // last thing done to it).
        //
        // single_sweep = false: CppAD's driver, ADFun::Jacobian. In forward
        // mode -- which it chooses when the tape has no more independents than
        // non-constant dependents -- it makes one first-order sweep of the
        // whole tape per independent, after a zero-order sweep of its own.
        //
        // single_sweep = true: the same first-order coefficients, all
        // directions carried through ONE sweep (ADFun::Forward(1, r, xq) with
        // r = n unit directions). Every operation applies the same first-order
        // rule to each direction as the one-direction sweep does, so the
        // Jacobian is the driver's, entry for entry; that is what
        // tests/ODE/test_NewtonEvaluations.cpp asserts, bit for bit, and it is a property
        // of the CppAD in use, not a promise of its documentation. Where the
        // driver would work in reverse mode it is called as before.
        inline std::vector<double> tape_jacobian(CppAD::ADFun<double>& tape,
            const std::vector<double>& xd,
            bool single_sweep)
        {
            if (!single_sweep) return tape.Jacobian(xd);

            const std::size_t n = tape.Domain();
            const std::size_t m = tape.Range();
            std::size_t non_constant = 0;
            for (std::size_t i = 0; i < m; ++i)
                if (!tape.Parameter(i)) ++non_constant;
            if (n > non_constant) return tape.Jacobian(xd); // the driver's reverse-mode case

            std::vector<double> directions(n * n, 0.0);
            for (std::size_t j = 0; j < n; ++j) directions[n * j + j] = 1.0;
            return tape.Forward(1, n, directions); // [n * i + j] = dy_i / dx_j
        }

        [[nodiscard]] inline bool same_point(const std::vector<double>& a, const std::vector<double>& b) noexcept
        {
            if (a.size() != b.size()) return false;
            for (std::size_t i = 0; i < a.size(); ++i)
                if (!BitIdentical(a[i], b[i])) return false;
            return true;
        }

    } // namespace detail

    // =========================================================================
    // FIXED-SIZE PATH  (N known at compile time)
    // =========================================================================

    template <int N, class Eval>
    inline NewtonResult NewtonSolve(Eval&& eval,
        Eigen::Matrix<double, N, 1>& x,
        const NewtonOptions& opt = {})
    {
        static_assert(N > 0, "NewtonSolve<N> requires N > 0 at compile time");
        using VecN = Eigen::Matrix<double, N, 1>;
        using MatN = Eigen::Matrix<double, N, N>;
        return detail::newton_loop<VecN, MatN>(
            std::forward<Eval>(eval), x, opt, [](int) { return VecN{}; });
    }

    template <int N, class Residual, class Jacobian>
    inline NewtonResult NewtonSolveRJ(Residual&& residual, // NOLINT(cppcoreguidelines-missing-std-forward): borrowed for this solve
        Jacobian&& jacobian,                               // NOLINT(cppcoreguidelines-missing-std-forward): borrowed for this solve
        Eigen::Matrix<double, N, 1>& x,
        const NewtonOptions& opt = {})
    {
        detail::ResidualJacobianPair<Residual, Jacobian> eval{ residual, jacobian };
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

    template <int N, class Functor>
    class FixedCppADResidualJacobian final
    {
    public:
        using VecN = Eigen::Matrix<double, N, 1>;
        using MatN = Eigen::Matrix<double, N, N>;

        explicit FixedCppADResidualJacobian(Functor f)
            : functor_(std::move(f))
        {
            static_assert(N > 0, "FixedCppADResidualJacobian requires N > 0");
        }

        // The tape is recorded at the first point it is asked about and kept
        // while the branches recorded on it (piecewise conditions, table
        // segments, atmosphere layers -- any comparison the functor makes on
        // AD values) still hold at x. CppAD reports a comparison going the
        // other way through compare_change_number(); the tape is then
        // re-recorded at x. Without this the Newton iteration converged on the
        // residual of the region the tape was recorded in, not on the
        // residual of the function (pre-DOI audit finding F01). The stale tape
        // is evaluated without CppAD's NaN check, so arithmetic that is invalid
        // only in the old region cannot stop the re-recording (N08; see
        // detail::forward_or_retape).
        void operator()(const VecN& x, VecN& r, MatN& J) const
        {
            residual(x, r);
            jacobian(x, J);
        }

        /// Residual at x. This is the call that judges the tape: it is
        /// re-recorded here if a comparison on it changed at x.
        void residual(const VecN& x, VecN& r) const
        {
            last_x_ = to_std_(x);
            const std::vector<double> yd = sweep_();
            for (int i = 0; i < N; ++i) r(i) = yd[(std::size_t)i];
        }

        /// Jacobian at x. The tape is valid at the point of the last
        /// residual() call; anywhere else it is judged first.
        void jacobian(const VecN& x, MatN& J) const
        {
            const std::vector<double> xd = to_std_(x);
            if (!taped_ || !detail::same_point(xd, last_x_)) {
                last_x_ = xd;
                (void)sweep_();
            }
            const std::vector<double> jd = detail::tape_jacobian(tape_, xd, single_sweep_);
            ++jacobians_;
            for (int rr = 0; rr < N; ++rr)
                for (int cc = 0; cc < N; ++cc)
                    J(rr, cc) = jd[(std::size_t)rr * N + (std::size_t)cc];
        }

        /// Number of times the tape was re-recorded because a comparison on it
        /// went the other way at the point being evaluated.
        [[nodiscard]] int retape_count() const noexcept { return retapes_; }

        /// Take the Jacobian in one first-order sweep carrying every
        /// direction instead of one sweep per direction (off by default; see
        /// detail::tape_jacobian).
        void set_jacobian_single_sweep(bool on) noexcept { single_sweep_ = on; }

        /// Number of tape recordings (the first one and every re-recording).
        [[nodiscard]] int tape_count() const noexcept { return tapes_; }
        /// Number of residual evaluations (zero-order sweeps asked for).
        [[nodiscard]] int residual_count() const noexcept { return residuals_; }
        /// Number of Jacobian evaluations.
        [[nodiscard]] int jacobian_count() const noexcept { return jacobians_; }

    private:
        static std::vector<double> to_std_(const VecN& x)
        {
            std::vector<double> v(N);
            for (int i = 0; i < N; ++i) v[(std::size_t)i] = x(i);
            return v;
        }

        // Zero-order sweep at last_x_, recording or re-recording as needed.
        std::vector<double> sweep_() const
        {
            if (!taped_) {
                tape_ = build_tape_(last_x_);
                taped_ = true;
                ++tapes_;
            }
            const int before = retapes_;
            std::vector<double> yd = detail::forward_or_retape(
                tape_, last_x_, [this](const std::vector<double>& at) { return build_tape_(at); }, retapes_);
            tapes_ += retapes_ - before;
            ++residuals_;
            return yd;
        }

        CppAD::ADFun<double> build_tape_(const std::vector<double>& at) const
        {
            using AD = CppAD::AD<double>;
            using VecAD = Eigen::Matrix<AD, N, 1>;

            CppAD::vector<AD> ax(N);
            for (int i = 0; i < N; ++i) ax[(std::size_t)i] = AD(at[(std::size_t)i]);
            CppAD::Independent(ax);

            VecAD x_ad;
            for (int i = 0; i < N; ++i) x_ad(i) = ax[(std::size_t)i];
            const VecAD y_ad = functor_.template operator()<AD>(x_ad);

            CppAD::vector<AD> ay(N);
            for (int i = 0; i < N; ++i) ay[(std::size_t)i] = y_ad(i);

            CppAD::ADFun<double> f;
            f.Dependent(ax, ay);
            f.compare_change_count(1); // off by default: switch change tracking on
            return f;
        }

        Functor functor_;
        mutable CppAD::ADFun<double> tape_;
        mutable bool taped_{ false };
        mutable int retapes_{ 0 };
        mutable int tapes_{ 0 };
        mutable int residuals_{ 0 };
        mutable int jacobians_{ 0 };
        mutable std::vector<double> last_x_;
        bool single_sweep_{ false };
    };

    // Factory: deduces Functor, N must be specified explicitly.
    //   auto eval = MakeCppADResidualJacobian<18>(my_residual);
    template <int N, class Functor>
    [[nodiscard]] auto MakeCppADResidualJacobian(Functor&& f)
        -> FixedCppADResidualJacobian<N, std::decay_t<Functor>>
    {
        return FixedCppADResidualJacobian<N, std::decay_t<Functor>>(
            std::forward<Functor>(f));
    }

    // =========================================================================
    // DYNAMIC PATH  (n known only at runtime)
    // =========================================================================

    template <class Eval>
    inline NewtonResult NewtonSolve(Eval&& eval,
        Eigen::VectorXd& x,
        const NewtonOptions& opt = {})
    {
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

    template <class Residual, class Jacobian>
    inline NewtonResult NewtonSolveRJ(Residual&& residual, // NOLINT(cppcoreguidelines-missing-std-forward): borrowed for this solve
        Jacobian&& jacobian,                               // NOLINT(cppcoreguidelines-missing-std-forward): borrowed for this solve
        Eigen::VectorXd& x,
        const NewtonOptions& opt = {})
    {
        detail::ResidualJacobianPair<Residual, Jacobian> eval{ residual, jacobian };
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

    template <class Functor>
    class CppADResidualJacobian final
    {
    public:
        explicit CppADResidualJacobian(Functor f, int n)
            : functor_(std::move(f))
            , n_(n)
        {
            if (n_ <= 0) throw std::invalid_argument(
                "CppADResidualJacobian: n must be > 0");
        }

        // Recorded at the first point asked about; re-recorded whenever a
        // comparison on the tape goes the other way at x (see the fixed-size
        // evaluator above).
        void operator()(const Eigen::VectorXd& x,
            Eigen::VectorXd& r,
            Eigen::MatrixXd& J) const
        {
            residual(x, r);
            jacobian(x, J);
        }

        /// Residual at x. This is the call that judges the tape: it is
        /// re-recorded here if a comparison on it changed at x.
        void residual(const Eigen::VectorXd& x, Eigen::VectorXd& r) const
        {
            last_x_ = to_std_(x);
            const std::vector<double> yd = sweep_();
            r.resize(n_);
            for (int i = 0; i < n_; ++i) r(i) = yd[(std::size_t)i];
        }

        /// Jacobian at x. The tape is valid at the point of the last
        /// residual() call; anywhere else it is judged first.
        void jacobian(const Eigen::VectorXd& x, Eigen::MatrixXd& J) const
        {
            const std::vector<double> xd = to_std_(x);
            if (!taped_ || !detail::same_point(xd, last_x_)) {
                last_x_ = xd;
                (void)sweep_();
            }
            const std::vector<double> jd = detail::tape_jacobian(tape_, xd, single_sweep_);
            ++jacobians_;
            J.resize(n_, n_);
            for (int rr = 0; rr < n_; ++rr)
                for (int cc = 0; cc < n_; ++cc)
                    J(rr, cc) = jd[(std::size_t)rr * n_ + (std::size_t)cc];
        }

        [[nodiscard]] int size() const noexcept { return n_; }

        /// Number of times the tape was re-recorded because a comparison on it
        /// went the other way at the point being evaluated.
        [[nodiscard]] int retape_count() const noexcept { return retapes_; }

        /// Take the Jacobian in one first-order sweep carrying every
        /// direction instead of one sweep per direction (off by default; see
        /// detail::tape_jacobian).
        void set_jacobian_single_sweep(bool on) noexcept { single_sweep_ = on; }

        /// Number of tape recordings (the first one and every re-recording).
        [[nodiscard]] int tape_count() const noexcept { return tapes_; }
        /// Number of residual evaluations (zero-order sweeps asked for).
        [[nodiscard]] int residual_count() const noexcept { return residuals_; }
        /// Number of Jacobian evaluations.
        [[nodiscard]] int jacobian_count() const noexcept { return jacobians_; }

    private:
        std::vector<double> to_std_(const Eigen::VectorXd& x) const
        {
            std::vector<double> v((std::size_t)n_);
            for (int i = 0; i < n_; ++i) v[(std::size_t)i] = x(i);
            return v;
        }

        // Zero-order sweep at last_x_, recording or re-recording as needed.
        std::vector<double> sweep_() const
        {
            if (!taped_) {
                tape_ = build_tape_(last_x_);
                taped_ = true;
                ++tapes_;
            }
            const int before = retapes_;
            std::vector<double> yd = detail::forward_or_retape(
                tape_, last_x_, [this](const std::vector<double>& at) { return build_tape_(at); }, retapes_);
            tapes_ += retapes_ - before;
            ++residuals_;
            return yd;
        }

        CppAD::ADFun<double> build_tape_(const std::vector<double>& at) const
        {
            using AD = CppAD::AD<double>;

            CppAD::vector<AD> ax((std::size_t)n_);
            for (int i = 0; i < n_; ++i) ax[(std::size_t)i] = AD(at[(std::size_t)i]);
            CppAD::Independent(ax);

            Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
            for (int i = 0; i < n_; ++i) x_ad(i) = ax[(std::size_t)i];
            const auto y_ad = functor_.template operator()<AD>(x_ad);

            CppAD::vector<AD> ay((std::size_t)n_);
            for (int i = 0; i < n_; ++i) ay[(std::size_t)i] = y_ad(i);

            CppAD::ADFun<double> f;
            f.Dependent(ax, ay);
            f.compare_change_count(1); // off by default: switch change tracking on
            return f;
        }

        Functor functor_;
        int n_{ 0 };
        mutable CppAD::ADFun<double> tape_;
        mutable bool taped_{ false };
        mutable int retapes_{ 0 };
        mutable int tapes_{ 0 };
        mutable int residuals_{ 0 };
        mutable int jacobians_{ 0 };
        mutable std::vector<double> last_x_;
        bool single_sweep_{ false };
    };

    // -------------------------------------------------------------------------
    // ParametricCppADResidualJacobian (dynamic size, tape kept across solves)
    //
    // The two evaluators above record a tape for one functor and are thrown
    // away with it. An implicit integrator builds a new stage residual for
    // every step -- another start state, another time, other controls -- so
    // the residual was recorded again on every step although, as a sequence of
    // operations, it had not changed.
    //
    // This evaluator is kept by its owner and given the functor of each solve
    // with bind(). The functor says which of its doubles change from solve to
    // solve (TapeParameterised, see TapeParameters.h): those are recorded as
    // CppAD dynamic parameters, and a bind() sets their values on the kept
    // tape (ADFun::new_dynamic) instead of recording again. The tape is
    // recorded again
    //
    //   - when the functor's constants are not bit for bit those the tape was
    //     recorded with (tapeConstantsIdentical), and
    //   - when a comparison recorded on it goes the other way at the point
    //     being evaluated -- on the unknowns or on a parameter, CppAD records
    //     both -- exactly as in the evaluators above (F01, N08;
    //     detail::forward_or_retape). A kept tape is judged by a zero-order
    //     sweep at the first point of every solve before anything is taken
    //     from it.
    //
    // A kept tape replays the operations a fresh recording would make, on the
    // same numbers: the parameter arithmetic CppAD folds into constants when
    // it records is done instead by new_dynamic, operation for operation. The
    // one difference is a product with a parameter that is exactly zero: as a
    // constant CppAD drops it (the result is the constant +0), as a dynamic
    // parameter it is computed (+0 or -0). The sign of an exact zero is the
    // only thing that can differ.
    //
    // Copying the evaluator copies nothing: the copy records its own tape.
    // -------------------------------------------------------------------------

    template <class Functor>
        requires TapeParameterised<Functor>
    // Rvalues use the cold-cache copy operations. Moving the tape separately
    // from its bound functor could invalidate recorded parameter addresses.
    // NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
    class ParametricCppADResidualJacobian final
    {
    public:
        explicit ParametricCppADResidualJacobian(int n)
            : n_(n)
        {
            if (n_ <= 0) throw std::invalid_argument(
                "ParametricCppADResidualJacobian: n must be > 0");
        }

        ParametricCppADResidualJacobian(const ParametricCppADResidualJacobian& other)
            : n_(other.n_)
        {}
        ParametricCppADResidualJacobian& operator=(const ParametricCppADResidualJacobian& other)
        {
            if (this != &other) {
                n_ = other.n_;
                functor_.reset();
                taped_ = false;
                last_x_.clear();
            }
            return *this;
        }

        /// @brief Make @p f the functor of the solve that follows.
        ///
        /// The kept tape is given the parameter values of @p f if the
        /// constants of @p f are those it was recorded with; otherwise it is
        /// dropped and recorded at the first point evaluated.
        void bind(const Functor& f)
        {
            const bool keep = taped_ && functor_.has_value() && functor_->tapeConstantsIdentical(f);
            functor_.emplace(f);
            params_.clear();
            functor_->forEachTapeParameter([this](const double& v) { params_.push_back(v); });
            if (keep && params_.size() == tape_.size_dyn_ind())
                tape_.new_dynamic(params_);
            else
                taped_ = false;
            last_x_.clear(); // nothing is taken from the tape before a zero-order sweep has judged it
        }

        void operator()(const Eigen::VectorXd& x,
            Eigen::VectorXd& r,
            Eigen::MatrixXd& J)
        {
            residual(x, r);
            jacobian(x, J);
        }

        /// Residual at x. This is the call that judges the tape: it is
        /// re-recorded here if a comparison on it changed at x.
        void residual(const Eigen::VectorXd& x, Eigen::VectorXd& r)
        {
            last_x_ = to_std_(x);
            const std::vector<double> yd = sweep_();
            r.resize(n_);
            for (int i = 0; i < n_; ++i) r(i) = yd[(std::size_t)i];
        }

        /// Jacobian at x. The tape is valid at the point of the last
        /// residual() call; anywhere else it is judged first.
        void jacobian(const Eigen::VectorXd& x, Eigen::MatrixXd& J)
        {
            const std::vector<double> xd = to_std_(x);
            if (!taped_ || !detail::same_point(xd, last_x_)) {
                last_x_ = xd;
                (void)sweep_();
            }
            const std::vector<double> jd = detail::tape_jacobian(tape_, xd, single_sweep_);
            ++jacobians_;
            J.resize(n_, n_);
            for (int rr = 0; rr < n_; ++rr)
                for (int cc = 0; cc < n_; ++cc)
                    J(rr, cc) = jd[(std::size_t)rr * n_ + (std::size_t)cc];
        }

        [[nodiscard]] int size() const noexcept { return n_; }

        /// Number of times the tape was re-recorded because a comparison on it
        /// went the other way at the point being evaluated.
        [[nodiscard]] int retape_count() const noexcept { return retapes_; }
        /// Take the Jacobian in one first-order sweep carrying every
        /// direction instead of one sweep per direction (off by default; see
        /// detail::tape_jacobian).
        void set_jacobian_single_sweep(bool on) noexcept { single_sweep_ = on; }

        /// Number of tape recordings: the first, one for every bind() that
        /// could not keep the tape, and every re-recording.
        [[nodiscard]] int tape_count() const noexcept { return tapes_; }
        /// Number of residual evaluations (zero-order sweeps asked for).
        [[nodiscard]] int residual_count() const noexcept { return residuals_; }
        /// Number of Jacobian evaluations.
        [[nodiscard]] int jacobian_count() const noexcept { return jacobians_; }
        /// Number of dynamic parameters of the functor bound last.
        [[nodiscard]] std::size_t parameter_count() const noexcept { return params_.size(); }

    private:
        std::vector<double> to_std_(const Eigen::VectorXd& x) const
        {
            if (x.size() != n_) throw std::invalid_argument(
                "ParametricCppADResidualJacobian: x has the wrong size");
            std::vector<double> v((std::size_t)n_);
            for (int i = 0; i < n_; ++i) v[(std::size_t)i] = x(i);
            return v;
        }

        // Zero-order sweep at last_x_, recording or re-recording as needed.
        std::vector<double> sweep_()
        {
            if (!functor_.has_value()) throw std::logic_error(
                "ParametricCppADResidualJacobian: bind() a functor before evaluating");
            if (!taped_) {
                tape_ = build_tape_(last_x_);
                taped_ = true;
                ++tapes_;
            }
            const int before = retapes_;
            std::vector<double> yd = detail::forward_or_retape(
                tape_, last_x_, [this](const std::vector<double>& at) { return build_tape_(at); }, retapes_);
            tapes_ += retapes_ - before;
            ++residuals_;
            return yd;
        }

        // Record functor_ at `at`, its parameters as dynamic parameters holding
        // their current values.
        CppAD::ADFun<double> build_tape_(const std::vector<double>& at) const
        {
            using AD = CppAD::AD<double>;

            detail::TapeParameterBinding binding;
            functor_->forEachTapeParameter([&binding](const double& v) { binding.addresses.push_back(&v); });
            binding.values.resize(binding.addresses.size());
            for (std::size_t i = 0; i < binding.addresses.size(); ++i) binding.values[i] = AD(*binding.addresses[i]);

            CppAD::vector<AD> ax((std::size_t)n_);
            for (int i = 0; i < n_; ++i) ax[(std::size_t)i] = AD(at[(std::size_t)i]);
            const std::size_t abort_op_index = 0;
            const bool record_compare = true;
            CppAD::Independent(ax, abort_op_index, record_compare, binding.values);

            Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
            for (int i = 0; i < n_; ++i) x_ad(i) = ax[(std::size_t)i];

            CppAD::vector<AD> ay((std::size_t)n_);
            {
                const TapeParameterScope scope(binding);
                const auto y_ad = functor_->template operator()<AD>(x_ad);
                for (int i = 0; i < n_; ++i) ay[(std::size_t)i] = y_ad(i);
            }

            CppAD::ADFun<double> f;
            f.Dependent(ax, ay);
            f.compare_change_count(1); // off by default: switch change tracking on
            return f;
        }

        std::optional<Functor> functor_;
        std::vector<double> params_;
        std::vector<double> last_x_;
        CppAD::ADFun<double> tape_;
        int n_{ 0 };
        int retapes_{ 0 };
        int tapes_{ 0 };
        int residuals_{ 0 };
        int jacobians_{ 0 };
        bool taped_{ false };
        bool single_sweep_{ false };
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
////  - CppAD tape is recorded at the first evaluation point and re-recorded
////    whenever a comparison recorded on it changes result.
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
