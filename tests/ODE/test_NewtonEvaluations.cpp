// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_NewtonEvaluations.cpp
//
// What the Newton loop asks of its evaluator, and the tape kept across solves.
//
//   - The loop evaluates each point once. It used to evaluate the starting
//     point twice, and every accepted line-search point twice.
//   - An evaluator that offers residual() and jacobian() separately is asked
//     for a Jacobian only where a Newton step is taken from it: a rejected
//     line-search trial costs none.
//   - NewtonOptions::report_step_norm = false drops the Jacobian that was
//     taken at the converged iterate only to report the step norm.
//   - None of this changes an iterate: every variant is compared, bit for
//     bit, with a loop written out here the way the old one worked.
//   - The single-sweep Jacobian is the dense driver's, bit for bit.
//   - ParametricCppADResidualJacobian: a tape kept across solves gives the
//     residual and Jacobian of a tape recorded for each solve, and is recorded
//     again when a comparison changes (on an unknown or on a parameter) and
//     when a constant of the functor changes.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Newton.h>
#include <Aetherion/ODE/RKMK/Core/TapeParameters.h>

namespace {
    namespace Core = Aetherion::ODE::RKMK::Core;
    using Core::TapeParameter;

    bool sameBits(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
    {
        return a.rows() == b.rows() && a.cols() == b.cols() && std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
    }

    // r(x) = [atan(x0) + 0.1 x1,  x1^3 + x1 - 1 + 0.2 sin(x0)]
    // From x0 = 3 the full Newton step overshoots, so the line search backtracks.
    void atanResidual(const Eigen::VectorXd& x, Eigen::VectorXd& r)
    {
        r.resize(2);
        r(0) = std::atan(x(0)) + 0.1 * x(1);
        r(1) = x(1) * x(1) * x(1) + x(1) - 1.0 + 0.2 * std::sin(x(0));
    }
    void atanJacobian(const Eigen::VectorXd& x, Eigen::MatrixXd& J)
    {
        J.resize(2, 2);
        J(0, 0) = 1.0 / (1.0 + x(0) * x(0));
        J(0, 1) = 0.1;
        J(1, 0) = 0.2 * std::cos(x(0));
        J(1, 1) = 3.0 * x(1) * x(1) + 1.0;
    }

    /// Evaluator called in full only, counting its calls.
    struct FullEvaluator
    {
        int calls = 0;
        void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J)
        {
            ++calls;
            atanResidual(x, r);
            atanJacobian(x, J);
        }
    };

    /// Evaluator offering its halves separately, counting each.
    struct SplitEvaluator
    {
        int residuals = 0, jacobians = 0, full = 0;
        std::vector<double> last;
        bool jacobian_elsewhere = false; // a Jacobian asked for away from the last residual point
        void residual(const Eigen::VectorXd& x, Eigen::VectorXd& r)
        {
            ++residuals;
            last.assign(x.data(), x.data() + x.size());
            atanResidual(x, r);
        }
        void jacobian(const Eigen::VectorXd& x, Eigen::MatrixXd& J)
        {
            ++jacobians;
            if (last != std::vector<double>(x.data(), x.data() + x.size())) jacobian_elsewhere = true;
            atanJacobian(x, J);
        }
        void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J)
        {
            ++full;
            atanResidual(x, r);
            atanJacobian(x, J);
        }
    };

    /// The loop as it was before the evaluations were counted: the residual and
    /// the Jacobian at the top of every iteration and at every trial point.
    /// Returns the iterates, for comparison.
    std::vector<Eigen::VectorXd> referenceIterates(Eigen::VectorXd x, const Core::NewtonOptions& opt, int& evaluations)
    {
        std::vector<Eigen::VectorXd> xs{ x };
        Eigen::VectorXd r, r_trial;
        Eigen::MatrixXd J, J_trial;
        evaluations = 0;
        auto eval = [&](const Eigen::VectorXd& at, Eigen::VectorXd& rr, Eigen::MatrixXd& JJ) {
            ++evaluations;
            atanResidual(at, rr);
            atanJacobian(at, JJ);
        };
        eval(x, r, J);
        const double r0 = r.norm();
        for (int k = 0; k < opt.max_iters; ++k) {
            eval(x, r, J);
            const double r_norm = r.norm();
            const Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-r);
            if (Core::newton_converged(r_norm, r0, dx.norm(), x.norm(), opt)) break;
            double lambda = opt.damping_init;
            for (int ls = 0; ls < opt.max_line_search_iters; ++ls) {
                const Eigen::VectorXd x_trial = x + lambda * dx;
                eval(x_trial, r_trial, J_trial);
                if (r_trial.norm() <= opt.sufficient_decrease * r_norm) {
                    x = x_trial;
                    break;
                }
                lambda *= opt.damping_shrink;
            }
            xs.push_back(x);
        }
        return xs;
    }

    Eigen::VectorXd start()
    {
        Eigen::VectorXd x(2);
        x << 3.0, 0.5;
        return x;
    }
}

TEST_CASE("NewtonSolve: each point is evaluated once, and the iterates are those of the loop that evaluated twice",
    "[newton][evaluations]")
{
    Core::NewtonOptions opt;
    opt.abs_tol = 1e-13;
    opt.rel_tol = 0.0;

    int reference_evaluations = 0;
    const auto ref = referenceIterates(start(), opt, reference_evaluations);

    FullEvaluator eval;
    Eigen::VectorXd x = start();
    const auto res = Core::NewtonSolve(eval, x, opt);

    REQUIRE(res.converged);
    REQUIRE(res.line_search_iters > res.iters - 1); // the line search did backtrack
    CHECK(sameBits(x, ref.back()));
    // One evaluation at the start and one per line-search trial; the old loop
    // made one more per iteration (reference_evaluations counts those).
    CHECK(eval.calls == 1 + res.line_search_iters);
    CHECK(reference_evaluations == 1 + res.iters + res.line_search_iters);
}

TEST_CASE("NewtonSolve: a split evaluator is asked for a Jacobian only where a step is taken", "[newton][evaluations]")
{
    Core::NewtonOptions opt;
    opt.abs_tol = 1e-13;
    opt.rel_tol = 0.0;

    FullEvaluator full;
    Eigen::VectorXd x_full = start();
    const auto res_full = Core::NewtonSolve(full, x_full, opt);

    SplitEvaluator split;
    Eigen::VectorXd x_split = start();
    const auto res_split = Core::NewtonSolve(split, x_split, opt);

    REQUIRE(res_split.converged);
    CHECK(sameBits(x_split, x_full));
    CHECK(res_split.iters == res_full.iters);
    CHECK(res_split.line_search_iters == res_full.line_search_iters);
    CHECK(res_split.r_norm == res_full.r_norm);
    CHECK(res_split.dx_norm == res_full.dx_norm);

    CHECK(split.full == 0);
    CHECK(split.residuals == 1 + res_split.line_search_iters); // every point once
    CHECK(split.jacobians == res_split.iters);                 // one per Newton step computed
    CHECK(split.jacobians < split.residuals);                  // the rejected trials cost none
    CHECK_FALSE(split.jacobian_elsewhere);
}

TEST_CASE("NewtonSolve: report_step_norm = false saves the Jacobian at the converged iterate and nothing else",
    "[newton][evaluations]")
{
    Core::NewtonOptions opt;
    opt.abs_tol = 1e-13;
    opt.rel_tol = 0.0;

    SplitEvaluator with;
    Eigen::VectorXd x_with = start();
    const auto res_with = Core::NewtonSolve(with, x_with, opt);

    opt.report_step_norm = false;
    SplitEvaluator without;
    Eigen::VectorXd x_without = start();
    const auto res_without = Core::NewtonSolve(without, x_without, opt);

    REQUIRE(res_with.converged);
    REQUIRE(res_without.converged);
    CHECK(sameBits(x_without, x_with));
    CHECK(res_without.iters == res_with.iters);
    CHECK(res_without.r_norm == res_with.r_norm);
    CHECK(std::isnan(res_without.dx_norm));
    CHECK(std::isfinite(res_with.dx_norm));
    CHECK(without.residuals == with.residuals);
    CHECK(without.jacobians == with.jacobians - 1);
}

TEST_CASE("NewtonSolveRJ: the callbacks are called as a split evaluator", "[newton][evaluations]")
{
    Core::NewtonOptions opt;
    opt.abs_tol = 1e-13;
    opt.rel_tol = 0.0;

    FullEvaluator full;
    Eigen::VectorXd x_full = start();
    const auto res_full = Core::NewtonSolve(full, x_full, opt);

    REQUIRE(res_full.converged);

    int residuals = 0, jacobians = 0;
    auto residual = [&](const Eigen::VectorXd& x, Eigen::VectorXd& r) {
        ++residuals;
        atanResidual(x, r);
    };
    auto jacobian = [&](const Eigen::VectorXd& x, Eigen::MatrixXd& J) {
        ++jacobians;
        atanJacobian(x, J);
    };
    Eigen::VectorXd x = start();
    const auto res = Core::NewtonSolveRJ(residual, jacobian, x, opt);

    REQUIRE(res.converged);
    CHECK(sameBits(x, x_full));
    CHECK(residuals == 1 + res.line_search_iters);
    CHECK(jacobians == res.iters);
}

// -- Taped evaluators -----------------------------------------------------------

namespace {

    /// A residual with the operations a flight model puts on a tape, a branch
    /// on an unknown, a branch on a parameter, two tape parameters and one
    /// constant.
    struct ParamFunctor
    {
        double gain = 1.5;    // tape parameter
        double offset = 0.25; // tape parameter
        double scale = 2.0;   // constant of the tape

        template <class S>
        Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const
        {
            using CppAD::atan;
            using CppAD::cos;
            using CppAD::exp;
            using CppAD::log;
            using CppAD::pow;
            using CppAD::sin;
            using CppAD::sqrt;
            using CppAD::tan;
            using std::atan;
            using std::cos;
            using std::exp;
            using std::log;
            using std::pow;
            using std::sin;
            using std::sqrt;
            using std::tan;

            const S g = TapeParameter<S>(gain);
            const S b = TapeParameter<S>(offset);
            Eigen::Matrix<S, Eigen::Dynamic, 1> r(3);
            r(0) = g * sin(x(0)) * cos(x(1)) + sqrt(x(2) * x(2) + S(1.0)) / (S(2.0) + x(0) * x(0)) - b;
            r(1) = exp(S(0.3) * x(1)) - log(S(3.0) + x(0) * x(0)) + atan(g * x(2)) + tan(S(0.1) * x(0));
            r(2) = pow(x(2) * x(2) + S(1.0), S(1.5)) * S(scale) - x(0) * x(1) * g;
            if (x(0) < S(1.0)) // a branch on an unknown
                r(2) += x(0);
            else
                r(2) += S(2.0) * x(0) - S(1.0);
            if (b < S(0.0)) // a branch on a parameter
                r(1) += b * x(1);
            return r;
        }

        template <class Visitor>
        // Invoke the same visitor as an lvalue throughout the traversal.
        // NOLINTNEXTLINE(cppcoreguidelines-missing-std-forward)
        void forEachTapeParameter(Visitor&& visit) const
        {
            visit(gain);
            visit(offset);
        }
        [[nodiscard]] bool tapeConstantsIdentical(const ParamFunctor& other) const noexcept
        {
            return Core::BitIdentical(scale, other.scale);
        }
    };

    /// The same residual with nothing said about parameters: every double is
    /// a constant of the tape, which is recorded for each functor.
    struct PlainFunctor
    {
        ParamFunctor f;
        template <class S>
        Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const
        {
            return f(x);
        }
    };

    static_assert(Core::TapeParameterised<ParamFunctor>);
    static_assert(!Core::TapeParameterised<PlainFunctor>);

    Eigen::VectorXd point(double a, double b, double c)
    {
        Eigen::VectorXd x(3);
        x << a, b, c;
        return x;
    }

    /// Residual and Jacobian of f at x from a tape recorded for this call alone.
    void fresh(const ParamFunctor& f, const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J)
    {
        const Core::CppADResidualJacobian<PlainFunctor> eval(PlainFunctor{ f }, 3);
        eval(x, r, J);
    }
}

TEST_CASE("TapeParameter: without a binding it is the value, for double and for AD<double>", "[newton][tape]")
{
    const double v = 0.75;
    CHECK(TapeParameter<double>(v) == 0.75);

    using AD = CppAD::AD<double>;
    const AD a = TapeParameter<AD>(v);
    CHECK(CppAD::Value(a) == 0.75);
    CHECK(CppAD::Constant(a)); // a constant, not a dynamic parameter and not a variable
}

TEST_CASE("Taped Jacobian: one sweep carrying all directions gives the dense driver's matrix, bit for bit",
    "[newton][cppad][jacobian]")
{
    ParamFunctor f;
    Core::CppADResidualJacobian<PlainFunctor> dense(PlainFunctor{ f }, 3);
    Core::CppADResidualJacobian<PlainFunctor> single(PlainFunctor{ f }, 3);
    single.set_jacobian_single_sweep(true);

    for (const Eigen::VectorXd& x : { point(0.3, -0.7, 1.9), point(-1.2, 0.4, -0.6), point(2.5, 1.1, 0.05), point(0.0, 0.0, 0.0) }) {
        Eigen::VectorXd r_dense, r_single;
        Eigen::MatrixXd J_dense, J_single;
        dense(x, r_dense, J_dense);
        single(x, r_single, J_single);
        CHECK(sameBits(r_single, r_dense));
        CHECK(sameBits(J_single, J_dense));
        CHECK(J_dense.norm() > 0.1);
    }
}

TEST_CASE("ParametricCppADResidualJacobian: a kept tape gives what a tape recorded for each solve gives",
    "[newton][cppad][tape]")
{
    Core::ParametricCppADResidualJacobian<ParamFunctor> kept(3);
    CHECK(kept.parameter_count() == 0);

    ParamFunctor f;
    const Eigen::VectorXd x = point(0.3, -0.7, 1.9);
    Eigen::VectorXd r, r_ref;
    Eigen::MatrixXd J, J_ref;

    // Twelve solves, each with other parameter values and another point, all
    // on the branches the tape was recorded on.
    for (int k = 0; k < 12; ++k) {
        f.gain = 1.5 + 0.11 * k;
        f.offset = 0.25 + 0.03 * k;
        const Eigen::VectorXd xk = x + point(0.01 * k, -0.02 * k, 0.005 * k);
        kept.bind(f);
        kept(xk, r, J);
        fresh(f, xk, r_ref, J_ref);
        INFO("solve " << k);
        CHECK(sameBits(r, r_ref));
        CHECK(sameBits(J, J_ref));
    }
    CHECK(kept.parameter_count() == 2);
    CHECK(kept.tape_count() == 1); // recorded once, for twelve functors
    CHECK(kept.retape_count() == 0);
}

TEST_CASE("ParametricCppADResidualJacobian: a comparison that changes re-records the kept tape",
    "[newton][cppad][tape][regression]")
{
    Core::ParametricCppADResidualJacobian<ParamFunctor> kept(3);
    ParamFunctor f;
    Eigen::VectorXd r, r_ref;
    Eigen::MatrixXd J, J_ref;

    kept.bind(f);
    kept(point(0.3, -0.7, 1.9), r, J); // recorded with x0 < 1 and offset >= 0
    REQUIRE(kept.tape_count() == 1);

    // A later solve on the other branch of the unknown: F01 across solves.
    const Eigen::VectorXd x_far = point(2.5, -0.7, 1.9);
    kept.bind(f);
    kept(x_far, r, J);
    fresh(f, x_far, r_ref, J_ref);
    CHECK(sameBits(r, r_ref));
    CHECK(sameBits(J, J_ref));
    CHECK(kept.retape_count() == 1);

    // A later solve whose PARAMETER lies on the other branch, at the same point.
    f.offset = -0.4;
    kept.bind(f);
    kept(x_far, r, J);
    fresh(f, x_far, r_ref, J_ref);
    CHECK(sameBits(r, r_ref));
    CHECK(sameBits(J, J_ref));
    CHECK(kept.retape_count() == 2);

    // The Jacobian alone, asked for first: the tape is judged before it is used.
    f.offset = 0.4;
    kept.bind(f);
    kept.jacobian(x_far, J);
    fresh(f, x_far, r_ref, J_ref);
    CHECK(sameBits(J, J_ref));
    CHECK(kept.retape_count() == 3);
}

TEST_CASE("ParametricCppADResidualJacobian: a changed constant re-records the tape", "[newton][cppad][tape]")
{
    Core::ParametricCppADResidualJacobian<ParamFunctor> kept(3);
    ParamFunctor f;
    const Eigen::VectorXd x = point(0.3, -0.7, 1.9);
    Eigen::VectorXd r, r_ref;
    Eigen::MatrixXd J, J_ref;

    kept.bind(f);
    kept(x, r, J);
    REQUIRE(kept.tape_count() == 1);

    f.scale = 2.5; // not a tape parameter: the kept tape has 2.0 on it
    kept.bind(f);
    kept(x, r, J);
    fresh(f, x, r_ref, J_ref);
    CHECK(sameBits(r, r_ref));
    CHECK(sameBits(J, J_ref));
    CHECK(kept.tape_count() == 2);
    CHECK(kept.retape_count() == 0); // a new recording, not a comparison that changed
}

namespace {
    /// ParamFunctor that claims its constants never differ. This is the
    /// mistake the tapeConstantsIdentical() contract exists to prevent.
    struct ForgetfulFunctor
    {
        ParamFunctor f;
        template <class S>
        Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const
        {
            return f(x);
        }
        template <class Visitor>
        // Invoke the same visitor as an lvalue throughout the traversal.
        // NOLINTNEXTLINE(cppcoreguidelines-missing-std-forward)
        void forEachTapeParameter(Visitor&& visit) const
        {
            f.forEachTapeParameter(visit);
        }
        [[nodiscard]] static bool tapeConstantsIdentical(const ForgetfulFunctor& /*other*/) noexcept { return true; }
    };
}

TEST_CASE("ParametricCppADResidualJacobian: a functor that misreports its constants gets the stale tape",
    "[newton][cppad][tape]")
{
    // The perturbation that shows the previous test can fail: with the
    // constant check answered "identical" regardless, the kept tape goes on
    // holding the old constant.
    Core::ParametricCppADResidualJacobian<ForgetfulFunctor> kept(3);
    ForgetfulFunctor f;
    const Eigen::VectorXd x = point(0.3, -0.7, 1.9);
    Eigen::VectorXd r_old, r_stale, r_ref;
    Eigen::MatrixXd J, J_ref;

    kept.bind(f);
    kept(x, r_old, J);

    f.f.scale = 2.5;
    kept.bind(f);
    kept(x, r_stale, J);
    fresh(f.f, x, r_ref, J_ref);

    CHECK(kept.tape_count() == 1);
    CHECK(sameBits(r_stale, r_old));       // the residual of the old constant
    CHECK_FALSE(sameBits(r_stale, r_ref)); // not that of the functor bound
}

TEST_CASE("ParametricCppADResidualJacobian: a copy records its own tape", "[newton][cppad][tape]")
{
    Core::ParametricCppADResidualJacobian<ParamFunctor> kept(3);
    ParamFunctor f;
    const Eigen::VectorXd x = point(0.3, -0.7, 1.9);
    Eigen::VectorXd r, r_copy;
    Eigen::MatrixXd J, J_copy;
    kept.bind(f);
    kept(x, r, J);

    Core::ParametricCppADResidualJacobian<ParamFunctor> copy(kept);
    CHECK(copy.tape_count() == 0);
    CHECK_THROWS_AS(copy(x, r_copy, J_copy), std::logic_error); // nothing bound yet
    copy.bind(f);
    copy(x, r_copy, J_copy);
    CHECK(copy.tape_count() == 1);
    CHECK(sameBits(r_copy, r));
    CHECK(sameBits(J_copy, J));
}

namespace {
    /// ParamFunctor minus a target: r(x) = f(x) - c, with c three more tape
    /// parameters, so that the root is known (c = f(x*)) and moves with them.
    struct RootedFunctor
    {
        ParamFunctor f;
        double c0 = 0.0, c1 = 0.0, c2 = 0.0;

        template <class S>
        Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const
        {
            Eigen::Matrix<S, Eigen::Dynamic, 1> r = f(x);
            r(0) -= TapeParameter<S>(c0);
            r(1) -= TapeParameter<S>(c1);
            r(2) -= TapeParameter<S>(c2);
            return r;
        }
        template <class Visitor>
        // Invoke the same visitor as an lvalue throughout the traversal.
        // NOLINTNEXTLINE(cppcoreguidelines-missing-std-forward)
        void forEachTapeParameter(Visitor&& visit) const
        {
            f.forEachTapeParameter(visit);
            visit(c0);
            visit(c1);
            visit(c2);
        }
        [[nodiscard]] bool tapeConstantsIdentical(const RootedFunctor& other) const noexcept
        {
            return f.tapeConstantsIdentical(other.f);
        }
    };

    struct PlainRootedFunctor
    {
        RootedFunctor f;
        template <class S>
        Eigen::Matrix<S, Eigen::Dynamic, 1> operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const
        {
            return f(x);
        }
    };
}

TEST_CASE("NewtonSolve on a kept tape follows the root as the parameters move", "[newton][cppad][tape]")
{
    Core::ParametricCppADResidualJacobian<RootedFunctor> kept(3);
    Core::NewtonOptions opt;
    opt.abs_tol = 1e-11;
    opt.rel_tol = 0.0;
    opt.max_iters = 50;

    RootedFunctor f;
    Eigen::VectorXd x = point(0.3, -0.7, 0.5);
    for (int k = 0; k < 10; ++k) {
        f.f.gain = 1.5 + 0.2 * k;
        f.f.offset = 0.25 - 0.1 * k; // crosses zero: the parameter branch flips on the way
        // The root moves from x0 = 0.4 to 1.3: across the branch on the unknown.
        const Eigen::VectorXd x_star = point(0.4 + 0.1 * k, -0.6, 0.5 + 0.05 * k);
        const Eigen::VectorXd c = f.f(x_star);
        f.c0 = c(0);
        f.c1 = c(1);
        f.c2 = c(2);

        Eigen::VectorXd x_kept = x;
        kept.bind(f);
        const auto res_kept = Core::NewtonSolve(kept, x_kept, opt);

        Eigen::VectorXd x_fresh = x;
        const Core::CppADResidualJacobian<PlainRootedFunctor> eval(PlainRootedFunctor{ f }, 3);
        const auto res_fresh = Core::NewtonSolve(eval, x_fresh, opt);

        INFO("solve " << k);
        REQUIRE(res_fresh.converged);
        REQUIRE(res_kept.converged);
        CHECK(res_kept.iters == res_fresh.iters);
        CHECK(sameBits(x_kept, x_fresh));
        x = x_fresh;
    }
    CHECK(kept.parameter_count() == 5);
    CHECK(kept.tape_count() < 10);
    CHECK(kept.retape_count() >= 2); // the unknown's branch and the parameter's
}
