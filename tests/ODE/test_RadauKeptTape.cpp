// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_RadauKeptTape.cpp
//
// The Radau IIA product stepper keeps the AD tape of its stage residual from
// one step to the next when the Euclidean field declares its tape parameters
// (ODE/RKMK/Core/TapeParameters.h). These tests hold the kept tape to the one
// thing that matters: a step computes what a step with a tape recorded for it
// alone computes, bit for bit --
//
//   - along a trajectory, with the field's parameter changed between steps;
//   - across a branch of the field reached by the state (finding F01, now
//     between steps and not only between Newton iterations);
//   - across a branch reached by a PARAMETER of the field;
//   - when a constant of the field changes.
//
// The reference is the same field wrapped so that it declares nothing: the
// stepper then records a tape on every step, every double a constant on it,
// which is what it did before tapes were kept.
//
// The last test is the perturbation: a field that changes a double it
// declared neither parameter nor constant leaves a stale tape behind, and the
// stepper's check of the real residual refuses the step.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Core/TapeParameters.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/RigidBody/KinematicsXiField.h>

namespace {
    namespace Core = Aetherion::ODE::RKMK::Core;
    namespace Lie = Aetherion::ODE::RKMK::Lie;
    namespace Int = Aetherion::ODE::RKMK::Integrators;
    using Core::TapeParameter;
    using SE3d = Lie::SE3<double>;
    using Vec7 = Eigen::Matrix<double, 7, 1>;

    /// A nonlinear field on the twist and the 7th state, with
    ///   - `drive`, a tape parameter (changed between steps);
    ///   - `x_switch`, a tape parameter that a branch on the state compares with;
    ///   - `drive_limit`, a tape parameter that a branch on `drive` compares with;
    ///   - `damping`, a constant of the tape.
    struct ParamField
    {
        double drive = 0.4;
        double x_switch = 1.1;
        double drive_limit = 10.0;
        double damping = 0.3;

        template <class S>
        Eigen::Matrix<S, 7, 1> operator()(S t, const Lie::SE3<S>& g, const Eigen::Matrix<S, 7, 1>& x) const
        {
            using CppAD::sin;
            using std::sin;
            const S u = TapeParameter<S>(drive);
            const S sw = TapeParameter<S>(x_switch);
            const S lim = TapeParameter<S>(drive_limit);

            Eigen::Matrix<S, 7, 1> f;
            // Euler-like coupling of the body rates, driven and damped.
            f(0) = S(0.7) * x(1) * x(2) - S(damping) * x(0) + u;
            f(1) = S(-0.5) * x(2) * x(0) - S(damping) * x(1) + u * sin(t);
            f(2) = S(0.2) * x(0) * x(1) - S(damping) * x(2);
            // Velocity pulled by the pose, so the SE(3) part is on the tape too.
            f(3) = -S(damping) * x(3) + S(0.1) * g.p(1);
            f(4) = -S(damping) * x(4) + g.R(0, 1);
            f(5) = -S(damping) * x(5) + u * x(6);
            // The 7th state: rate 1 below the switch, 2 above (as in
            // test_RadauBranchCrossing.cpp), saturating drive on top.
            if (x(6) < sw)
                f(6) = S(1.0);
            else
                f(6) = S(2.0);
            if (u < lim)
                f(6) += S(0.1) * u;
            else
                f(6) += S(0.1) * lim;
            return f;
        }

        template <class Visitor>
        // Invoke the same visitor as an lvalue throughout the traversal.
        // NOLINTNEXTLINE(cppcoreguidelines-missing-std-forward)
        void forEachTapeParameter(Visitor&& visit) const
        {
            visit(drive);
            visit(x_switch);
            visit(drive_limit);
        }
        [[nodiscard]] bool tapeConstantsIdentical(const ParamField& other) const noexcept
        {
            return Core::BitIdentical(damping, other.damping);
        }
    };

    /// ParamField declaring nothing: recorded on every step, as before.
    struct PlainField
    {
        ParamField f;
        template <class S>
        Eigen::Matrix<S, 7, 1> operator()(S t, const Lie::SE3<S>& g, const Eigen::Matrix<S, 7, 1>& x) const
        {
            return f(t, g, x);
        }
    };

    /// ParamField that says nothing about `damping`: neither a parameter nor
    /// compared as a constant.
    struct ForgetfulField
    {
        ParamField f;
        template <class S>
        Eigen::Matrix<S, 7, 1> operator()(S t, const Lie::SE3<S>& g, const Eigen::Matrix<S, 7, 1>& x) const
        {
            return f(t, g, x);
        }
        template <class Visitor>
        // Invoke the same visitor as an lvalue throughout the traversal.
        // NOLINTNEXTLINE(cppcoreguidelines-missing-std-forward)
        void forEachTapeParameter(Visitor&& visit) const
        {
            f.forEachTapeParameter(visit);
        }
        [[nodiscard]] static bool tapeConstantsIdentical(const ForgetfulField& /*other*/) noexcept { return true; }
    };

    using Xi = Aetherion::RigidBody::KinematicsXiField;
    using KeptStepper = Int::RadauIIA_RKMK_ProductSE3<Xi, ParamField, 7>;
    using PlainStepper = Int::RadauIIA_RKMK_ProductSE3<Xi, PlainField, 7>;
    using ForgetfulStepper = Int::RadauIIA_RKMK_ProductSE3<Xi, ForgetfulField, 7>;

    static_assert(KeptStepper::kKeepsTape);
    static_assert(!PlainStepper::kKeepsTape);

    Core::NewtonOptions tight()
    {
        Core::NewtonOptions opt;
        opt.max_iters = 50;
        opt.abs_tol = 1e-12;
        opt.rel_tol = 1e-12;
        return opt;
    }

    Vec7 startState()
    {
        Vec7 x;
        x << 0.3, -0.2, 0.5, 1.0, 0.2, -0.4, 0.2;
        return x;
    }

    template <class A, class B>
    bool sameStep(const A& a, const B& b)
    {
        using Aetherion::ODE::RKMK::Core::BitIdentical;
        return BitIdentical(a.x1, b.x1)
            && BitIdentical(a.g1.R, b.g1.R)
            && BitIdentical(a.g1.p, b.g1.p)
            && a.iters == b.iters
            && a.converged == b.converged;
    }
}

TEST_CASE("RadauIIA product stepper: a kept tape steps as a tape recorded for every step does",
    "[rkmk][radau][tape]")
{
    ParamField field;
    KeptStepper kept(Xi{}, field);

    SE3d g = SE3d::Identity();
    Vec7 x = startState();
    const double h = 0.05;
    int tapes = 0, steps = 0;

    // 60 steps: the drive changes on every one, the state crosses the switch
    // (x6 reaches 1.1 after about 0.9 s) and the drive crosses its limit.
    for (int k = 0; k < 60; ++k) {
        field.drive = 0.4 + 0.3 * std::sin(0.37 * k);
        field.drive_limit = (k >= 30 && k < 40) ? 0.2 : 10.0;
        kept.setVectorField(field);
        const PlainStepper plain(Xi{}, PlainField{ field });

        const auto a = kept.step(k * h, g, x, h, tight());
        const auto b = plain.step(k * h, g, x, h, tight());
        INFO("step " << k << ", x6 = " << b.x1(6));
        REQUIRE(b.converged);
        REQUIRE(sameStep(a, b));
        CHECK(b.tapes_recorded >= 1); // the reference records on every step
        tapes += a.tapes_recorded;
        ++steps;
        g = b.g1;
        x = b.x1;
    }
    CHECK(x(6) > 1.1); // the switch was crossed
    // Recorded at the first step and again where a comparison changed: the
    // state crossing the switch, the drive crossing its limit both ways.
    CHECK(tapes >= 3);
    CHECK(tapes < steps / 4);
}

TEST_CASE("RadauIIA product stepper: a step across a branch is solved on the right branch with a kept tape",
    "[rkmk][radau][branch][tape][regression]")
{
    ParamField field;
    field.drive = 0.0;
    KeptStepper kept(Xi{}, field);

    const auto g0 = SE3d::Identity();
    Vec7 x0 = Vec7::Zero();
    x0(6) = 1.0;

    // A first step that stays below the switch records the tape there ...
    const auto first = kept.step(0.0, g0, x0, 0.05, tight());
    REQUIRE(first.converged);
    CHECK(first.tapes_recorded == 1);
    CHECK(first.x1(6) < 1.1);

    // ... and the step of test_RadauBranchCrossing.cpp, from the same state,
    // crosses it: x(0.2) = 1.1 + 2 * 0.1 = 1.3, not the 1.2 of the slope-1 tape.
    const auto res = kept.step(0.0, g0, x0, 0.2, tight());
    REQUIRE(res.converged);
    CHECK(res.tapes_recorded >= 1);
    CHECK(res.residual_norm < 1e-9);
    CHECK(res.x1(6) > 1.2);

    // The same crossing reached by moving the switch instead of the state.
    field.x_switch = 1.02;
    kept.setVectorField(field);
    const auto moved = kept.step(0.0, g0, x0, 0.05, tight());
    const PlainStepper plain(Xi{}, PlainField{ field });
    const auto ref = plain.step(0.0, g0, x0, 0.05, tight());
    REQUIRE(ref.converged);
    CHECK(sameStep(moved, ref));
    CHECK(moved.x1(6) > 1.06); // 1.05 on the slope-1 tape
}

TEST_CASE("RadauIIA product stepper: a changed constant of the field re-records the kept tape", "[rkmk][radau][tape]")
{
    ParamField field;
    KeptStepper kept(Xi{}, field);
    const auto g0 = SE3d::Identity();
    const Vec7 x0 = startState();

    REQUIRE(kept.step(0.0, g0, x0, 0.05, tight()).converged);
    const auto again = kept.step(0.0, g0, x0, 0.05, tight());
    CHECK(again.tapes_recorded == 0);

    field.damping = 0.9;
    kept.setVectorField(field);
    const auto res = kept.step(0.0, g0, x0, 0.05, tight());
    const PlainStepper plain(Xi{}, PlainField{ field });
    const auto ref = plain.step(0.0, g0, x0, 0.05, tight());
    REQUIRE(ref.converged);
    CHECK(res.tapes_recorded == 1);
    CHECK(sameStep(res, ref));
}

TEST_CASE("RadauIIA product stepper: a copy of the stepper does not share the kept tape", "[rkmk][radau][tape]")
{
    ParamField field;
    KeptStepper kept(Xi{}, field);
    const auto g0 = SE3d::Identity();
    const Vec7 x0 = startState();
    const auto first = kept.step(0.0, g0, x0, 0.05, tight());

    // Exercise the copy constructor and its independent tape cache.
    const KeptStepper copy(kept); // NOLINT(performance-unnecessary-copy-initialization)
    const auto from_copy = copy.step(0.0, g0, x0, 0.05, tight());
    CHECK(from_copy.tapes_recorded == 1);
    CHECK(sameStep(from_copy, first));
}

TEST_CASE("RadauIIA product stepper: a stale tape is refused on the real residual", "[rkmk][radau][tape]")
{
    // The perturbation: a field that changes a double it declared neither a
    // parameter nor a constant. The tape is kept, Newton converges on the
    // residual of the old field, and the step is not accepted, because the
    // stepper judges it on the residual of the field it was given.
    ForgetfulField field;
    ForgetfulStepper kept(Xi{}, field);
    const auto g0 = SE3d::Identity();
    const Vec7 x0 = startState();
    REQUIRE(kept.step(0.0, g0, x0, 0.05, tight()).converged);

    field.f.damping = 0.9;
    kept.setVectorField(field);
    const auto res = kept.step(0.0, g0, x0, 0.05, tight());
    CHECK(res.tapes_recorded == 0);
    CHECK_FALSE(res.converged);
    CHECK(res.residual_norm > 1e-6);
}
