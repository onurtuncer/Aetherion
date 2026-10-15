// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_RadauBranchCrossing.cpp
//
// Regression for the pre-DOI audit's finding F01 at the integrator level.
//
// A vector field with a branch (here: dx/dt = 1 below x = 1.1, 2 above) is
// taped by the Radau IIA stepper's Newton evaluator. The tape used to be
// recorded once, at a zero stage increment, and every Newton iteration reused
// it: for a step that crosses the branch the iteration converged on the
// residual of the region the tape was recorded in, the integrator then
// computed the real residual (0.28 for this step) and reported the step as
// converged anyway. The evaluator now re-records its tape when a comparison
// on it changes result, and the stepper accepts a step on the real residual.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/RigidBody/KinematicsXiField.h>

namespace {
    using Catch::Approx;
    namespace Core = Aetherion::ODE::RKMK::Core;
    namespace Lie  = Aetherion::ODE::RKMK::Lie;
    namespace Int  = Aetherion::ODE::RKMK::Integrators;

    /// Euclidean field on the 7-vector: only x(6) evolves, at rate 1 below the
    /// switch value and 2 above it. The comparison is made on S, so CppAD
    /// records it when the field is taped.
    struct SwitchingField {
        double x_switch = 1.1;
        template<class S>
        Eigen::Matrix<S, 7, 1> operator()(S, const Lie::SE3<S>&, const Eigen::Matrix<S, 7, 1>& x) const
        {
            Eigen::Matrix<S, 7, 1> f = Eigen::Matrix<S, 7, 1>::Zero();
            if (x(6) < S(x_switch)) f(6) = S(1.0);
            else                    f(6) = S(2.0);
            return f;
        }
    };

    using Stepper = Int::RadauIIA_RKMK_ProductSE3<Aetherion::RigidBody::KinematicsXiField, SwitchingField, 7>;
}

TEST_CASE("RadauIIA product stepper: a step across a branch of the field is solved on the right branch",
          "[rkmk][radau][branch][regression]")
{
    Stepper stepper(Aetherion::RigidBody::KinematicsXiField{}, SwitchingField{});

    Eigen::Matrix<double, 7, 1> x0 = Eigen::Matrix<double, 7, 1>::Zero();
    x0(6) = 1.0;
    const auto g0 = Lie::SE3<double>::Identity();

    Core::NewtonOptions opt;
    opt.max_iters = 50;
    opt.abs_tol = 1e-12;
    opt.rel_tol = 1e-12;

    // Step length 0.2 from x = 1.0: the switch at 1.1 is reached after 0.1 s,
    // and the last two Radau stages (c = 0.64, 1.0) sit beyond it. The exact
    // solution is x(0.2) = 1.1 + 2 * 0.1 = 1.3.
    const auto res = stepper.step(0.0, g0, x0, 0.2, opt);

    INFO("residual " << res.residual_norm << ", x1 = " << res.x1(6) << ", iters " << res.iters);
    REQUIRE(res.converged);
    CHECK(res.residual_norm < 1e-9);          // was 0.28 with the frozen tape
    CHECK(res.x1(6) == Approx(1.3).margin(0.05));  // collocation across a jump: close, not exact
    CHECK(res.x1(6) > 1.2);                   // the frozen tape gave 1.2, the slope-1 answer
}

TEST_CASE("RadauIIA product stepper: a step that stays on one branch is unchanged",
          "[rkmk][radau][branch]")
{
    Stepper stepper(Aetherion::RigidBody::KinematicsXiField{}, SwitchingField{});

    Eigen::Matrix<double, 7, 1> x0 = Eigen::Matrix<double, 7, 1>::Zero();
    x0(6) = 1.0;
    const auto g0 = Lie::SE3<double>::Identity();

    Core::NewtonOptions opt;
    opt.abs_tol = 1e-12;
    opt.rel_tol = 1e-12;

    const auto res = stepper.step(0.0, g0, x0, 0.05, opt);   // reaches 1.05, below the switch
    REQUIRE(res.converged);
    CHECK(res.residual_norm < 1e-12);
    CHECK(res.x1(6) == Approx(1.05).margin(1e-12));
}
