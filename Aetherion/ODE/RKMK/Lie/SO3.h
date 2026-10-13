// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// SO3.h
//
// SO(3) exponential and left Jacobian via Rodrigues' formula.
//
// Refactored:
//   - SO3::Skew   removed; all call sites use Spatial::skew<S>() directly.
//     (Spatial::skew is AD-compatible and is the canonical skew operator in
//      Aetherion.  The old local copy had a self-acknowledged TODO.)
//
//   - SO3::Coeffs removed; A/B/C are now computed by Core::so3_ABC which is
//     the more accurate version (8-term series, CondExp-based AD branching).
//     The old Coeffs used if-constexpr on std::is_same_v<S,double> which
//     broke CppAD taping for the AD path.
//
// Public API:
//   SO3::Exp_R(w)                → 3×3 rotation matrix  (Rodrigues)
//   SO3::Log_R(R)                → rotation vector w with Exp_R(w) = R
//   SO3::LeftJacobian(w)         → 3×3 left Jacobian  J_l(w)
//   SO3::LeftJacobianInverse(w)  → J_l(w)^-1 in closed form
//
// Log_R and LeftJacobianInverse were added for the SE2(3) estimator
// (Estimation/SE23.h); both are AD-safe by the same CondExp switch as
// Core::so3_ABC.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <Aetherion/ODE/RKMK/Lie/Math.h>
#include <Aetherion/ODE/RKMK/Core/Scalar.h>   // Core::so3_ABC  (canonical A/B/C)
#include <Aetherion/Spatial/Skew.h>            // Spatial::skew<S>()

namespace Aetherion::ODE::RKMK::Lie::SO3 {

    // -------------------------------------------------------------------------
    // Exp_R(w)
    //
    // Rodrigues' rotation formula:
    //   R = I + A·[w]× + B·[w]×²
    //
    // where A, B are the standard SO(3) sinc-like coefficients computed by
    // Core::so3_ABC — AD-safe via CondExp and numerically stable near zero.
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 3>
        Exp_R(const Eigen::Matrix<S, 3, 1>& w)
    {
        const S theta2 = w.dot(w);
        S A{}, B{}, C{};
        Core::so3_ABC(theta2, A, B, C);
        (void)C;  // C is only needed for LeftJacobian

        const Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
        const Eigen::Matrix<S, 3, 3> W = Spatial::skew(w);
        const Eigen::Matrix<S, 3, 3> W2 = W * W;

        return I + A * W + B * W2;
    }

    // -------------------------------------------------------------------------
    // LeftJacobian(w)
    //
    // Left Jacobian of SO(3):
    //   J_l(w) = I + B·[w]× + C·[w]×²
    //
    // Used in SE3::Exp to propagate the translational part:
    //   p_exp = J_l(w) · v
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 3>
        LeftJacobian(const Eigen::Matrix<S, 3, 1>& w)
    {
        const S theta2 = w.dot(w);
        S A{}, B{}, C{};
        Core::so3_ABC(theta2, A, B, C);
        (void)A;  // A is only needed for Exp_R

        const Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
        const Eigen::Matrix<S, 3, 3> W = Spatial::skew(w);
        const Eigen::Matrix<S, 3, 3> W2 = W * W;

        return I + B * W + C * W2;
    }

    // -------------------------------------------------------------------------
    // LeftJacobianInverse(w)
    //
    //   J_l(w)^-1 = I - 1/2·[w]× + D·[w]×²,
    //   D = (1 - A/(2B)) / theta²  →  1/12 + theta²/720 + ... as theta → 0
    //
    // with A, B the coefficients of Core::so3_ABC. The series branch is
    // selected by CondExp below theta = 1e-4, like so3_ABC itself, so the
    // AD tape stays valid through the switch.
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 3>
        LeftJacobianInverse(const Eigen::Matrix<S, 3, 1>& w)
    {
        const S theta2 = w.dot(w);
        S A{}, B{}, C{};
        Core::so3_ABC(theta2, A, B, C);
        (void)C;

        const S eps2 = S(1e-8);
        // Keep the discarded branch free of 0/0: both branches are evaluated
        // under CondExp.
        const S theta2_safe = Core::condexp_lt(theta2, eps2, S(1), theta2);
        const S D_series    = S(1) / S(12) + theta2 / S(720);
        const S D_closed    = (S(1) - A / (S(2) * B)) / theta2_safe;
        const S D           = Core::condexp_lt(theta2, eps2, D_series, D_closed);

        const Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
        const Eigen::Matrix<S, 3, 3> W = Spatial::skew(w);
        const Eigen::Matrix<S, 3, 3> W2 = W * W;

        return I - S(0.5) * W + D * W2;
    }

    // -------------------------------------------------------------------------
    // Log_R(R)
    //
    // Rotation vector of R: theta = acos((tr R - 1)/2), axis from the
    // antisymmetric part, w = theta/(2 sin theta) · vee(R - Rᵀ). The series
    // 1 + sin²theta/6 replaces theta/sin(theta) below theta = 1e-4. Not
    // defined at theta = pi (sin theta = 0); the estimator only takes
    // logarithms of small errors, so that case is not handled.
    //
    // AD note: acos has an infinite derivative at the identity, so neither
    // the switch nor the series may be written in terms of theta -- even
    // theta² would put 0·inf on the tape. Both use sin²theta = |half_vee|²,
    // which is polynomial in R; theta only enters the closed-form branch,
    // which CondExp discards near the identity. CondExp discards the value,
    // not the evaluation: CppAD still differentiates acos in the dead
    // branch, and at c = 1 exactly that is a division by zero (UBSAN,
    // issue seen in CI on the process-model AD test). So c is clamped to
    // ±(1 - 1e-12), not ±1: theta then bottoms out at ~1.4e-6, far inside
    // the series region, and the dead branch's derivative is finite.
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 1>
        Log_R(const Eigen::Matrix<S, 3, 3>& R)
    {
        using std::acos;
        using std::sin;

        const Eigen::Matrix<S, 3, 1> half_vee{
            S(0.5) * (R(2, 1) - R(1, 2)),
            S(0.5) * (R(0, 2) - R(2, 0)),
            S(0.5) * (R(1, 0) - R(0, 1)) };  // = sin(theta) · axis
        const S s2 = half_vee.dot(half_vee);  // sin²(theta)

        // Clamp against round-off beyond [-1, 1] and away from the
        // singular derivative at ±1 (see the AD note); CondExp keeps it AD-safe.
        const S c_max = S(1.0 - 1e-12);
        S c = (R.trace() - S(1)) * S(0.5);
        c = Core::condexp_gt(c, c_max, c_max, c);
        c = Core::condexp_lt(c, -c_max, -c_max, c);
        const S theta = acos(c);

        const S eps2   = S(1e-8);
        const S s      = sin(theta);
        const S s_safe = Core::condexp_lt(s2, eps2, S(1), s);
        const S k      = Core::condexp_lt(s2, eps2, S(1) + s2 / S(6), theta / s_safe);

        return k * half_vee;
    }

} // namespace Aetherion::ODE::RKMK::Lie::SO3
