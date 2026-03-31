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
// Public API (unchanged):
//   SO3::Exp_R(w)       → 3×3 rotation matrix  (Rodrigues)
//   SO3::LeftJacobian(w) → 3×3 left Jacobian  J_l(w)
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

} // namespace Aetherion::ODE::RKMK::Lie::SO3
