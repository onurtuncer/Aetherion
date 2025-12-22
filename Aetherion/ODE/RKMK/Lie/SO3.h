// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <type_traits>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Aetherion/ODE/RKMK/Lie/Math.h>

namespace Aetherion::ODE::RKMK::Lie::SO3 {

    // -------------------------------------------------------------------------
    // Skew (hat) operator
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 3> Skew(const Eigen::Matrix<S, 3, 1>& w) {
        Eigen::Matrix<S, 3, 3> W;
        W << S(0), -w(2), w(1),
            w(2), S(0), -w(0),
            -w(1), w(0), S(0);
        return W;
    }

    // -------------------------------------------------------------------------
    // Coefficients for Rodrigues + Jacobians
    //
    // A = sin(theta)/theta
    // B = (1-cos(theta))/theta^2
    // C = (theta - sin(theta))/theta^3
    //
    // IMPORTANT:
    //  - For AD types: NO branching on theta using CppAD::Value.
    //    Use series expansions unconditionally (analytic, AD-safe).
    //  - For double: use a small-angle branch for numeric robustness.
    // -------------------------------------------------------------------------
    template <class S>
    inline void Coeffs(const Eigen::Matrix<S, 3, 1>& w, S& A, S& B, S& C) {
        const S theta2 = w.dot(w);

        auto series = [&]() {
            const S t2 = theta2;
            const S t4 = t2 * t2;
            const S t6 = t4 * t2;

            A = S(1) - t2 / S(6) + t4 / S(120) - t6 / S(5040);
            B = S(0.5) - t2 / S(24) + t4 / S(720) - t6 / S(40320);
            C = S(S(1) / S(6)) - t2 / S(120) + t4 / S(5040) - t6 / S(362880);
            };

        if constexpr (std::is_same_v<S, double>) {
            const double th2 = theta2;
            const double eps2 = 1e-16; // (1e-8)^2
            if (th2 < eps2) {
                series();
            }
            else {
                const double th = std::sqrt(th2);
                A = std::sin(th) / th;
                B = (1.0 - std::cos(th)) / th2;
                C = (th - std::sin(th)) / (th2 * th);
            }
        }
        else {
            // AD-safe path
            series();
        }
    }

    // -------------------------------------------------------------------------
    // Rotation matrix exponential: Exp([w]x) = I + A W + B W^2
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 3> Exp_R(const Eigen::Matrix<S, 3, 1>& w) {
        const Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
        const auto W = Skew(w);
        const auto W2 = W * W;

        S A, B, C;
        Coeffs(w, A, B, C);

        return I + A * W + B * W2;
    }

    // -------------------------------------------------------------------------
    // Left Jacobian J_l(w) = I + B W + C W^2
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline Eigen::Matrix<S, 3, 3> LeftJacobian(const Eigen::Matrix<S, 3, 1>& w) {
        const Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
        const auto W = Skew(w);
        const auto W2 = W * W;

        S A, B, C;
        Coeffs(w, A, B, C);

        return I + B * W + C * W2;
    }

} // namespace Aetherion::ODE::RKMK::Lie::SO3
