// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <type_traits>
#include <limits>
#include <cmath>

#include <cppad/cppad.hpp>

namespace Aetherion::ODE::RKMK::Core {

    // -------------------- CppAD scalar detection --------------------

    template<class T>
    struct is_cppad_ad : std::false_type {};

    template<>
    struct is_cppad_ad<CppAD::AD<double>> : std::true_type {};

    template<class T>
    inline constexpr bool is_cppad_ad_v = is_cppad_ad<std::decay_t<T>>::value;

    // -------------------- AD-friendly elementary functions --------------------
    // (Use CppAD versions for AD, std for plain doubles.)

    template<class S>
    [[nodiscard]] inline S sin_s(const S& x) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::sin(x);
        else return std::sin(x);
    }

    template<class S>
    [[nodiscard]] inline S cos_s(const S& x) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::cos(x);
        else return std::cos(x);
    }

    template<class S>
    [[nodiscard]] inline S sqrt_s(const S& x) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::sqrt(x);
        else return std::sqrt(x);
    }

    template<class S>
    [[nodiscard]] inline S abs_s(const S& x) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::abs(x);
        else return std::abs(x);
    }

    // -------------------- AD-friendly conditional expression --------------------
    // For AD, use CondExp (keeps graph differentiable piecewise).
    // For double, normal branch is fine.

    template<class S>
    [[nodiscard]] inline S condexp_lt(const S& lhs, const S& rhs, const S& t, const S& f) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::CondExpLt(lhs, rhs, t, f);
        else return (lhs < rhs) ? t : f;
    }

    template<class S>
    [[nodiscard]] inline S condexp_le(const S& lhs, const S& rhs, const S& t, const S& f) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::CondExpLe(lhs, rhs, t, f);
        else return (lhs <= rhs) ? t : f;
    }

    template<class S>
    [[nodiscard]] inline S condexp_gt(const S& lhs, const S& rhs, const S& t, const S& f) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::CondExpGt(lhs, rhs, t, f);
        else return (lhs > rhs) ? t : f;
    }

    template<class S>
    [[nodiscard]] inline S condexp_ge(const S& lhs, const S& rhs, const S& t, const S& f) {
        if constexpr (is_cppad_ad_v<S>) return CppAD::CondExpGe(lhs, rhs, t, f);
        else return (lhs >= rhs) ? t : f;
    }

    // -------------------- Small-angle safe SO(3) coefficients --------------------
    // Given theta2 = ||w||^2, compute:
    //   A = sin(theta)/theta
    //   B = (1 - cos(theta))/theta^2
    //   C = (theta - sin(theta))/theta^3
    //
    // Uses series expansions for theta^2 < eps^2 (AD-safe via CondExp).
    //
    // These are used in:
    //   Exp(w) = I + A [w] + B [w]^2
    //   J(w)   = I + B [w] + C [w]^2   (left Jacobian)
    //
    // Note: eps2 is chosen in double-space; for AD it becomes a constant threshold.
    //
    template<class S>
    inline void so3_ABC(const S& theta2, S& A, S& B, S& C) {
        // Switch earlier to avoid catastrophic cancellation in (1 - cos(theta))/theta2
        // theta2 < 1e-8  <=> theta < 1e-4  -> series is excellent there.
        const S eps2 = S(1e-8);

        const S t2 = theta2;
        const S t4 = t2 * t2;
        const S t6 = t4 * t2;
        const S t8 = t4 * t4;

        const S A_ser = S(1) - t2 / S(6) + t4 / S(120) - t6 / S(5040) + t8 / S(362880);
        const S B_ser = S(1) / S(2) - t2 / S(24) + t4 / S(720) - t6 / S(40320) + t8 / S(3628800);
        const S C_ser = S(1) / S(6) - t2 / S(120) + t4 / S(5040) - t6 / S(362880) + t8 / S(39916800);

        const S theta = sqrt_s(theta2);

        // A is ok as sin(theta)/theta
        const S A_trig = sin_s(theta) / theta;

        // B: use stable half-angle identity to avoid cancellation:
        // 1 - cos(theta) = 2 sin^2(theta/2)
        const S half = theta / S(2);
        const S sh = sin_s(half);
        const S B_trig = (S(2) * sh * sh) / theta2;

        // C: use C = (1 - A)/theta^2 (works well once theta is not tiny)
        const S C_trig = (S(1) - A_trig) / theta2;

        A = condexp_lt(theta2, eps2, A_ser, A_trig);
        B = condexp_lt(theta2, eps2, B_ser, B_trig);
        C = condexp_lt(theta2, eps2, C_ser, C_trig);
    }
} // namespace Aetherion::ODE::RKMK::Core
