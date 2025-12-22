// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <type_traits>
#include <cmath>

#if __has_include(<cppad/cppad.hpp>)
#include <cppad/cppad.hpp>
#endif

namespace Aetherion::ODE::RKMK::Lie::Math {

    // -------------------------------------------------------------------------
    // Scalar traits
    // -------------------------------------------------------------------------
    template<class S>
    struct is_ad : std::false_type {};

#if __has_include(<cppad/cppad.hpp>)
    template<>
    struct is_ad<CppAD::AD<double>> : std::true_type {};
#endif

    template<class S>
    inline constexpr bool is_ad_v = is_ad<S>::value;

    // -------------------------------------------------------------------------
    // AD-friendly elementary functions (ADL finds CppAD overloads)
    // -------------------------------------------------------------------------
    template <class S>
    [[nodiscard]] inline S Sin(const S& x) { using std::sin; return sin(x); }

    template <class S>
    [[nodiscard]] inline S Cos(const S& x) { using std::cos; return cos(x); }

    template <class S>
    [[nodiscard]] inline S Sqrt(const S& x) { using std::sqrt; return sqrt(x); }

    template <class S>
    [[nodiscard]] inline S Abs(const S& x) { using std::abs; return abs(x); }

    template<class...>
    inline constexpr bool always_false_v = false;

} // namespace Aetherion::ODE::RKMK::Lie::Math
