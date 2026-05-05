// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <cmath>

namespace Aetherion::Environment::detail {

    template <class S>
    inline S SquareRoot(const S& x) {
        using std::sqrt;
        return sqrt(x);
    }

    template <class S>
    inline S Power(const S& x, const S& y) {
        using std::pow;
        return pow(x, y);
    }

    template <class S>
    inline S Exponential(const S& x) {
        using std::exp;
        return exp(x);
    }

    template <class S>
    inline S Sine(const S& x) {
        using std::sin;
        return sin(x);
    }

    template <class S>
    inline S Cosine(const S& x) {
        using std::cos;
        return cos(x);
    }

    template <class S>
    inline S ArcTangent(const S& x) {
        using std::atan;
        return atan(x);
    }

    /// @brief Two-argument arctangent, AD-safe via ADL.
    ///
    /// For `CppAD::AD<double>` arguments, ADL finds `CppAD::atan2` because the
    /// arguments live in the `CppAD` namespace.  For `double`, `std::atan2` is used.
    template <class S>
    inline S ArcTangent2(const S& y, const S& x) {
        using std::atan2;
        return atan2(y, x);
    }

    template <class S>
    inline S ArcSine(const S& x) {
        using std::asin;
        return asin(x);
    }

} // namespace Aetherion::Environment::detail