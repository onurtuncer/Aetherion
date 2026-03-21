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

} // namespace Aetherion::Environment::detail