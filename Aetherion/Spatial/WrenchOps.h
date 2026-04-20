// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// Aetherion/Spatial/WrenchOps.h
#pragma once

#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::Spatial {

/// @brief Construct a zero-initialised @c Wrench.
/// @tparam Scalar Numeric type.
/// @return Wrench with all six components set to zero.
    template<class Scalar>
    inline Wrench<Scalar> ZeroWrench()
    {
        Wrench<Scalar> w{};
        w.f.setZero();
        return w;
    }

/// @brief Accumulate a wrench in-place: @c acc += @c w.
/// @param acc  Accumulator wrench (modified in-place).
/// @param w    Wrench to add.
    template<class Scalar>
    inline void AddInPlace(Wrench<Scalar>& acc, const Wrench<Scalar>& w)
    {
        acc.f += w.f;
    }

} // namespace Aetherion::Spatial
