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

    template<class Scalar>
    inline Wrench<Scalar> ZeroWrench()
    {
        Wrench<Scalar> w{};
        w.f.setZero();
        return w;
    }

    template<class Scalar>
    inline void AddInPlace(Wrench<Scalar>& acc, const Wrench<Scalar>& w)
    {
        acc.f += w.f;
    }

} // namespace Aetherion::Spatial