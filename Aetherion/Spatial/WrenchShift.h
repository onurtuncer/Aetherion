// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#pragma once
#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::Spatial {

    // Shift a wrench applied at point P to an equivalent wrench at point Q
    // with r = r_PQ = (P - Q) expressed in the wrench frame.
    //
    // f_Q = [ M_P + r x F
    //         F     ]
    template<class Scalar>
    inline Wrench<Scalar> ShiftWrenchToNewPoint(const Wrench<Scalar>& f_at_P,
        const Eigen::Matrix<Scalar, 3, 1>& r_P_minus_Q)
    {
        Wrench<Scalar> out = f_at_P;
        const auto F = f_at_P.f.template segment<3>(3);
        out.f.template segment<3>(0) = f_at_P.f.template segment<3>(0) + r_P_minus_Q.cross(F);
        return out;
    }

} // namespace