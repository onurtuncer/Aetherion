// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::Spatial {

/// @brief Shift a wrench from application point P to point Q.
///
/// Re-expresses a wrench that acts at P as an equivalent wrench at Q:
/// @f[
///   \begin{bmatrix} M_Q \\ F_Q \end{bmatrix} =
///   \begin{bmatrix} M_P + \mathbf{r}_{PQ} \times F_P \\ F_P \end{bmatrix}
/// @f]
/// where @f$\mathbf{r}_{PQ} = P - Q@f$ (expressed in the wrench frame).
///
/// @param f_at_P       Wrench applied at point P.
/// @param r_P_minus_Q  Vector @f$(P - Q)@f$ [m], expressed in wrench frame.
/// @return             Equivalent wrench at point Q.
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