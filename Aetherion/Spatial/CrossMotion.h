// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//
// CrossMotion.h
//
// Spatial motion cross-product operator (Featherstone).
//
// Given twists v and u (both 6x1, stored as [ω; v]),
// the spatial motion cross product is:
//
//    v × u  = crm(v) * u
//
// where
//    crm(v) = [ skew(ω)   0
//              skew(v)  skew(ω) ]
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Twist.h"
//#include "Aetherion/Spatial/Skew.h" 
#include "Aetherion/Spatial/Adjoint.h"

namespace Aetherion::Spatial {

    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6> CrossMotionMatrix(const Twist<Scalar>& v)
    {
        return ad(v);
    }

    template<typename Scalar>
    inline Twist<Scalar> CrossMotion(const Twist<Scalar>& v, const Twist<Scalar>& u)
    {
        Twist<Scalar> out{};
        out.v = CrossMotionMatrix(v) * u.v;
        return out;
    }

} // namespace Aetherion::Spatial