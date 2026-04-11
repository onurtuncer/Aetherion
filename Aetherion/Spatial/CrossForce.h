// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//
// CrossForce.h
//
// Spatial force cross-product operator (Featherstone).
//
// Given twist v (stored as [ω; v]) and wrench f (stored as [M; F]),
// the spatial force cross product is:
//
//    v ×* f = crf(v) * f
//
// where
//    crf(v) = [ skew(ω)  skew(v)
//               0        skew(ω) ]
//
// Duality:
//    crf(v) = -crm(v)^T
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Wrench.h"
//#include "Aetherion/Spatial/Skew.h"
#include "Aetherion/Spatial/Adjoint.h"

namespace Aetherion::Spatial {

    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6> CrossForceMatrix(const Twist<Scalar>& v)
    {
        return ad_star(v);
    }

    template<typename Scalar>
    inline Wrench<Scalar> CrossForce(const Twist<Scalar>& v, const Wrench<Scalar>& f)
    {
        Wrench<Scalar> out{};
        out.f = CrossForceMatrix(v) * f.f;
        return out;
    }

} // namespace Aetherion::Spatial