// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
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

    /*
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6> CrossForceMatrix(const Twist<Scalar>& v)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

        const Vec3 w = v.v.template segment<3>(0);
        const Vec3 lin = v.v.template segment<3>(3);

        const Mat3 wx = skew(w);
        const Mat3 vx = skew(lin);

        Eigen::Matrix<Scalar, 6, 6> crf;
        crf.setZero();

        crf.template block<3, 3>(0, 0) = wx;
        crf.template block<3, 3>(0, 3) = vx;
        crf.template block<3, 3>(3, 3) = wx;

        return crf;
    }

    // Convenience: v ×* f (force cross product)
    template<typename Scalar>
    inline Wrench<Scalar> CrossForce(const Twist<Scalar>& v, const Wrench<Scalar>& f)
    {
        Wrench<Scalar> out{};
        out.f = CrossForceMatrix(v) * f.f;
        return out;
    }

    // Convenience: check duality crf(v) = -crm(v)^T (optional helper)
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6> CrossForceMatrixFromDuality(const Twist<Scalar>& v)
    {
        return -CrossMotionMatrix(v).transpose();
    }
    */

} // namespace Aetherion::Spatial