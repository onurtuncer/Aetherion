// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
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

    /*
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6> CrossMotionMatrix(const Twist<Scalar>& v)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

        const Vec3 w = v.v.template segment<3>(0);
        const Vec3 lin = v.v.template segment<3>(3);

        const Mat3 wx = skew(w);
        const Mat3 vx = skew(lin);

        Eigen::Matrix<Scalar, 6, 6> crm;
        crm.setZero();

        crm.template block<3, 3>(0, 0) = wx;
        crm.template block<3, 3>(3, 0) = vx;
        crm.template block<3, 3>(3, 3) = wx;

        return crm;
    }

    // Convenience: v × u (motion cross product)
    template<typename Scalar>
    inline Twist<Scalar> CrossMotion(const Twist<Scalar>& v, const Twist<Scalar>& u)
    {
        Twist<Scalar> out{};
        out.v = CrossMotionMatrix(v) * u.v;
        return out;
    }
    */

} // namespace Aetherion::Spatial