// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Transform.h
//
// Spatial transform utilities (Featherstone).
//
// Conventions:
//   Twist   v = [ω; v]
//   Wrench  f = [M; F]
//
// Transform defined by:
//   R : rotation matrix (3x3)
//   r : translation vector (3x1)
//       (vector from old origin to new origin, expressed in destination frame)
//
// Motion transform:
//   v_B = X * v_A
//
// Force transform (dual):
//   f_B = X* * f_A
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/Spatial/Skew.h"

namespace Aetherion::Spatial {

    template<typename Scalar>
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

    template<typename Scalar>
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    template<typename Scalar>
    using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

    // -------------------------------------------------------------------------
    // Spatial Motion Transform Matrix
    // -------------------------------------------------------------------------
    template<typename Scalar>
    inline Mat6<Scalar> MotionTransformMatrix(
        const Mat3<Scalar>& R,
        const Vec3<Scalar>& r)
    {
        Mat6<Scalar> X;
        X.setZero();

        const Mat3<Scalar> rx = skew(r);

        X.template block<3, 3>(0, 0) = R;
        X.template block<3, 3>(3, 0) = rx * R;
        X.template block<3, 3>(3, 3) = R;

        return X;
    }

    // -------------------------------------------------------------------------
    // Spatial Force Transform Matrix (dual)
    // -------------------------------------------------------------------------
    template<typename Scalar>
    inline Mat6<Scalar> ForceTransformMatrix(
        const Mat3<Scalar>& R,
        const Vec3<Scalar>& r)
    {
        Mat6<Scalar> X;
        X.setZero();

        const Mat3<Scalar> rx = skew(r);

        X.template block<3, 3>(0, 0) = R;
        X.template block<3, 3>(0, 3) = rx * R;
        X.template block<3, 3>(3, 3) = R;

        return X;
    }

    // -------------------------------------------------------------------------
    // Apply Motion Transform
    // -------------------------------------------------------------------------
    template<typename Scalar>
    inline Twist<Scalar> TransformMotion(
        const Mat3<Scalar>& R,
        const Vec3<Scalar>& r,
        const Twist<Scalar>& v_A)
    {
        Twist<Scalar> v_B{};
        v_B.v = MotionTransformMatrix(R, r) * v_A.v;
        return v_B;
    }

    // -------------------------------------------------------------------------
    // Apply Force Transform
    // -------------------------------------------------------------------------
    template<typename Scalar>
    inline Wrench<Scalar> TransformForce(
        const Mat3<Scalar>& R,
        const Vec3<Scalar>& r,
        const Wrench<Scalar>& f_A)
    {
        Wrench<Scalar> f_B{};
        f_B.f = ForceTransformMatrix(R, r) * f_A.f;
        return f_B;
    }

} // namespace Aetherion::Spatial