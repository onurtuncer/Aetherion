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

    /// @brief Build the 6×6 spatial motion transform matrix.
    ///
    /// Maps a @c Twist expressed in frame A to frame B:
    /// @f[ X = \begin{bmatrix} R & 0 \\ [r]_\times R & R \end{bmatrix} @f]
    /// so that @f$ \mathbf{v}_B = X\,\mathbf{v}_A @f$.
    ///
    /// @param R  Rotation matrix from A to B (3×3).
    /// @param r  Translation from A-origin to B-origin, expressed in B (3×1) [m].
    /// @return   6×6 spatial motion transform.
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

    /// @brief Build the 6×6 spatial force transform matrix (dual of motion transform).
    ///
    /// Maps a @c Wrench expressed in frame A to frame B:
    /// @f[ X^* = \begin{bmatrix} R & [r]_\times R \\ 0 & R \end{bmatrix} @f]
    /// so that @f$ \mathbf{f}_B = X^*\,\mathbf{f}_A @f$.
    ///
    /// @param R  Rotation matrix from A to B (3×3).
    /// @param r  Translation from A-origin to B-origin, expressed in B (3×1) [m].
    /// @return   6×6 spatial force transform.
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

    /// @brief Apply spatial motion transform to a @c Twist.
    /// @param R    Rotation from source to target frame.
    /// @param r    Translation from source to target origin (in target frame) [m].
    /// @param v_A  Input twist in source frame A.
    /// @return     Equivalent twist in target frame B.
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

    /// @brief Apply spatial force transform to a @c Wrench.
    /// @param R    Rotation from source to target frame.
    /// @param r    Translation from source to target origin (in target frame) [m].
    /// @param f_A  Input wrench in source frame A.
    /// @return     Equivalent wrench in target frame B.
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