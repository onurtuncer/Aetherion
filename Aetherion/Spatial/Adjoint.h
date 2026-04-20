// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Adjoint.h
//
// Lie algebra adjoint operators for SE(3).
//
// xi = [ω; v] ∈ se(3)
//
// ad(xi)      : motion adjoint (a.k.a crm)
// ad*(xi)     : force adjoint  (a.k.a crf)
//
// Conventions:
//   Twist  = [ω; v]
//   Wrench = [M; F]
//
// Identities:
//   ad*(xi) = - ad(xi)^T
//
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Skew.h"
#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::Spatial {

    /// @brief Lie algebra adjoint (motion cross-product matrix).
    ///
    /// Returns the 6×6 matrix @f$ \mathrm{ad}(\xi) @f$ such that
    /// @f[ \xi \times \mathbf{u} = \mathrm{ad}(\xi)\,\mathbf{u} @f]
    /// with @f$ \xi = [\omega;\, v] @f$:
    /// @f[
    ///   \mathrm{ad}(\xi) = \begin{bmatrix} [\omega]_\times & 0 \\ [v]_\times & [\omega]_\times \end{bmatrix}
    /// @f]
    ///
    /// @param xi  Input twist @f$[\omega;\,v]@f$.
    /// @return    6×6 motion adjoint matrix.
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6>
        ad(const Twist<Scalar>& xi)
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

        const Vec3 w = xi.v.template segment<3>(0);
        const Vec3 v = xi.v.template segment<3>(3);

        const Mat3 Wx = skew(w);
        const Mat3 Vx = skew(v);

        Eigen::Matrix<Scalar, 6, 6> out;
        out.setZero();

        out.template block<3, 3>(0, 0) = Wx;
        out.template block<3, 3>(3, 0) = Vx;
        out.template block<3, 3>(3, 3) = Wx;

        return out;
    }

    /// @brief Lie algebra co-adjoint (force cross-product matrix).
    ///
    /// Returns @f$ \mathrm{ad}^*(\xi) = -\mathrm{ad}(\xi)^\top @f$, used as
    /// @f[ \xi \times^* \mathbf{f} = \mathrm{ad}^*(\xi)\,\mathbf{f} @f]
    ///
    /// @param xi  Input twist @f$[\omega;\,v]@f$.
    /// @return    6×6 force adjoint matrix.
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6>
        ad_star(const Twist<Scalar>& xi)
    {
        return -ad(xi).transpose();
    }

    /// @brief Efficient product @f$ \mathrm{ad}^*(\xi)\,\mathbf{y} @f$ without forming the 6×6 matrix.
    ///
    /// For @f$ \mathbf{y} = [a;\, b] @f$:
    /// @f[
    ///   \mathrm{ad}^*(\xi)\,\mathbf{y} = \begin{bmatrix} \omega\times a + v\times b \\ \omega\times b \end{bmatrix}
    /// @f]
    ///
    /// @param xi  Input twist @f$[\omega;\,v]@f$.
    /// @param y   6-vector to multiply (e.g. spatial momentum @f$[h_\omega;\,h_v]@f$).
    /// @return    6-vector result.
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 1>
        ad_star_times(
            const Twist<Scalar>& xi,
            const Eigen::Matrix<Scalar, 6, 1>& y)
    {
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

        const Vec3 w = xi.v.template segment<3>(0);
        const Vec3 v = xi.v.template segment<3>(3);

        const Vec3 a = y.template segment<3>(0);
        const Vec3 b = y.template segment<3>(3);

        Eigen::Matrix<Scalar, 6, 1> out;

        out.template segment<3>(0) = w.cross(a) + v.cross(b);
        out.template segment<3>(3) = w.cross(b);

        return out;
    }

} // namespace Aetherion::Spatial