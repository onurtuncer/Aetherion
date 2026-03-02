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

    // -------------------------------------------------------------------------
    // ad(xi)  — motion adjoint (6x6)
    //
    // xi = [ω; v]
    //
    // ad(xi) =
    //   [ [ω]x   0
    //     [v]x  [ω]x ]
    //
    // Used for:
    //   xi × u = ad(xi) u
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // ad*(xi) — force adjoint (dual)
    //
    // ad*(xi) = - ad(xi)^T
    //
    // Used for:
    //   xi ×* f = ad*(xi) f
    // -------------------------------------------------------------------------
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 6>
        ad_star(const Twist<Scalar>& xi)
    {
        return -ad(xi).transpose();
    }

    // -------------------------------------------------------------------------
    // Efficient ad*(xi) * y without building 6x6
    //
    // For y = [a; b]:
    //
    // ad*(xi) y =
    //   [ ω×a + v×b
    //     ω×b ]
    // -------------------------------------------------------------------------
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