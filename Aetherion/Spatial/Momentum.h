// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Momentum.h
//
// Spatial momentum utilities.
//
// Conventions:
//   - Twist v:   [ω; v]
//   - Spatial momentum h: [angular_momentum; linear_momentum]
//
// Given a spatial inertia matrix I (6x6), spatial momentum is:
//   h = I * v
//
// Notes:
//   - This header stays generic: it accepts I as Eigen::Matrix<Scalar,6,6>.
//   - When you add a SpatialInertia type later, you can overload MomentumFromInertia(...).
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Twist.h"

namespace Aetherion::Spatial {

    template<typename Scalar>
    struct Momentum {
        Eigen::Matrix<Scalar, 6, 1> h; // [h_angular; h_linear]
    };

    // h = I * v
    template<typename Scalar>
    inline Momentum<Scalar> MomentumFromSpatialInertiaAndTwist(
        const Eigen::Matrix<Scalar, 6, 6>& I_spatial,
        const Twist<Scalar>& v)
    {
        Momentum<Scalar> out{};
        out.h = I_spatial * v.v;
        return out;
    }

    // Convenience accessors if you prefer split form
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 1> AngularMomentum(const Momentum<Scalar>& h)
    {
        return h.h.template segment<3>(0);
    }

    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 1> LinearMomentum(const Momentum<Scalar>& h)
    {
        return h.h.template segment<3>(3);
    }

} // namespace Aetherion::Spatial