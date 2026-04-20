// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
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

/// @brief Spatial momentum @f$\mathbf{h} = \mathbf{I}\,\mathbf{v}@f$.
///
/// Stores the 6-D spatial momentum vector @f$[h_\omega;\, h_v]@f$:
/// indices 0–2 are angular momentum [kg·m²/s], indices 3–5 are linear
/// momentum [kg·m/s].
///
/// @tparam Scalar Numeric type.
    template<typename Scalar>
    struct Momentum {
        /// Packed 6-vector: indices 0–2 angular [kg·m²/s], 3–5 linear [kg·m/s].
        Eigen::Matrix<Scalar, 6, 1> h; // [h_angular; h_linear]
    };

/// @brief Compute spatial momentum @f$\mathbf{h} = \mathbf{I}\,\mathbf{v}@f$.
/// @param I_spatial  6×6 spatial inertia matrix.
/// @param v          Body twist.
/// @return           Spatial momentum.
    template<typename Scalar>
    inline Momentum<Scalar> MomentumFromSpatialInertiaAndTwist(
        const Eigen::Matrix<Scalar, 6, 6>& I_spatial,
        const Twist<Scalar>& v)
    {
        Momentum<Scalar> out{};
        out.h = I_spatial * v.v;
        return out;
    }

/// @brief Extract the angular-momentum part of a spatial momentum vector.
/// @param h  Spatial momentum.
/// @return   3-vector angular momentum [kg·m²/s].
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 1> AngularMomentum(const Momentum<Scalar>& h)
    {
        return h.h.template segment<3>(0);
    }

/// @brief Extract the linear-momentum part of a spatial momentum vector.
/// @param h  Spatial momentum.
/// @return   3-vector linear momentum [kg·m/s].
    template<typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 1> LinearMomentum(const Momentum<Scalar>& h)
    {
        return h.h.template segment<3>(3);
    }

} // namespace Aetherion::Spatial
