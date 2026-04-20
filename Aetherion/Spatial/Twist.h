// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace Aetherion::Spatial {

/// @brief Spatial velocity (twist) following Featherstone conventions.
///
/// Stores the 6-D body velocity as a single column vector:
/// @f[ \mathbf{v} = [\omega_x,\, \omega_y,\, \omega_z,\, v_x,\, v_y,\, v_z]^\top @f]
/// where @f$\boldsymbol\omega@f$ is the angular velocity [rad/s] and
/// @f$\mathbf{v}@f$ is the linear velocity [m/s], both expressed in the
/// body frame unless noted otherwise.
///
/// @tparam Scalar Numeric type (e.g. @c double or @c CppAD::AD<double>).
    template<typename Scalar>
    struct Twist {
        /// Packed 6-vector @f$[\omega; v]@f$: indices 0–2 angular, 3–5 linear.
        Eigen::Matrix<Scalar, 6, 1> v; // [ω_x,ω_y,ω_z, v_x,v_y,v_z]^T
    };

} // namespace Aetherion::Spatial