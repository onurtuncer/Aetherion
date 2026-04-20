// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//
// State on the product manifold SE(3) x R^7.
//
//   g     ∈ SE(3)   -- pose (attitude + ECI position)
//   nu_B  ∈ R^6     -- body twist [omega_B(3); v_B(3)]
//   m     ∈ R^1     -- vehicle mass [kg]
//
// x = [nu_B; m] ∈ R^7 is the Euclidean part stored/stepped separately.

#pragma once

#include <Eigen/Dense>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

namespace Aetherion::RigidBody {

/// @brief Full state of a 6-DoF rigid body on @f$ SE(3) \times \mathbb{R}^7 @f$.
///
/// The state is split into a Lie-group part and a Euclidean part:
/// - @c g   : pose @f$\in SE(3)@f$ — rotation matrix + ECI position.
/// - @c nu_B: body-frame twist @f$[\omega_B(3);\, v_B(3)]@f$ [rad/s | m/s].
/// - @c m   : current vehicle mass [kg].
///
/// The Euclidean part @f$\mathbf{x} = [\nu_B;\, m] \in \mathbb{R}^7@f$ is
/// stepped by the Euclidean ODE solver; @c g is stepped on the manifold.
///
/// @tparam S Scalar type (@c double or @c CppAD::AD<double>).
    template<class S>
    struct State {
        using Scalar = S;
        using SE3Type = ODE::RKMK::Lie::SE3<S>;
        using Vec6 = Eigen::Matrix<S, 6, 1>;

        SE3Type g{};              ///< Pose in SE(3): attitude (rotation) + ECI position.
        Vec6    nu_B{ Vec6::Zero() }; ///< Body twist @f$[\omega_B;\, v_B]@f$ [rad/s | m/s].
        S       m{ S(0) };        ///< Vehicle mass [kg].
    };

    using StateD = State<double>;

#if __has_include(<cppad/cppad.hpp>)
#include <cppad/cppad.hpp>
    using StateAD = State<CppAD::AD<double>>;
#endif

} // namespace Aetherion::RigidBody