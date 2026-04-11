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

    template<class S>
    struct State {
        using Scalar = S;
        using SE3Type = ODE::RKMK::Lie::SE3<S>;
        using Vec6 = Eigen::Matrix<S, 6, 1>;

        SE3Type g{};
        Vec6    nu_B{ Vec6::Zero() };
        S       m{ S(0) };
    };

    using StateD = State<double>;

#if __has_include(<cppad/cppad.hpp>)
#include <cppad/cppad.hpp>
    using StateAD = State<CppAD::AD<double>>;
#endif

} // namespace Aetherion::RigidBody