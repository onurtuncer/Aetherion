

// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// RigidBodyState.h
//
// State on the product manifold SE(3) x R^7.
//
//   g     ∈ SE(3)   — pose (attitude + ECI position)
//   nu_B  ∈ R^6     — body twist [omega_B(3); v_B(3)]
//   m     ∈ R^1     — vehicle mass [kg]
//
// x = [nu_B; m] ∈ R^7 is the Euclidean part stored/stepped separately.
//
#pragma once

#include <Eigen/Dense>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

namespace Aetherion::FlightDynamics {

    template<class S>
    struct RigidBodyState {
        using Scalar = S;
        using SE3Type = ODE::RKMK::Lie::SE3<S>;
        using Vec6 = Eigen::Matrix<S, 6, 1>;

        SE3Type g{};
        Vec6    nu_B{ Vec6::Zero() };
        S       m{ S(0) };
    };

    using RigidBodyStateD = RigidBodyState<double>;
    using RigidBodyStateAD = RigidBodyState<CppAD::AD<double>>;

} // namespace Aetherion::FlightDynamics