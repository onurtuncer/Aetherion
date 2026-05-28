// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// PropulsionPolicies.h
//
// Propulsion wrench policies.
// signature: operator()(g, nu_B, mass, t) -> Wrench<S>
//
#pragma once

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>

namespace Aetherion::FlightDynamics {

    /// @brief Propulsion policy that returns a zero wrench (coasting / unpowered phase).
    ///
    /// Satisfies @c PropulsionPolicy.  The compiler eliminates the zero addition.
    struct ZeroPropulsionPolicy {
        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>&,
                const Eigen::Matrix<S, 6, 1>&,
                S, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            return w;
        }
    };

    static_assert(PropulsionPolicy<ZeroPropulsionPolicy>);

    /// @brief Constant axial thrust along the body +z axis.
    ///
    /// Sets @f$F_z = \text{thrust\_N}@f$ in the body frame; all other
    /// wrench components are zero.  Satisfies @c PropulsionPolicy.
    struct ConstantThrustPolicy {
        double thrust_N{ 0.0 }; ///< Axial thrust magnitude [N] (positive = thrust along +z body axis).

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>&,
                const Eigen::Matrix<S, 6, 1>&,
                S, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f(5) = S(thrust_N);  // F_z in body frame
            return w;
        }
    };

    static_assert(PropulsionPolicy<ConstantThrustPolicy>);

    /// @brief Axial thrust along the body +x axis (forward/nose direction).
    ///
    /// Puts @f$F_x = \text{thrust\_N}@f$ in the body frame; all other wrench
    /// components are zero.  Matches the @c bodyThrustForce_X convention used
    /// by the NASA two-stage rocket DAVE-ML propulsion model.
    ///
    /// @c thrust_N is a mutable public member so closed-loop and staged-burn
    /// simulators can update it between integration steps (zero-order hold).
    ///
    /// Satisfies @c PropulsionPolicy.
    struct AxialThrustPolicy {
        double thrust_N{ 0.0 }; ///< Thrust magnitude [N] (positive = forward along body +x).

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>&,
                const Eigen::Matrix<S, 6, 1>&,
                S, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f(3) = S(thrust_N);  // Fx body (+x = nose / thrust axis)
            return w;
        }
    };

    static_assert(PropulsionPolicy<AxialThrustPolicy>);

} // namespace Aetherion::FlightDynamics