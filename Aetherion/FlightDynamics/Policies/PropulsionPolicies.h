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

    // -------------------------------------------------------------------------
    // Zero thrust — coasting, unpowered phases.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Constant axial thrust along body +z axis.
    // -------------------------------------------------------------------------
    struct ConstantThrustPolicy {
        double thrust_N{ 0.0 };  // [N]

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

} // namespace Aetherion::FlightDynamics