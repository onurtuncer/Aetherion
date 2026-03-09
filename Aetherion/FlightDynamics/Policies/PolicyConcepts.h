// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// PolicyConcepts.h
//
// Concepts that define the compile-time interface for each physics policy.
// Each concept is checked for both double and CppAD::AD<double> --
// a policy that is not AD-safe fails here, not inside CppAD tape evaluation.
//
#pragma once

#include <concepts>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/Spatial/Wrench.h>

namespace Aetherion::FlightDynamics {

    namespace detail {
        template<class P, class S>
        concept GravityPolicyFor =
            requires(const P & p,
        const ODE::RKMK::Lie::SE3<S>&g,
            S mass)
        { { p(g, mass) } -> std::same_as<Spatial::Wrench<S>>;
        };

        template<class P, class S>
        concept AeroPolicyFor =
            requires(const P & p,
        const ODE::RKMK::Lie::SE3<S>&g,
            const Eigen::Matrix<S, 6, 1>&nu_B,
            S mass, S t)
        { { p(g, nu_B, mass, t) } -> std::same_as<Spatial::Wrench<S>>;
        };

        template<class P, class S>
        concept MassPolicyFor =
            requires(const P & p, S t, S m)
        { { p.mdot(t, m) } -> std::same_as<S>;
        };
    }

    // -------------------------------------------------------------------------
    // Public concepts -- must satisfy both double and AD<double>.
    // -------------------------------------------------------------------------

    template<class P>
    concept GravityPolicy =
        detail::GravityPolicyFor<P, double>&&
        detail::GravityPolicyFor<P, CppAD::AD<double>>;

    template<class P>
    concept AeroPolicy =
        detail::AeroPolicyFor<P, double>&&
        detail::AeroPolicyFor<P, CppAD::AD<double>>;

    // PropulsionPolicy has identical signature to AeroPolicy.
    template<class P>
    concept PropulsionPolicy = AeroPolicy<P>;

    template<class P>
    concept MassPolicy =
        detail::MassPolicyFor<P, double>&&
        detail::MassPolicyFor<P, CppAD::AD<double>>;

} // namespace Aetherion::FlightDynamics