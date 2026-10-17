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
        template <class P, class S>
        concept GravityPolicyFor =
            requires(const P& p,
                const ODE::RKMK::Lie::SE3<S>& g,
                S mass) {
                {
                    p(g, mass)
                } -> std::same_as<Spatial::Wrench<S>>;
            };

        template <class P, class S>
        concept AeroPolicyFor =
            requires(const P& p,
                const ODE::RKMK::Lie::SE3<S>& g,
                const Eigen::Matrix<S, 6, 1>& nu_B,
                S mass, S t) {
                {
                    p(g, nu_B, mass, t)
                } -> std::same_as<Spatial::Wrench<S>>;
            };

        template <class P, class S>
        concept MassPolicyFor =
            requires(const P& p, S t, S m) {
                {
                    p.mdot(t, m)
                } -> std::same_as<S>;
            };
    }

    // -------------------------------------------------------------------------
    // Public concepts -- must satisfy both double and AD<double>.
    // -------------------------------------------------------------------------

    /// @brief Concept for a gravity wrench policy.
    ///
    /// A type @c P satisfies @c GravityPolicy if, for both @c S = @c double
    /// and @c S = @c CppAD::AD\<double\>, it provides:
    /// @code
    ///   Spatial::Wrench<S> operator()(const SE3<S>& g, S mass) const;
    /// @endcode
    /// The returned wrench must be in the **body frame** with storage convention
    /// @f$[M;\,F]@f$ (moment first, force last), and is the wrench **at the
    /// centre of mass**, where gravity acts: force @f$m\,\mathbf{g}@f$ and no
    /// moment (a gravity-gradient torque about the centre of mass, if the model
    /// has one).  @c RigidBody::VectorField moves it to the body origin, about
    /// which the equations of motion are written; a policy must not add that
    /// transfer itself.
    template <class P>
    concept GravityPolicy =
        detail::GravityPolicyFor<P, double> &&
        detail::GravityPolicyFor<P, CppAD::AD<double>>;

    /// @brief Concept for an aerodynamic wrench policy.
    ///
    /// A type @c P satisfies @c AeroPolicy if, for both @c S = @c double
    /// and @c S = @c CppAD::AD\<double\>, it provides:
    /// @code
    ///   Spatial::Wrench<S> operator()(const SE3<S>& g,
    ///                                  const Eigen::Matrix<S,6,1>& nu_B,
    ///                                  S mass, S t) const;
    /// @endcode
    /// The returned wrench is in the body frame, @f$[M;\,F]@f$, and is **about
    /// the body origin** -- the vehicle model's moment reference centre; it is
    /// summed into the equations of motion as it is.
    template <class P>
    concept AeroPolicy =
        detail::AeroPolicyFor<P, double> &&
        detail::AeroPolicyFor<P, CppAD::AD<double>>;

    /// @brief Concept for a propulsion wrench policy.
    ///
    /// Identical call signature to @c AeroPolicy:
    /// @code
    ///   Spatial::Wrench<S> operator()(const SE3<S>& g,
    ///                                  const Eigen::Matrix<S,6,1>& nu_B,
    ///                                  S mass, S t) const;
    /// @endcode
    /// The wrench is about the body origin, as for @c AeroPolicy.
    template <class P>
    concept PropulsionPolicy = AeroPolicy<P>;

    /// @brief Concept for a mass-rate (propellant burn) policy.
    ///
    /// A type @c P satisfies @c MassPolicy if, for both @c S = @c double
    /// and @c S = @c CppAD::AD\<double\>, it provides:
    /// @code
    ///   S mdot(S t, S mass) const;   // [kg/s], typically ≤ 0
    /// @endcode
    template <class P>
    concept MassPolicy =
        detail::MassPolicyFor<P, double> &&
        detail::MassPolicyFor<P, CppAD::AD<double>>;

} // namespace Aetherion::FlightDynamics