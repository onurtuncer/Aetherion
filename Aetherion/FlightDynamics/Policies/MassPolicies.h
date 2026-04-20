// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

/// @brief Mass policy for a vehicle with constant mass (no propellant burn).
///
/// @c mdot() always returns zero.  Satisfies @c MassPolicy.
    struct ConstantMassPolicy {
        /// @brief Mass rate — always zero.
        /// @param t     Current time [s] (unused).
        /// @param mass  Current mass [kg] (unused).
        /// @return      0.
        template<class S>
        S mdot(S /*t*/, S /*mass*/) const { return S(0); }
    };

    static_assert(MassPolicy<ConstantMassPolicy>);

/// @brief Mass policy for a constant propellant burn rate.
///
/// @c mdot() returns a fixed negative value @c mdot_kgs [kg/s].
/// Satisfies @c MassPolicy.
    struct LinearBurnPolicy {
        double mdot_kgs{ 0.0 }; ///< Constant mass flow rate [kg/s] (negative for burning propellant).
        /// @brief Mass rate — returns the constant @c mdot_kgs.
        /// @param t     Current time [s] (unused).
        /// @param mass  Current mass [kg] (unused).
        /// @return      @c mdot_kgs [kg/s].
        template<class S>
        S mdot(S /*t*/, S /*mass*/) const { return S(mdot_kgs); }
    };

    static_assert(MassPolicy<LinearBurnPolicy>);

} // namespace Aetherion::FlightDynamics