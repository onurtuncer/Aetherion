// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

    struct ConstantMassPolicy {
        template<class S>
        S mdot(S /*t*/, S /*mass*/) const { return S(0); }
    };

    static_assert(MassPolicy<ConstantMassPolicy>);

    struct LinearBurnPolicy {
        double mdot_kgs{ 0.0 }; // [kg/s], negative
        template<class S>
        S mdot(S /*t*/, S /*mass*/) const { return S(mdot_kgs); }
    };

    static_assert(MassPolicy<LinearBurnPolicy>);

} // namespace Aetherion::FlightDynamics