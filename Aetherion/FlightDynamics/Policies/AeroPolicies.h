// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/ODE/RKMK/Lie/SE3.h"

namespace Aetherion::FlightDynamics {

/// @brief Aerodynamic policy that returns a zero wrench (no aerodynamic forces).
///
/// Models a dragless, liftless body (e.g. vacuum trajectory or baseline case).
/// Satisfies @c AeroPolicy; the compiler optimises the zero addition away entirely.
    struct ZeroAeroPolicy {
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

    static_assert(AeroPolicy<ZeroAeroPolicy>);

} // namespace Aetherion::FlightDynamics