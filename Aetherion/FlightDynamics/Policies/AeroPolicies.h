// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/ODE/RKMK/Lie/SE3.h"

namespace Aetherion::FlightDynamics {

    // Dragless sphere -- satisfies AeroPolicy, returns zero.
    // Optimised away entirely by the compiler.
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

} // namespace Aetherion::FlightDynamics