// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::RigidBody {

    template<class Scalar>
    struct WrenchContext
    {
        // Inertial position from Earth center [m], expressed in W
        Aetherion::Environment::Vec3<Scalar> r_W;

        // Rotation mapping W vectors into body frame B: x_B = R_BW * x_W
        Eigen::Matrix<Scalar, 3, 3> R_BW;

        // Mass [kg]
        Scalar mass_kg;

    };

    template<class Scalar>
    struct IWrenchProvider
    {
        virtual ~IWrenchProvider() = default;

        // Canonical output: Wrench expressed in body frame at CG
        virtual Aetherion::Spatial::Wrench<Scalar>
            ComputeWrench_B_at_CG(const WrenchContext<Scalar>& ctx) const = 0;
    };

} // namespace Aetherion::RigidBody