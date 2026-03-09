// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace Aetherion::Spatial {

    template<typename Scalar>
    struct Twist {
        Eigen::Matrix<Scalar, 6, 1> v; // [ω_x,ω_y,ω_z, v_x,v_y,v_z]^T
    };

} // namespace Aetherion::Spatial