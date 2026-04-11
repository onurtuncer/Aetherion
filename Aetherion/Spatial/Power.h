// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//
// Power.h
//
// Spatial power utilities.
//
// Conventions:
//   - Twist v:   [ω_x,ω_y,ω_z, v_x,v_y,v_z]^T
//   - Wrench f:  [M_x,M_y,M_z, F_x,F_y,F_z]^T
//
// Spatial power (instantaneous):
//   P = fᵀ v = M·ω + F·v
// Units: [W] = [N*m/s] = [N*m] * [rad/s] + [N] * [m/s]
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::Spatial {

    template<typename Scalar>
    inline Scalar Power(const Wrench<Scalar>& f, const Twist<Scalar>& v)
    {
        // No branching; Eigen expression is CppAD-friendly as long as Scalar is.
        return f.f.dot(v.v);
    }

    template<typename Scalar>
    inline Scalar Power(
        const Eigen::Matrix<Scalar, 3, 1>& M_Nm,
        const Eigen::Matrix<Scalar, 3, 1>& F_N,
        const Eigen::Matrix<Scalar, 3, 1>& omega_rad_s,
        const Eigen::Matrix<Scalar, 3, 1>& v_m_s)
    {
        return M_Nm.dot(omega_rad_s) + F_N.dot(v_m_s);
    }

} // namespace Aetherion::Spatial