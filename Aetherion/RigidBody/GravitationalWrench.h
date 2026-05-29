// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// GravitationalWrench.h
//
// Builds gravitational wrench from gravity acceleration models.
//
// Conventions:
//   - Spatial::Wrench storage: [M_x, M_y, M_z, F_x, F_y, F_z]^T
//   - Gravity acceleration is expressed in inertial frame W.
//   - Output wrench is expressed in the SAME frame as the computed acceleration.
//     (i.e., if g is in W, force is in W, moment computed in W).
//
// Notes:
//   - No branching; CppAD-friendly.
//   - Depends on Aetherion::Environment gravity acceleration models:
//       CentralGravity(...), J2(...)
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/Environment/Gravity.h"   // also pulls in WGS84.h

namespace Aetherion::RigidBody {

    // =========================================================================
    // (1) Central gravity wrench applied at CG (no moment)
    // =========================================================================
    template <class Scalar>
    inline Spatial::Wrench<Scalar> GravitationalWrenchAtCG(
        const Aetherion::Environment::Vec3<Scalar>& r_W,
        const Scalar& mass_kg,
        const Scalar& mu = Scalar(Environment::WGS84::kGM_m3_s2))
    {
        Spatial::Wrench<Scalar> w{};

        const Aetherion::Environment::Vec3<Scalar> g_arr = Environment::CentralGravity(r_W, mu);

        Eigen::Matrix<Scalar, 3, 1> g_W;
        g_W << g_arr[0], g_arr[1], g_arr[2];

        w.f.template segment<3>(0).setZero(); // M_W = 0 at CG
        w.f.template segment<3>(3) = mass_kg * g_W;
        return w;
    }

    // =========================================================================
    // (2) Central gravity + J2 wrench applied at CG (no moment)
    // =========================================================================
    template <class Scalar>
    inline Spatial::Wrench<Scalar> GravitationalWrenchJ2AtCG(
        const Aetherion::Environment::Vec3<Scalar>& r_W,
        const Scalar& mass_kg,
        const Scalar& mu       = Scalar(Environment::WGS84::kGM_m3_s2),
        const Scalar& Re       = Scalar(Environment::WGS84::kSemiMajorAxis_m),
        const Scalar& J2_coeff = Scalar(Environment::WGS84::kJ2))
    {
        Spatial::Wrench<Scalar> w{};

        const Aetherion::Environment::Vec3<Scalar> g_arr = Aetherion::Environment::J2(r_W, mu, Re, J2_coeff);

        Eigen::Matrix<Scalar, 3, 1> g_W;
        g_W << g_arr[0], g_arr[1], g_arr[2];

        w.f.template segment<3>(0).setZero();
        w.f.template segment<3>(3) = mass_kg * g_W;
        return w;
    }

    // =========================================================================
    // (3) Central gravity wrench applied at an offset from CG:
    //     M = r_app_minus_cg x F
    // =========================================================================
    template <class Scalar>
    inline Spatial::Wrench<Scalar> GravitationalWrenchWithOffset(
        const Aetherion::Environment::Vec3<Scalar>&  r_W,
        const Scalar&                                mass_kg,
        const Eigen::Matrix<Scalar, 3, 1>&           r_app_minus_cg_W_m,
        const Scalar& mu = Scalar(Environment::WGS84::kGM_m3_s2))
    {
        Spatial::Wrench<Scalar> w{};

        const Environment::Vec3<Scalar> g_arr = Environment::CentralGravity(r_W, mu);

        Eigen::Matrix<Scalar, 3, 1> g_W;
        g_W << g_arr[0], g_arr[1], g_arr[2];

        const Eigen::Matrix<Scalar, 3, 1> F_W_N  = mass_kg * g_W;
        const Eigen::Matrix<Scalar, 3, 1> M_W_Nm = r_app_minus_cg_W_m.cross(F_W_N);

        w.f.template segment<3>(0) = M_W_Nm;
        w.f.template segment<3>(3) = F_W_N;
        return w;
    }

} // namespace Aetherion::RigidBody
