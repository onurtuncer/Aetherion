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

// IMPORTANT: include the header that defines CentralGravity() and J2().
// Adjust this include to your actual filename/path.
#include "Aetherion/Environment/Gravity.h"

namespace Aetherion::Environment {

    template <class Scalar>
    using Vec3E = Eigen::Matrix<Scalar, 3, 1>;

    // -------------------------------------------------------------------------
    // Cross product (branch-free, Eigen-vector in/out)
    // -------------------------------------------------------------------------
    template <class Scalar>
    inline Vec3E<Scalar> Cross(const Vec3E<Scalar>& a, const Vec3E<Scalar>& b)
    {
        return Vec3E<Scalar>{
            a(1)* b(2) - a(2) * b(1),
                a(2)* b(0) - a(0) * b(2),
                a(0)* b(1) - a(1) * b(0)
        };
    }

    // =========================================================================
    // (1) Central gravity wrench applied at CG (no moment)
    // =========================================================================
    template <class Scalar>
    inline Spatial::Wrench<Scalar> GravitationalWrenchAtCG(
        const Vec3<Scalar>& r_W,
        const Scalar& mass_kg,
        const Scalar& mu = Scalar(3.986004418e14)) // [m^3/s^2]
    {
        Spatial::Wrench<Scalar> w{};

        // g_W (std::array<Scalar,3>)
        const Vec3<Scalar> g_arr = CentralGravity(r_W, mu);

        Vec3E<Scalar> g_W;
        g_W << g_arr[0], g_arr[1], g_arr[2];

        const Vec3E<Scalar> F_W_N = mass_kg * g_W;

        w.f.template segment<3>(0).setZero(); // M_W = 0 at CG
        w.f.template segment<3>(3) = F_W_N;   // F_W
        return w;
    }

    // =========================================================================
    // (2) Central gravity + J2 wrench applied at CG (no moment)
    // =========================================================================
    template <class Scalar>
    inline Spatial::Wrench<Scalar> GravitationalWrenchJ2AtCG(
        const Vec3<Scalar>& r_W,
        const Scalar& mass_kg,
        const Scalar& mu = Scalar(3.986004418e14),      // [m^3/s^2]
        const Scalar& Re = Scalar(6378137.0),           // [m]
        const Scalar& J2_coeff = Scalar(1.08262668e-3)) // [-]  <-- DO NOT name this "J2"
    {
        Spatial::Wrench<Scalar> w{};

        // Fully qualify to avoid any accidental name hiding
        const Vec3<Scalar> g_arr = Aetherion::Environment::J2(r_W, mu, Re, J2_coeff);

        Vec3E<Scalar> g_W;
        g_W << g_arr[0], g_arr[1], g_arr[2];

        const Vec3E<Scalar> F_W_N = mass_kg * g_W;

        w.f.template segment<3>(0).setZero();
        w.f.template segment<3>(3) = F_W_N;
        return w;
    }

    // =========================================================================
    // (3) Central gravity wrench applied at an offset from CG:
    //     M = r_app_minus_cg x F
    // =========================================================================
    template <class Scalar>
    inline Spatial::Wrench<Scalar> GravitationalWrenchWithOffset(
        const Vec3<Scalar>& r_W,
        const Scalar& mass_kg,
        const Vec3E<Scalar>& r_app_minus_cg_W_m,
        const Scalar& mu = Scalar(3.986004418e14)) // [m^3/s^2]
    {
        Spatial::Wrench<Scalar> w{};

        const Vec3<Scalar> g_arr = CentralGravity(r_W, mu);

        Vec3E<Scalar> g_W;
        g_W << g_arr[0], g_arr[1], g_arr[2];

        const Vec3E<Scalar> F_W_N = mass_kg * g_W;
        const Vec3E<Scalar> M_W_Nm = Cross(r_app_minus_cg_W_m, F_W_N);

        w.f.template segment<3>(0) = M_W_Nm;
        w.f.template segment<3>(3) = F_W_N;
        return w;
    }

} // namespace Aetherion::Environment