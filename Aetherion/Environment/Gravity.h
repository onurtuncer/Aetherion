// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstddef>
#include <cmath>

#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>   // detail::SquareRoot

namespace Aetherion::Environment {

    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

    // -------------------------------------------------------------------------
    // CentralGravity
    //
    // Point-mass (Newtonian) gravitational acceleration in ECI/inertial frame.
    //
    // Input:  r_W   -- position [m] from Earth centre, expressed in ECI
    // Output: g_W   -- gravitational acceleration [m/s^2] in ECI
    //
    // Default mu = WGS-84 geocentric gravitational parameter.
    // -------------------------------------------------------------------------
    template <class Scalar>
    inline Vec3<Scalar> CentralGravity(
        const Vec3<Scalar>& r_W,
        const Scalar& mu = Scalar(WGS84::kGM_m3_s2))
    {
        using detail::SquareRoot;

        const Scalar x = r_W[0];
        const Scalar y = r_W[1];
        const Scalar z = r_W[2];

        const Scalar r2 = x * x + y * y + z * z;
        const Scalar r = SquareRoot(r2);
        const Scalar inv_r3 = Scalar(1) / (r2 * r);

        const Scalar k = -mu * inv_r3;

        return Vec3<Scalar>{ k* x, k* y, k* z };
    }

    // -------------------------------------------------------------------------
    // J2
    //
    // Gravitational acceleration including the WGS-84 J2 oblateness perturbation.
    // Calls CentralGravity() for the central term and adds the J2 delta.
    //
    // Input:  r_W      -- position [m] in ECI
    //         mu       -- GM  [m^3/s^2]      (default: WGS-84)
    //         Re       -- equatorial radius [m]  (default: WGS-84 semi-major axis)
    //         J2_coeff -- second zonal harmonic [-]  (default: WGS-84)
    //
    // Output: g_W -- total acceleration [m/s^2] (central + J2 perturbation)
    // -------------------------------------------------------------------------
    template <class Scalar>
    inline Vec3<Scalar> J2(
        const Vec3<Scalar>& r_W,
        const Scalar& mu = Scalar(WGS84::kGM_m3_s2),
        const Scalar& Re = Scalar(WGS84::kSemiMajorAxis_m),
        const Scalar& J2_coeff = Scalar(WGS84::kJ2))
    {
        using detail::SquareRoot;

        // Central term -- no code duplication
        Vec3<Scalar> g = CentralGravity(r_W, mu);

        // J2 perturbation delta
        const Scalar x = r_W[0];
        const Scalar y = r_W[1];
        const Scalar z = r_W[2];

        const Scalar r2 = x * x + y * y + z * z;
        const Scalar r = SquareRoot(r2);
        const Scalar r5 = r2 * r2 * r;
        const Scalar inv_r5 = Scalar(1) / r5;

        const Scalar Re2 = Re * Re;
        const Scalar k_J2 = Scalar(1.5) * J2_coeff * mu * Re2 * inv_r5;

        const Scalar five_z2_over_r2 = Scalar(5) * z * z / r2;

        g[0] += k_J2 * (five_z2_over_r2 - Scalar(1)) * x;
        g[1] += k_J2 * (five_z2_over_r2 - Scalar(1)) * y;
        g[2] += k_J2 * (five_z2_over_r2 - Scalar(3)) * z;

        return g;
    }

} // namespace Aetherion::Environment
