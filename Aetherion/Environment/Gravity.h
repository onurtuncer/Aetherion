// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstddef>
#include <cmath>

namespace Aetherion::Environment {

    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

    namespace detail {

        template <class S>
        inline S SquareRoot(const S& x) {
            using std::sqrt;
            return sqrt(x);
        }

    } // namespace detail

    /// Central Newtonian gravity (point-mass Earth) in an inertial/ECI frame.
    ///
    /// Input:
    ///   r_W  : position [m] of body in ECI/inertial frame W, from Earth's center.
    /// Constants:
    ///   mu   : Earth's GM [m^3/s^2] (default: WGS-84 value)
    ///
    /// Output:
    ///   g_W  : gravitational acceleration [m/s^2] in frame W.
    template <class Scalar>
    inline Vec3<Scalar> CentralGravity(
        const Vec3<Scalar>& r_W,
        const Scalar& mu = Scalar(3.986004418e14)) // [m^3/s^2]
    {
        using detail::SquareRoot;

        const Scalar x = r_W[0];
        const Scalar y = r_W[1];
        const Scalar z = r_W[2];

        const Scalar r2 = x * x + y * y + z * z;
        const Scalar r = SquareRoot(r2);
        const Scalar r3 = r2 * r;         // r^3
        const Scalar inv_r3 = Scalar(1) / r3;

        const Scalar k = -mu * inv_r3;

        return Vec3<Scalar>{k* x, k* y, k* z};
    }

    /// Central gravity *with J2* perturbation.
    ///
    /// Input:
    ///   r_W  : position [m] in ECI/inertial frame W.
    ///   mu   : GM [m^3/s^2]
    ///   Re   : mean equatorial radius [m]
    ///   J2   : 2nd zonal harmonic [-]
    ///
    /// Output:
    ///   g_W  : acceleration [m/s^2] including J2.
    template <class Scalar>
    inline Vec3<Scalar> J2(
        const Vec3<Scalar>& r_W,
        const Scalar& mu = Scalar(3.986004418e14),
        const Scalar& Re = Scalar(6378137.0),
        const Scalar& J2 = Scalar(1.08262668e-3))
    {
        using detail::SquareRoot;

        const Scalar x = r_W[0];
        const Scalar y = r_W[1];
        const Scalar z = r_W[2];

        const Scalar r2 = x * x + y * y + z * z;
        const Scalar r = SquareRoot(r2);
        const Scalar r3 = r2 * r;
        const Scalar r5 = r3 * r2;
        const Scalar inv_r3 = Scalar(1) / r3;
        const Scalar inv_r5 = Scalar(1) / r5;

        // --- Central term ---
        const Scalar k_c = -mu * inv_r3;
        Scalar gx = k_c * x;
        Scalar gy = k_c * y;
        Scalar gz = k_c * z;

        // --- J2 perturbation ---
        const Scalar z2 = z * z;
        const Scalar Re2 = Re * Re;
        const Scalar k_J2 = Scalar(1.5) * J2 * mu * Re2 * inv_r5;

        const Scalar factor_xy = Scalar(5) * z2 / r2 - Scalar(1);
        const Scalar factor_z = Scalar(5) * z2 / r2 - Scalar(3);

        gx += k_J2 * factor_xy * x;
        gy += k_J2 * factor_xy * y;
        gz += k_J2 * factor_z * z;

        return Vec3<Scalar>{gx, gy, gz};
    }

} // namespace Aetherion::Environment
