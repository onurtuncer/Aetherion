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

/// @brief 3-element position/acceleration vector; compatible with CppAD scalar types.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
template <class Scalar>
using Vec3 = std::array<Scalar, 3>;

/// @brief Point-mass (Newtonian) gravitational acceleration in the ECI inertial frame.
///
/// Computes @f$ \mathbf{g} = -\frac{\mu}{r^3}\,\mathbf{r} @f$ where
/// @f$ r = \|\mathbf{r}\| @f$ is the geocentric distance.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param r_W Position vector from Earth's centre expressed in ECI [m].
/// @param mu Geocentric gravitational parameter [m³/s²]; defaults to the WGS-84 value.
/// @return Gravitational acceleration vector in ECI [m/s²].
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

/// @brief Gravitational acceleration including the WGS-84 J2 oblateness perturbation.
///
/// Adds the second zonal harmonic correction to the central-gravity term:
/// @f[
///   \mathbf{g}_{J2} = \mathbf{g}_0 + \frac{3}{2}\frac{J_2\,\mu\,R_e^2}{r^5}
///   \begin{bmatrix}
///     \left(\tfrac{5z^2}{r^2}-1\right)x \\
///     \left(\tfrac{5z^2}{r^2}-1\right)y \\
///     \left(\tfrac{5z^2}{r^2}-3\right)z
///   \end{bmatrix}
/// @f]
/// where @f$ \mathbf{g}_0 @f$ is the central-gravity term.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param r_W Position vector from Earth's centre expressed in ECI [m].
/// @param mu Geocentric gravitational parameter [m³/s²]; defaults to the WGS-84 value.
/// @param Re Earth equatorial radius [m]; defaults to the WGS-84 semi-major axis.
/// @param J2_coeff Second zonal harmonic coefficient (dimensionless); defaults to the WGS-84 value.
/// @return Total gravitational acceleration (central + J2 perturbation) in ECI [m/s²].
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
