// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// GeometricAltitude.h
//
// Computes geometric altitude above the WGS-84 ellipsoid from an ECI position
// vector, using a first-order flattening correction.
//
// Problem with the naive formula  h ≈ |r| − a
// ─────────────────────────────────────────────
// At the equator the geocentric surface radius equals the semi-major axis a, so
// the approximation is exact there.  At mid-latitudes the geocentric surface
// radius is 7–21 km shorter than a.  Subtracting the equatorial radius gives
// large negative "altitudes" for any non-equatorial location below ~20 km:
//   Kitty Hawk (36 °N, h = 3 052 m) → naive formula gives −4 253 m.
//
// Corrected formula
// ─────────────────
// The geocentric surface radius at geocentric latitude λ_gc = arcsin(rz / |r|)
// is approximated to first order in the flattening f:
//
//   r_surface(λ_gc) ≈ a × (1 − f × sin²(λ_gc))
//
// giving a geometric altitude error of ≈ 30 m for any latitude and altitude
// below 80 km — more than adequate for atmospheric property lookup.
//
// AD compatibility: the template parameter S may be double or
// CppAD::AD<double>.  All operations are elementary and tape-recordable.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Environment/WGS84.h>
#include <Eigen/Dense>

namespace Aetherion::Environment {

/// @brief Geometric altitude above the WGS-84 ellipsoid [m], given ECI position.
///
/// Uses a first-order flattening correction to remove the systematic offset
/// that arises from subtracting the equatorial radius from the geocentric
/// distance at non-equatorial latitudes.
///
/// @tparam S  Scalar type (@c double or @c CppAD::AD<double>).
/// @param  r_eci  ECI position vector [m].
/// @return        Geometric altitude above the WGS-84 ellipsoid [m].
///                Negative values indicate a position below the surface
///                (physically impossible in flight; the atmosphere model clamps).
template<class S>
inline S GeometricAltitude_m(const Eigen::Matrix<S, 3, 1>& r_eci)
{
    const S r         = r_eci.norm();
    const S sin_gc    = r_eci(2) / r;                         // sin(geocentric lat)
    const S r_surface = S(WGS84::kSemiMajorAxis_m)
                      * (S(1.0) - S(WGS84::kFlattening) * sin_gc * sin_gc);
    return r - r_surface;
}

} // namespace Aetherion::Environment
