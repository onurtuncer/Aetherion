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
// vector, measured along the geocentric radial to the exact ellipsoid surface.
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
// is that of the meridian ellipse, in closed form:
//
//   r_surface(λ_gc) = b / √(1 − e² cos²(λ_gc)),      b = a (1 − f)
//
// What remains is the difference between height along the geocentric radial
// and height along the ellipsoid normal: 2 cm at 3 km, 5 cm at 9 km and 0.5 m
// at 80 km, worst case near 45° latitude.
//
// The first-order expansion r_surface ≈ a (1 − f sin²λ_gc) used here previously
// drops a term of order a·f², which is not small: it reads 24 m low at 36° N
// and 27 m low at 45°, vanishing only at the equator and the poles.  For a
// dropped sphere that is immaterial.  For a trimmed aircraft it is not — the
// trim solver works from the geodetic altitude, so the aero policy saw air
// 0.25 % denser than the trim had balanced: 50 lbf of excess lift on the F-16
// at the NASA Scenario-11 point, as large as the effects the trim is otherwise
// accurate to.
//
// AD compatibility: the template parameter S may be double or
// CppAD::AD<double>.  All operations are elementary and tape-recordable.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <Eigen/Dense>

namespace Aetherion::Environment {

/// @brief Geometric altitude above the WGS-84 ellipsoid [m], given ECI position.
///
/// Subtracts the exact geocentric radius of the ellipsoid surface beneath the
/// vehicle.  Agrees with the geodetic height to better than 0.1 m anywhere
/// below 15 km — see the file header.
///
/// @tparam S  Scalar type (@c double or @c CppAD::AD\<double\>).
/// @param  r_eci  ECI position vector [m].
/// @return        Geometric altitude above the WGS-84 ellipsoid [m].
///                Negative values indicate a position below the surface
///                (physically impossible in flight; the atmosphere model clamps).
template<class S>
inline S GeometricAltitude_m(const Eigen::Matrix<S, 3, 1>& r_eci)
{
    const S r         = r_eci.norm();
    const S sin_gc    = r_eci(2) / r;                         // sin(geocentric lat)
    const S cos2_gc   = S(1.0) - sin_gc * sin_gc;
    const S r_surface = S(WGS84::kSemiMajorAxis_m * (1.0 - WGS84::kFlattening))
                      / detail::SquareRoot(S(1.0) - S(WGS84::kEccentricitySq) * cos2_gc);
    return r - r_surface;
}

} // namespace Aetherion::Environment
