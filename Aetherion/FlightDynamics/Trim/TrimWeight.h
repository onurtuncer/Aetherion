// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TrimWeight.h
//
// Single source of truth for the weight fed to TrimSolver through
// TrimInputs::weight_lbf.
//
// Rationale
// ─────────
// The six-DoF integrator pulls the vehicle down with J2GravityPolicy — the
// WGS-84 J2 field evaluated at the vehicle's own position.  A weight built
// instead from the standard sea-level constant g₀ = 9.80665 m/s² makes the
// trim solution balance a force the integrator never applies, so the vehicle
// departs from its "trimmed" state on the first step.
//
// The bias is not negligible, and it lands directly on α, elevator and
// throttle.  At the NASA TM-2015-218675 check-case position (36.019° N,
// 10 013 ft) the J2 field gives 9.81108 m/s² — 0.045 % above g₀ — and near the
// pole (89.95° N, 10 000 ft) it gives 9.83226 m/s², 0.26 % above.  The local
// field is also what the NASA reference itself uses: 32.18857 ft/s² here,
// against the 32.18876 ft/s² behind the reference weight of 20 509.4 lbf.
//
// Longitude and Earth rotation angle are deliberately absent from the
// interface.  The J2 field is axisymmetric about the polar axis, so |g|
// depends only on geodetic latitude and altitude: rotating the position about
// z — all that longitude, or the ECEF → ECI transform, does — leaves both the
// geocentric radius and the z component unchanged, and hence |g| too.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Environment/Gravity.h>

#include <cmath>
#include <numbers>

namespace Aetherion::FlightDynamics {

/// @brief Pound-force in newtons — matches TrimSolver::kLbf_N.
inline constexpr double kTrimLbf_N = 4.448221615260751;

/// @brief Magnitude of the WGS-84 J2 gravitational acceleration at a geodetic
///        position.
///
/// Evaluates the same field as @c J2GravityPolicy, so a weight built from this
/// value is consistent with what the integrator applies at the trim point.
///
/// @param lat_deg Geodetic latitude [deg].
/// @param alt_m   Ellipsoidal height above the WGS-84 ellipsoid [m].
/// @return Gravitational acceleration magnitude [m/s²].
inline double LocalGravityMagnitude_m_s2(double lat_deg, double alt_m)
{
    constexpr double kDegRad = std::numbers::pi / 180.0;

    // Longitude is arbitrary — see the axisymmetry note in the file header.
    const auto r = Coordinate::GeodeticToECEF(lat_deg * kDegRad, 0.0, alt_m);
    const auto g = Environment::J2(Environment::Vec3<double>{ r[0], r[1], r[2] });

    return std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
}

/// @brief Aircraft weight at a geodetic trim position, in the units
///        @c TrimInputs::weight_lbf expects.
///
/// @param mass_kg Vehicle mass [kg].
/// @param lat_deg Geodetic latitude of the trim point [deg].
/// @param alt_m   Ellipsoidal height of the trim point [m].
/// @return Weight [lbf].
inline double TrimWeight_lbf(double mass_kg, double lat_deg, double alt_m)
{
    return mass_kg * LocalGravityMagnitude_m_s2(lat_deg, alt_m) / kTrimLbf_N;
}

} // namespace Aetherion::FlightDynamics
