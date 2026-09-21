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
//
// Apparent weight in level flight
// ───────────────────────────────
// The J2 field is mass attraction only, and the integrator works in ECI.  A
// vehicle holding constant geodetic altitude is therefore not unaccelerated: it
// rides a curved path around a rotating Earth, and the centripetal acceleration
// of that path is supplied by gravity.  Only the remainder has to be carried by
// the wing:
//
//   g_app = G_D − V_N²/(M+h) − (V_E + Ω(N+h)cos φ)²/(N+h)
//
// with G_D the J2 attraction along the geodetic vertical, M and N the meridian
// and prime-vertical radii of curvature, and V_N, V_E the Earth-relative north
// and east velocity.  Expanding the square gives the three familiar terms:
// Ω²(N+h)cos²φ (centrifugal, the difference between attraction and plumb-bob
// gravity), 2ΩV_E cos φ (Eötvös) and V_E²/(N+h) (path curvature).
//
// Trimming against the static weight m·|g| instead leaves the vehicle with
// excess lift at t = 0.  At the NASA Scenario-11 point (M 0.53, heading 045°)
// the relief is 0.42 % of the weight — 86 lbf, +0.017° of α; at the Scenario-12
// point (M 2.0) it is 1.34 % — 275 lbf.  LevelFlightTrimWeight_lbf() reproduces
// the body-Z aerodynamic force of the altitude-holding NESC reference
// simulations to better than 1 lbf in both.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Environment/Gravity.h>
#include <Aetherion/Environment/WGS84.h>

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

/// @brief Meridian (M) and prime-vertical (N) radii of curvature of the WGS-84
///        ellipsoid at a geodetic latitude [m].
struct CurvatureRadii {
    double meridian_m{};       ///< M — north–south
    double primeVertical_m{};  ///< N — east–west
};

inline CurvatureRadii WGS84CurvatureRadii(double lat_rad)
{
    namespace WGS84 = Environment::WGS84;

    const double sinLat = std::sin(lat_rad);
    const double w2     = 1.0 - WGS84::kEccentricitySq * sinLat * sinLat;
    const double N      = WGS84::kSemiMajorAxis_m / std::sqrt(w2);
    return { N * (1.0 - WGS84::kEccentricitySq) / w2, N };
}

/// @brief Specific force a vehicle must generate, along the geodetic vertical,
///        to hold constant geodetic altitude at a given Earth-relative velocity.
///
/// J2 attraction along the geodetic vertical, less the centripetal acceleration
/// of a constant-altitude path over the rotating WGS-84 ellipsoid — see the
/// file header.  At rest this is plumb-bob gravity; it falls with eastward
/// speed and rises with westward speed (Eötvös effect).
///
/// @param lat_deg    Geodetic latitude [deg].
/// @param alt_m      Ellipsoidal height above the WGS-84 ellipsoid [m].
/// @param vNorth_mps Earth-relative north velocity [m/s].
/// @param vEast_mps  Earth-relative east velocity [m/s].
/// @return Apparent gravitational acceleration [m/s²].
inline double LevelFlightApparentGravity_m_s2(double lat_deg, double alt_m,
                                              double vNorth_mps, double vEast_mps)
{
    namespace WGS84 = Environment::WGS84;
    constexpr double kDegRad = std::numbers::pi / 180.0;

    const double lat    = lat_deg * kDegRad;
    const double sinLat = std::sin(lat);
    const double cosLat = std::cos(lat);

    // J2 attraction projected on geodetic down, (−cos φ, 0, −sin φ) at zero
    // longitude — arbitrary by the axisymmetry noted in the file header.
    const auto r = Coordinate::GeodeticToECEF(lat, 0.0, alt_m);
    const auto g = Environment::J2(Environment::Vec3<double>{ r[0], r[1], r[2] });
    const double gDown = -(g[0] * cosLat + g[2] * sinLat);

    const auto [M, N] = WGS84CurvatureRadii(lat);

    // East velocity seen from ECI: Earth-relative plus the local surface speed
    const double vEastInertial = vEast_mps
                               + WGS84::kRotationRate_rad_s * (N + alt_m) * cosLat;

    return gDown
         - vNorth_mps * vNorth_mps / (M + alt_m)
         - vEastInertial * vEastInertial / (N + alt_m);
}

/// @brief Apparent aircraft weight in level flight at a geodetic trim position
///        and Earth-relative velocity, in the units @c TrimInputs::weight_lbf
///        expects.
///
/// This — not the static TrimWeight_lbf() — is the weight a level-flight trim
/// has to balance for the vehicle to start the integration with zero vertical
/// acceleration.
///
/// @param mass_kg    Vehicle mass [kg].
/// @param lat_deg    Geodetic latitude of the trim point [deg].
/// @param alt_m      Ellipsoidal height of the trim point [m].
/// @param vNorth_mps Earth-relative north velocity at trim [m/s].
/// @param vEast_mps  Earth-relative east velocity at trim [m/s].
/// @return Apparent weight [lbf].
inline double LevelFlightTrimWeight_lbf(double mass_kg, double lat_deg, double alt_m,
                                        double vNorth_mps, double vEast_mps)
{
    return mass_kg
         * LevelFlightApparentGravity_m_s2(lat_deg, alt_m, vNorth_mps, vEast_mps)
         / kTrimLbf_N;
}

} // namespace Aetherion::FlightDynamics
