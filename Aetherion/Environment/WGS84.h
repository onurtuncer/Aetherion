// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// WGS84.h
//
// Single authoritative source for all WGS-84 physical constants used throughout
// Aetherion.  Every other file that previously defined these inline should be
// updated to include this header and use the constants below.
//
// References:
//   NIMA TR8350.2, 3rd ed. (2000) — "Department of Defense World Geodetic System 1984"
//   WGS-84 Implementation Manual v2.4 (1997)
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::Environment::WGS84 {

    // ── Geometric parameters --------------------------------------------------

    /// Semi-major (equatorial) axis  [m]
    inline constexpr double kSemiMajorAxis_m = 6'378'137.0;

    /// Flattening  f = (a − b) / a  [dimensionless]
    inline constexpr double kFlattening = 1.0 / 298.257'223'563;

    /// First eccentricity squared  e^2 = f(2−f)  [dimensionless]
    inline constexpr double kEccentricitySq =
        kFlattening * (2.0 - kFlattening);

    // ── Gravitational parameters ----------------------------------------------

    /// Geocentric gravitational parameter  GM  [m3/s2]
    inline constexpr double kGM_m3_s2 = 3.986'004'418e14;

    /// Second zonal harmonic (oblateness)  J2  [dimensionless]
    inline constexpr double kJ2 = 1.082'626'68e-3;

    // ── Earth rotation ---------------------------------------------------------

    /// Earth rotation rate  we  [rad/s]
    inline constexpr double kRotationRate_rad_s = 7.292'115'0e-5;

} // namespace Aetherion::Environment::WGS84