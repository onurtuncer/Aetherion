// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::Environment {

/// @brief Constant ambient wind velocity in the local NED frame.
///
/// A positive East component means the wind blows eastward (meteorological
/// "westerly").  All components default to zero (calm atmosphere).
struct ConstantWind
{
    double north_mps{ 0.0 }; ///< Northward wind component [m/s].
    double east_mps { 0.0 }; ///< Eastward  wind component [m/s].
    double down_mps { 0.0 }; ///< Downward  wind component [m/s] (usually zero).
};

/// @brief Linear altitude wind-shear configuration (NED frame).
///
/// Wind at geocentric altitude h [m]:
///   v_N(h) = gradient_N_mps_m * h + intercept_N_mps
///   v_E(h) = gradient_E_mps_m * h + intercept_E_mps
///
/// NASA TM-2015-218675 Scenario 8:
///   v_E(h) = 0.003 * h_m - 6.096  m/s   (= (0.003*h_ft - 20) ft/s from west)
///   gradient_E_mps_m = 0.003
///   intercept_E_mps  = -6.096
///
/// Use with WindAwareDragPolicy<LinearWindShear>.
struct WindShear
{
    double gradient_N_mps_m { 0.0 }; ///< North wind altitude gradient [m/s per m].
    double gradient_E_mps_m { 0.0 }; ///< East  wind altitude gradient [m/s per m].
    double intercept_N_mps  { 0.0 }; ///< North wind at h = 0 m (sea level) [m/s].
    double intercept_E_mps  { 0.0 }; ///< East  wind at h = 0 m (sea level) [m/s].
};

} // namespace Aetherion::Environment
