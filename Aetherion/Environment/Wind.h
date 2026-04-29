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

/// @brief Altitude-varying wind profile specification (NED frame at h_ref_m).
///
/// Wind speed at altitude h scales as (h / h_ref_m)^shear_exp.
/// Use with WindAwareDragPolicy<PowerLawWindShear>.
struct WindShear
{
    double north_ref_mps{ 0.0 };   ///< North wind at h_ref_m [m/s].
    double east_ref_mps { 0.0 };   ///< East  wind at h_ref_m [m/s].
    double down_ref_mps { 0.0 };   ///< Down  wind at h_ref_m [m/s] (usually zero).
    double h_ref_m      { 9144.0 };///< Reference altitude [m].
    double shear_exp    { 1.3333 };///< Power-law exponent (4/3 ≈ 1.333 fits Scenario 8).
};

} // namespace Aetherion::Environment
