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

} // namespace Aetherion::Environment
