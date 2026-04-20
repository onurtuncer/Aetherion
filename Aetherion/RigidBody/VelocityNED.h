// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody {

/// @brief Initial velocity of the rigid body expressed in the local NED frame.
///
/// Components follow the standard North-East-Down sign convention.
/// All values are in metres per second and default to zero (stationary).
struct VelocityNED
{
    double north_mps{ 0.0 }; ///< Northward velocity component [m/s].
    double east_mps{ 0.0 };  ///< Eastward velocity component [m/s].
    double down_mps{ 0.0 };  ///< Downward velocity component [m/s]; positive pointing toward Earth centre.
};

} // namespace Aetherion::RigidBody
