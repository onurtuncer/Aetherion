// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody {

/// @brief Initial angular velocity of the rigid body expressed in the body frame.
///
/// Components follow the standard body-axis convention (x = roll, y = pitch, z = yaw).
/// All values are in radians per second and default to zero (non-rotating).
struct BodyRates
{
    double roll_rad_s{ 0.0 };  ///< Roll rate about the body x-axis [rad/s].
    double pitch_rad_s{ 0.0 }; ///< Pitch rate about the body y-axis [rad/s].
    double yaw_rad_s{ 0.0 };   ///< Yaw rate about the body z-axis [rad/s].
};

} // namespace Aetherion::RigidBody
