// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// LoadInertiaFromDAVEML.h
//
// Load RigidBody::InertialParameters from a DAVE-ML inertia model file.
//
// Reads the following standard AIAA DAVE-ML varIDs and converts to SI:
//   XMASS  - total mass         [slug  → kg]
//   XIXX   - Ixx roll  MOI     [slugft2 → kg·m²]
//   XIYY   - Iyy pitch MOI     [slugft2 → kg·m²]
//   XIZZ   - Izz yaw   MOI     [slugft2 → kg·m²]
//   XIZX   - Ixz cross POI     [slugft2 → kg·m²]
//   XIXY   - Ixy cross POI     [slugft2 → kg·m²]
//   XIYZ   - Iyz cross POI     [slugft2 → kg·m²]
//   DXCG   - CG offset, x (FWD positive) [ft → m]
//   DYCG   - CG offset, y (RT  positive) [ft → m]
//   DZCG   - CG offset, z (DN  positive) [ft → m]
//
// If a varID is absent the corresponding field defaults to zero (e.g. for
// symmetric aircraft where Ixy = Iyz = 0 are not listed).
//
// Optional: supply an override for any DAVE-ML input variable (e.g. to
// evaluate the F-16 inertia at a specific CG position).
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/InertialParameters.h>
#include <string>
#include <unordered_map>

namespace Aetherion::Serialization {

/// @brief Load a @c RigidBody::InertialParameters from a DAVE-ML inertia file.
///
/// @param path     Path to the @c .dml / @c .xml DAVE-ML file.
/// @param inputs   Optional map of input varID → value to set before
///                 evaluating calculated outputs (e.g. @c {"CG_PCT_MAC", 35.0}).
///                 The values are interpreted in the units declared in the file.
/// @return         InertialParameters populated in SI units.
/// @throws std::runtime_error  if the file cannot be read or a required
///                             varID evaluation fails.
RigidBody::InertialParameters LoadInertiaFromDAVEML(
    const std::string& path,
    const std::unordered_map<std::string, double>& inputs = {});

} // namespace Aetherion::Serialization
