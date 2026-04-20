// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody{

/// @brief Constant aerodynamic coefficients for a rigid vehicle.
///
/// These are the stability-axis (or body-axis, depending on policy)
/// non-dimensional force and moment coefficients used by aerodynamic
/// wrench computation routines.
	struct AerodynamicParameters {
		double S{ 0.0 };  ///< Reference area [m²].
		double CL{ 0.0 }; ///< Lift coefficient [-].
		double CD{ 0.0 }; ///< Drag coefficient [-].
		double CY{ 0.0 }; ///< Side-force coefficient [-].
		double Cl{ 0.0 }; ///< Roll moment coefficient [-].
		double Cm{ 0.0 }; ///< Pitch moment coefficient [-].
		double Cn{ 0.0 }; ///< Yaw moment coefficient [-].
	};

} // namespace Aetherion::RigidBody::Parameters

