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
/// Covers static force/moment coefficients plus rotary damping derivatives
/// used by aerodynamic wrench policies (DragOnlyAeroPolicy,
/// BrickDampingAeroPolicy, etc.).
///
/// The rate-derivative fields (b, cbar, Clp, Clr, Cmq, Cnp, Cnr) default
/// to zero so that existing configurations (which omit them) remain valid.
	struct AerodynamicParameters {
		// ── Reference geometry ────────────────────────────────────────────
		double S   { 0.0 }; ///< Reference area [m²].
		double b   { 0.0 }; ///< Span reference length [m] (roll / yaw scaling).
		double cbar{ 0.0 }; ///< Chord reference length [m] (pitch scaling).

		// ── Static force / moment coefficients ────────────────────────────
		double CL{ 0.0 }; ///< Lift coefficient [-].
		double CD{ 0.0 }; ///< Drag coefficient [-].
		double CY{ 0.0 }; ///< Side-force coefficient [-].
		double Cl{ 0.0 }; ///< Roll moment coefficient [-].
		double Cm{ 0.0 }; ///< Pitch moment coefficient [-].
		double Cn{ 0.0 }; ///< Yaw moment coefficient [-].

		// ── Rotary damping derivatives ────────────────────────────────────
		double Clp{ 0.0 }; ///< Roll-moment / roll-rate derivative  [-] (Clp = ∂Cl/∂p̂).
		double Clr{ 0.0 }; ///< Roll-moment / yaw-rate derivative   [-].
		double Cmq{ 0.0 }; ///< Pitch-moment / pitch-rate derivative [-] (Cmq = ∂Cm/∂q̂).
		double Cnp{ 0.0 }; ///< Yaw-moment / roll-rate derivative   [-].
		double Cnr{ 0.0 }; ///< Yaw-moment / yaw-rate derivative    [-] (Cnr = ∂Cn/∂r̂).
	};

} // namespace Aetherion::RigidBody::Parameters

