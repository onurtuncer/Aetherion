// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody
{
/// @brief Flat index layout for a serialised 14-element state vector.
///
/// Used when the full state is packed into a contiguous @f$\mathbb{R}^{14}@f$
/// vector (e.g. for file I/O or finite-difference Jacobians):
/// | Indices | Quantity |
/// |---------|----------|
/// | 0–2     | ECI position @f$\mathbf{p}@f$ [m] |
/// | 3–6     | Unit quaternion attitude @f$\mathbf{q}@f$ (w, x, y, z) |
/// | 7–9     | Body angular velocity @f$\boldsymbol\omega_B@f$ [rad/s] |
/// | 10–12   | Body linear velocity @f$v_B@f$ [m/s] |
/// | 13      | Vehicle mass @f$m@f$ [kg] |
	struct StateLayout {
		static constexpr int IDX_P = 0;  ///< Start index of ECI position (3 elements).
		static constexpr int IDX_Q = 3;  ///< Start index of quaternion attitude (4 elements).
		static constexpr int IDX_W = 7;  ///< Start index of body angular velocity (3 elements).
		static constexpr int IDX_V = 10; ///< Start index of body linear velocity (3 elements).
		static constexpr int IDX_M = 13; ///< Index of vehicle mass (1 element).
		static constexpr int N = 14;     ///< Total number of state elements.
	};

} // namespace Aetherion::RigidBody