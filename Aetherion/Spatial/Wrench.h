// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::Spatial {

/// @brief Spatial force (wrench) following Featherstone conventions.
///
/// Stores the 6-D generalised force as a single column vector:
/// @f[ \mathbf{f} = [M_x,\, M_y,\, M_z,\, F_x,\, F_y,\, F_z]^\top @f]
/// where @f$\mathbf{M}@f$ is the moment [N·m] (indices 0–2) and
/// @f$\mathbf{F}@f$ is the force [N] (indices 3–5).
///
/// @note The storage order is **moment-first** (dual to @c Twist angular-first),
///       consistent with the @f$ P = \mathbf{f}^\top \mathbf{v} @f$ inner product.
///
/// @tparam Scalar Numeric type (e.g. @c double or @c CppAD::AD<double>).
	template<typename Scalar>
	struct Wrench {
		/// Packed 6-vector @f$[M; F]@f$: indices 0–2 moment [N·m], 3–5 force [N].
		Eigen::Matrix<Scalar, 6, 1> f; // [M_x,M_y,M_z, F_x,F_y,F_z]^T
	};

} // namespace Aetherion::Spatial
