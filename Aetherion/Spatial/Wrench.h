// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::Spatial {

	template<typename Scalar>
	struct Wrench {
		Eigen::Matrix<Scalar, 6, 1> f; // [M_x,M_y,M_z, F_x,F_y,F_z]^T
	};

} // namespace Aetherion::Spatial
