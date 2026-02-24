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
	struct Velocity {
		Eigen::Matrix<Scalar, 6, 1> v; // [?_x,?_y,?_z, v_x,v_y,v_z]^T
	};

} // namespace Aetherion::Spatial
