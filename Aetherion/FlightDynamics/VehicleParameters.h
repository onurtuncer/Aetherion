// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace Aetherion::FlightDynamics {

	struct VehicleParams {
		Eigen::Matrix3d J_B = Eigen::Matrix3d::Identity();
	};
} // namespace Aetherion::FlightDynamics








