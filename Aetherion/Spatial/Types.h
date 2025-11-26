// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <cppad/cppad.hpp>
#include <Eigen/Dense>

namespace Aetherion::Spatial {

	using CppAD::AD;

	using Twist = Eigen::Matrix<AD<double>, 6, 1>;
	using Wrench = Eigen::Matrix<AD<double>, 6, 1>;
	using Inertia = Eigen::Matrix<AD<Scalar>, 6, 6>;
} // namespace Aetherion::Spatial