// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once


#include <Eigen/Dense>

#define CPPAD_CPPAD_HPP_ALREADY_INCLUDED

#include <cppad/cppad.hpp>

namespace Aetherion::Spatial {

	using CppAD::AD;

	using Twist = Eigen::Matrix<AD<double>, 6, 1>;
	using Wrench = Eigen::Matrix<AD<double>, 6, 1>;
	using Inertia = Eigen::Matrix<AD<Scalar>, 6, 6>;
} // namespace Aetherion::Spatial