// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once 

template<class S> Eigen::Matrix<S, 3, 3> skew(const Eigen::Matrix<S, 3, 1>& v) {
	Eigen::Matrix<S, 3, 3> M;
	M << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return M;
}