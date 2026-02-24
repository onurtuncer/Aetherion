// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <Aetherion/Spatial/Inertia.h>

TEST_CASE("SpatialMomentum and derivative", "[spatial]") {
	using ADd = CppAD::AD<double>;

	Eigen::Matrix<ADd, 3, 1> c; CppAD::Independent(c);
	Eigen::Matrix<ADd, 3, 3> I = Eigen::Matrix<ADd, 3, 3>::Identity();
	SpatialInertia<ADd> SI((ADd)2.0, I, c);
	SI.build6x6();
	Eigen::Matrix<ADd, 6, 1> v; v << 1, 0, 0, 0, 1, 0;
	auto p = SI * v;
	// Derivative of spatial momentum w.r.t. c (Analytical counterpart can be checked)
	
	std::vector<ADd> y = { p(0), p(1), p(2), p(3), p(4), p(5) };
	CppAD::ADFun<double> f(c, y);
	auto J = f.Jacobian({ 0.1,0.2,0.3 });
	
		REQUIRE(J.rows() == 6); REQUIRE(J.cols() == 3);
}