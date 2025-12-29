// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics
{
	template<class Scalar>
	struct FlightModelT {
		using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
		using Vec = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

		ParamsT<Scalar> params;

		void rhs(Scalar t, const Vec& x, Vec& dxdt) const;
	};

} // namespace Aetherion::FlightDynamics