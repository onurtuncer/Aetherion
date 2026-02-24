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
	struct SpatialInertia {
		Scalar mass; // [kg]
		Eigen::Matrix<Scalar, 3, 3> I_com; // Ağırlık merkezine göre [kg·m²]
		Eigen::Matrix<Scalar, 3, 1> c; // CG ofset [m]
		Eigen::Matrix<Scalar, 6, 6> M; // Uzaysal eylemsizlik [kg·m²]
		SpatialInertia(Scalar m,
			const Eigen::Matrix<Scalar, 3, 3>& I_C,
			const Eigen::Matrix<Scalar, 3, 1>& c_C)
			: mass(m), I_com(I_C), c(c_C) {
			build6x6();
		}
		// 6x6 eylemsizlik matrisini oluşturur (paralel eksen teoremi)
		void build6x6() {
			Eigen::Matrix<Scalar, 3, 3> C = skew(c);
			Eigen::Matrix<Scalar, 3, 3> I_part = I_com + mass * C * C.transpose();
			Eigen::Matrix<Scalar, 3, 3> zero = Eigen::Matrix<Scalar, 3, 3>::Zero();
			Eigen::Matrix<Scalar, 3, 3> I3 = Eigen::Matrix<Scalar, 3, 3>::Identity();
			M.setZero();
			M.topLeftCorner<3, 3>() = I_part;
			M.topRightCorner<3, 3>() = mass * C;
			M.bottomLeftCorner<3, 3>() = -mass * C;
			M.bottomRightCorner<3, 3>() = mass * I3;
		}
		// Bloğa ayrılmış 3x3 alt matrisleri döndürür
		std::array<Eigen::Matrix<Scalar, 3, 3>, 4> toBlock() const {
			return { M.topLeftCorner<3,3>(), M.topRightCorner<3,3>(),
			M.bottomLeftCorner<3,3>(), M.bottomRightCorner<3,3>() };
		}

		// Uzaysal dönüşüm X (6x6) uygulama: M = X * M * X^T
		void applyTransform(const Eigen::Matrix<Scalar, 6, 6>& X) {
			M = X * M * X.transpose();
		}

		// Uzaysal momentum: p = M * v
		Eigen::Matrix<Scalar, 6, 1> operator*(const Eigen::Matrix<Scalar, 6, 1>& v)
			const {
			return M * v;
		}
	};

} // namespace Aetherion::Spatial