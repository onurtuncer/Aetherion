// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "Aetherion/Spatial/Skew.h"
#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Momentum.h"
#include "Aetherion/Spatial/Transform.h"

namespace Aetherion::Spatial {

    template<typename Scalar>
    struct Inertia
    {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

        Scalar mass;   // [kg]
        Mat3   I_com;  // Inertia about CoM [kg*m^2]
        Vec3   c;      // CoM offset [m] (expressed in body frame)

        // Cached 6x6 spatial inertia
        Mat6   M;

        Inertia(
            Scalar m,
            const Mat3& I_C,
            const Vec3& c_C)
            : mass(m), I_com(I_C), c(c_C)
        {
            build();
        }

        // ---------------------------------------------------------------------
        // Build spatial inertia matrix using parallel axis theorem
        // ---------------------------------------------------------------------
        void build()
        {
            const Mat3 C = skew(c);

            const Mat3 I_shifted =
                I_com + mass * C * C.transpose();

            M.setZero();

            M.template block<3, 3>(0, 0) = I_shifted;
            M.template block<3, 3>(0, 3) = mass * C;
            M.template block<3, 3>(3, 0) = -mass * C;
            M.template block<3, 3>(3, 3) = mass * Mat3::Identity();
        }

        // ---------------------------------------------------------------------
        // Spatial momentum: h = I * v
        // ---------------------------------------------------------------------
        Momentum<Scalar> operator*(const Twist<Scalar>& v) const
        {
            Momentum<Scalar> out{};
            out.h = M * v.v;
            return out;
        }

        // ---------------------------------------------------------------------
        // Return underlying matrix (if needed)
        // ---------------------------------------------------------------------
        const Mat6& matrix() const { return M; }

        // ---------------------------------------------------------------------
        // Apply spatial motion transform
        // I' = X I Xᵀ
        // ---------------------------------------------------------------------
        void applyMotionTransform(
            const Eigen::Matrix<Scalar, 3, 3>& R,
            const Eigen::Matrix<Scalar, 3, 1>& r)
        {
            const Mat6 X = MotionTransformMatrix(R, r);
            M = X * M * X.transpose();
        }

        // ---------------------------------------------------------------------
        // Energy (kinetic)
        // T = 1/2 vᵀ I v
        // ---------------------------------------------------------------------
        Scalar kineticEnergy(const Twist<Scalar>& v) const
        {
            return Scalar(0.5) * v.v.dot(M * v.v);
        }
    };

} // namespace Aetherion::Spatial



/*#pragma once

#include "Aetherion/FlightDynamics/InertialParameters.h"
#include "Aetherion/Spatial/Skew.h"

namespace Aetherion::Spatial {

	template<typename Scalar>
	struct Inertia {
		Scalar mass; // [kg]
		Eigen::Matrix<Scalar, 3, 3> I_com; // W.r.t c.o.g [kg.m2]
		Eigen::Matrix<Scalar, 3, 1> c; // CG offset [m]
		Eigen::Matrix<Scalar, 6, 6> M; // Spatial inertia [kg.m2]
		Inertia(Scalar m,
			const Eigen::Matrix<Scalar, 3, 3>& I_C,
			const Eigen::Matrix<Scalar, 3, 1>& c_C)
			: mass(m), I_com(I_C), c(c_C) {
			build6x6();
		}
		// Constructs 6x6 spatial inertia matrix (parallel axis theorem)
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
		// Turns 3x3 block sub-matrices
		std::array<Eigen::Matrix<Scalar, 3, 3>, 4> toBlock() const {
			return { M.topLeftCorner<3,3>(), M.topRightCorner<3,3>(),
			M.bottomLeftCorner<3,3>(), M.bottomRightCorner<3,3>() };
		}

		// Apply spatial transform X (6x6): M = X * M * X^T
		void applyTransform(const Eigen::Matrix<Scalar, 6, 6>& X) {
			M = X * M * X.transpose();
		}

		// Spatial momentum: p = M * v
		Eigen::Matrix<Scalar, 6, 1> operator*(const Eigen::Matrix<Scalar, 6, 1>& v)
			const {
			return M * v;
		}
	};

} // namespace Aetherion::Spatial */