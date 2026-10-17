// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// SE23.h
//
// The group of extended poses SE2(3): the navigation core of the estimator
// (sensor fusion step 5, doc/sensor_fusion.rst "State space").
//
//   X = [ R  v  p ]      R in SO(3) (body -> LCI), v, p in R^3 (LCI)
//       [ 0  1  0 ]
//       [ 0  0  1 ]
//
// Composition is matrix multiplication, X1 X2 = (R1 R2, R1 v2 + v1,
// R1 p2 + p1). A tangent vector is xi = (phi, nu, rho) in R^9, in that
// order (Blocks.h, NavCoreLayout), and
//
//   Exp(xi) = (Exp(phi), J_l(phi) nu, J_l(phi) rho)
//
// with the SO(3) exponential and left Jacobian of ODE/RKMK/Lie/SO3.h. Scalar
// templated: double for the reference filter, CppAD::AD<double> for
// Jacobians and the code generator; every branch goes through CondExp.
// The rotation is stored as a matrix only (no quaternion), so the type is
// plain enough to tape.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <cppad/example/cppad_eigen.hpp> // Eigen NumTraits for CppAD::AD

#include <Aetherion/ODE/RKMK/Lie/SO3.h>
#include <Aetherion/Spatial/Skew.h>

namespace Aetherion::Estimation {

    template <class S>
    struct SE23
    {
        using Mat3 = Eigen::Matrix<S, 3, 3>;
        using Vec3 = Eigen::Matrix<S, 3, 1>;
        using Tangent = Eigen::Matrix<S, 9, 1>; ///< (phi, nu, rho)
        using Mat9 = Eigen::Matrix<S, 9, 9>;

        Mat3 R = Mat3::Identity(); ///< Attitude, body -> LCI
        Vec3 v = Vec3::Zero();     ///< Velocity in LCI [m/s]
        Vec3 p = Vec3::Zero();     ///< Position in LCI [m]

        SE23() = default;
        SE23(Mat3 R_in, Vec3 v_in, Vec3 p_in)
            : R(std::move(R_in))
            , v(std::move(v_in))
            , p(std::move(p_in))
        {}

        [[nodiscard]] static SE23 Identity() { return SE23{}; }

        /// X1 X2 = (R1 R2, R1 v2 + v1, R1 p2 + p1)
        friend SE23 operator*(const SE23& a, const SE23& b) { return SE23(a.R * b.R, a.R * b.v + a.v, a.R * b.p + a.p); }

        /// X^-1 = (R^T, -R^T v, -R^T p)
        [[nodiscard]] SE23 inverse() const
        {
            const Mat3 Rt = R.transpose();
            return SE23(Rt, -(Rt * v), -(Rt * p));
        }

        [[nodiscard]] static SE23 Exp(const Tangent& xi)
        {
            namespace SO3 = ODE::RKMK::Lie::SO3;
            const Vec3 phi = xi.template segment<3>(0);
            const Mat3 J = SO3::LeftJacobian(phi);
            return SE23(SO3::Exp_R(phi), J * xi.template segment<3>(3), J * xi.template segment<3>(6));
        }

        /// Inverse of Exp: (Log(R), J_l(phi)^-1 v, J_l(phi)^-1 p). Intended for
        /// small group elements (errors); see SO3::Log_R for the theta = pi caveat.
        [[nodiscard]] static Tangent Log(const SE23& X)
        {
            namespace SO3 = ODE::RKMK::Lie::SO3;
            const Vec3 phi = SO3::Log_R(X.R);
            const Mat3 Jinv = SO3::LeftJacobianInverse(phi);
            Tangent xi;
            xi.template segment<3>(0) = phi;
            xi.template segment<3>(3) = Jinv * X.v;
            xi.template segment<3>(6) = Jinv * X.p;
            return xi;
        }

        /// Adjoint, Ad_X xi^ = X xi^ X^-1:
        ///   [ R         0  0 ]
        ///   [ [v]x R    R  0 ]
        ///   [ [p]x R    0  R ]
        /// It maps left-invariant to right-invariant error coordinates,
        /// xi_R = Ad_Xhat xi_L (doc/sensor_fusion.rst "First-order equivalence").
        [[nodiscard]] static Mat9 Adjoint(const SE23& X)
        {
            Mat9 Ad = Mat9::Zero();
            Ad.template block<3, 3>(0, 0) = X.R;
            Ad.template block<3, 3>(3, 0) = Spatial::skew(X.v) * X.R;
            Ad.template block<3, 3>(3, 3) = X.R;
            Ad.template block<3, 3>(6, 0) = Spatial::skew(X.p) * X.R;
            Ad.template block<3, 3>(6, 6) = X.R;
            return Ad;
        }
    };

} // namespace Aetherion::Estimation
