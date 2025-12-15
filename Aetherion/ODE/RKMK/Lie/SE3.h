// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Lie/SE3.h
//
// Value-semantics SE(3) group element + minimal Lie machinery needed for RKMK:
//  - Identity(), group composition
//  - Exp(xi)  with xi = [omega; v] in se(3)
//  - dexp_inv(x, y) via Bernoulli/ad-series (truncation up to ad^6)
//
// Notes:
//  - Uses Eigen for storage.
//  - Uses Core::so3_ABC / Core::sin_s / Core::cos_s / Core::sqrt_s (AD-friendly).
//  - If you already have an Eigen::NumTraits specialization for CppAD::AD<double>,
//    define AETHERION_EIGEN_NUMTRAITS_CPPAD_AD_DOUBLE_DEFINED before including
//    this header to avoid duplicate specialization.
//
#pragma once

#include <array>
#include <limits>
#include <type_traits>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Scalar.h>

#ifndef AETHERION_EIGEN_NUMTRAITS_CPPAD_AD_DOUBLE_DEFINED
#define AETHERION_EIGEN_NUMTRAITS_CPPAD_AD_DOUBLE_DEFINED
namespace Eigen {
    template<>
    struct NumTraits<CppAD::AD<double>> : NumTraits<double> {
        using Real = CppAD::AD<double>;
        using NonInteger = CppAD::AD<double>;
        using Literal = CppAD::AD<double>;
        using Nested = CppAD::AD<double>;

        enum {
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3
        };

        static inline Real epsilon() { return Real(std::numeric_limits<double>::epsilon()); }
        static inline Real dummy_precision() { return Real(1e-12); }
        static inline Real highest() { return Real((std::numeric_limits<double>::max)()); }
        static inline Real lowest() { return Real((std::numeric_limits<double>::lowest)()); }
    };
} // namespace Eigen
#endif

namespace Aetherion::ODE::RKMK::Lie {

    namespace Core = Aetherion::ODE::RKMK::Core;

    template<class TScalar>
    struct SE3 {
        using Scalar = TScalar;

        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Tangent = Eigen::Matrix<Scalar, 6, 1>; // xi = [omega; v]

        Mat3 R{ Mat3::Identity() };
        Vec3 p{ Vec3::Zero() };

        SE3() = default;
        SE3(const Mat3& R_in, const Vec3& p_in) : R(R_in), p(p_in) {}

        template<class S2>
        explicit SE3(const SE3<S2>& other)
            : R(other.R.template cast<Scalar>())
            , p(other.p.template cast<Scalar>()) {
        }

        [[nodiscard]] static SE3 Identity() { return SE3{}; }

        [[nodiscard]] friend SE3 operator*(const SE3& a, const SE3& b) {
            return SE3{ a.R * b.R, a.p + a.R * b.p };
        }

        [[nodiscard]] SE3 inverse() const {
            const Mat3 Rt = R.transpose();
            return SE3{ Rt, -(Rt * p) };
        }

        [[nodiscard]] static Mat3 skew(const Vec3& w) {
            Mat3 W;
            W << Scalar(0), -w.z(), w.y(),
                w.z(), Scalar(0), -w.x(),
                -w.y(), w.x(), Scalar(0);
            return W;
        }

        [[nodiscard]] static Mat3 so3_exp(const Vec3& w) {
            const Scalar theta2 = w.squaredNorm();

            Scalar A{}, B{}, C{};
            Core::so3_ABC(theta2, A, B, C);
            (void)C;

            const Mat3 W = skew(w);
            const Mat3 W2 = W * W;
            return Mat3::Identity() + A * W + B * W2;
        }

        [[nodiscard]] static Mat3 so3_left_jacobian(const Vec3& w) {
            const Scalar theta2 = w.squaredNorm();

            Scalar A{}, B{}, C{};
            Core::so3_ABC(theta2, A, B, C);
            (void)A;

            const Mat3 W = skew(w);
            const Mat3 W2 = W * W;
            return Mat3::Identity() + B * W + C * W2;
        }

        [[nodiscard]] static Eigen::Matrix<Scalar, 6, 6> ad_matrix(const Tangent& xi) {
            const Vec3 w = xi.template head<3>();
            const Vec3 v = xi.template tail<3>();

            const Mat3 W = skew(w);
            const Mat3 V = skew(v);

            Eigen::Matrix<Scalar, 6, 6> A;
            A.setZero();
            A.template block<3, 3>(0, 0) = W;
            A.template block<3, 3>(3, 0) = V;
            A.template block<3, 3>(3, 3) = W;
            return A;
        }

        [[nodiscard]] static SE3 Exp(const Tangent& xi) {
            const Vec3 w = xi.template head<3>();
            const Vec3 v = xi.template tail<3>();

            const Mat3 R = so3_exp(w);
            const Mat3 J = so3_left_jacobian(w);
            const Vec3 p = J * v;

            return SE3{ R, p };
        }

        [[nodiscard]] static Tangent dexp_inv(const Tangent& x, const Tangent& y) {
            const Eigen::Matrix<Scalar, 6, 6> A = ad_matrix(x);

            const Tangent Ay = A * y;
            const Tangent A2y = A * Ay;
            const Tangent A3y = A * A2y;
            const Tangent A4y = A * A3y;
            const Tangent A5y = A * A4y;
            const Tangent A6y = A * A5y;

            Tangent out = y;
            out.noalias() += Scalar(-0.5) * Ay;
            out.noalias() += Scalar(1.0 / 12.0) * A2y;
            out.noalias() += Scalar(-1.0 / 720.0) * A4y;
            out.noalias() += Scalar(1.0 / 30240.0) * A6y;
            return out;
        }
    };

} // namespace Aetherion::ODE::RKMK::Lie
