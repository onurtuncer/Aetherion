// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Aetherion/ODE/RKMK/Lie/SO3.h"

namespace Aetherion::ODE::RKMK::Lie {

    template <class Scalar>
    struct SE3 {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using Tangent = Eigen::Matrix<Scalar, 6, 1>;
        using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

        Eigen::Quaternion<Scalar> q = Eigen::Quaternion<Scalar>::Identity();
        Mat3 R = Mat3::Identity();
        Vec3 p = Vec3::Zero();

        SE3() = default;
        SE3(const Mat3& R_in, const Vec3& p_in);
        SE3(const Eigen::Quaternion<Scalar>& q_in, const Vec3& p_in);

        [[nodiscard]] static SE3 Identity() { return SE3{}; }

        [[nodiscard]] Eigen::Quaternion<Scalar> rotation() const { return q; }
        [[nodiscard]] Vec3 translation() const { return p; }

        friend SE3 operator*(const SE3& a, const SE3& b) { /* keep inline */
            SE3 out;
            out.q = (a.q * b.q);
            out.q.normalize();
            out.R = out.q.toRotationMatrix();
            out.p = a.p + a.R * b.p;
            return out;
        }

        [[nodiscard]] SE3 inverse() const;

        [[nodiscard]] static Mat6 ad_matrix(const Tangent& xi);
        [[nodiscard]] static SE3 Exp(const Tangent& xi);
        [[nodiscard]] static SE3 exp(const Tangent& xi) { return Exp(xi); }

        [[nodiscard]] static Mat6 dexp_inv(const Tangent& x);
        [[nodiscard]] static Tangent dexp_inv(const Tangent& x, const Tangent& y) { return dexp_inv(x) * y; }
    };

} // namespace Aetherion::ODE::RKMK::Lie

#include <Aetherion/ODE/RKMK/Lie/SE3.inl>
