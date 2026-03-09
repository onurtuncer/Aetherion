// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
#pragma once

namespace Aetherion::ODE::RKMK::Lie {

    template<class Scalar>
    SE3<Scalar>::SE3(const Mat3& R_in, const Vec3& p_in)
        : q(Eigen::Quaternion<Scalar>(R_in).normalized()), R(R_in), p(p_in)
    {
        R = q.toRotationMatrix(); // re-derive R from normalised q for consistency
    }

    template<class Scalar>
    SE3<Scalar>::SE3(const Eigen::Quaternion<Scalar>& q_in, const Vec3& p_in)
        : q(q_in.normalized()), R(Mat3::Identity()), p(p_in)
    {
        R = q.toRotationMatrix();
    }

    template<class Scalar>
    SE3<Scalar> SE3<Scalar>::inverse() const {
        SE3 out;
        out.R = R.transpose();
        out.q = Eigen::Quaternion<Scalar>(out.R);
        out.q.normalize();
        out.p = -(out.R * p);
        return out;
    }

    template<class Scalar>
    typename SE3<Scalar>::Mat6 SE3<Scalar>::ad_matrix(const Tangent& xi) {
        const Vec3 w = xi.template head<3>();
        const Vec3 v = xi.template tail<3>();
        Mat6 A = Mat6::Zero();
        const Mat3 W = SO3::Skew(w);
        const Mat3 V = SO3::Skew(v);
        A.template block<3, 3>(0, 0) = W;
        A.template block<3, 3>(3, 0) = V;
        A.template block<3, 3>(3, 3) = W;
        return A;
    }

    template<class Scalar>
    SE3<Scalar> SE3<Scalar>::Exp(const Tangent& xi) {
        const Vec3 w = xi.template head<3>();
        const Vec3 v = xi.template tail<3>();
        const Mat3 Rexp = SO3::Exp_R(w);
        const Mat3 J = SO3::LeftJacobian(w);
        const Vec3 pexp = J * v;
        return SE3(Rexp, pexp);
    }

    template<class Scalar>
    typename SE3<Scalar>::Mat6 SE3<Scalar>::dexp_inv(const Tangent& x) {
        const Mat6 adx = ad_matrix(x);
        const Mat6 adx2 = adx * adx;
        const Mat6 adx4 = adx2 * adx2;
        const Mat6 I = Mat6::Identity();
        return I
            - Scalar(0.5) * adx
            + (Scalar(1) / Scalar(12)) * adx2
            - (Scalar(1) / Scalar(720)) * adx4;
    }

} // namespace Aetherion::ODE::RKMK::Lie

