// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <type_traits>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#if __has_include(<cppad/cppad.hpp>)
#include <cppad/cppad.hpp>
#endif

namespace Aetherion::ODE::RKMK::Lie {

    namespace detail {

        // AD-friendly trig wrappers (ADL will find CppAD overloads if Scalar is AD)
        template <class S>
        inline S Sin(const S& x) { using std::sin; return sin(x); }

        template <class S>
        inline S Cos(const S& x) { using std::cos; return cos(x); }

        template <class S>
        inline S Sqrt(const S& x) { using std::sqrt; return sqrt(x); }

        template <class S>
        inline S Abs(const S& x) { using std::abs; return abs(x); }

        // Get a runtime double value for branching (works for double, and for CppAD::AD<double> if available)
        inline double value_as_double(double x) { return x; }

#if __has_include(<cppad/cppad.hpp>)
        template <class T>
        inline auto value_as_double(const T& x) -> decltype(CppAD::Value(x), double{}) { return CppAD::Value(x); }
#endif

        template <class S>
        inline Eigen::Matrix<S, 3, 3> Skew(const Eigen::Matrix<S, 3, 1>& w) {
            Eigen::Matrix<S, 3, 3> W;
            W << S(0), -w(2), w(1),
                w(2), S(0), -w(0),
                -w(1), w(0), S(0);
            return W;
        }

        // Robust coefficients for Rodrigues / Jacobians using a *value-based* small-angle branch (AD-friendly enough)
        template <class S>
        inline void so3_coeffs(const Eigen::Matrix<S, 3, 1>& w, S& A, S& B, S& C) {
            // theta = ||w||
            const S theta2 = w.dot(w);
            const S theta = Sqrt(theta2);

            const double th = value_as_double(theta);
            const double eps = 1e-8;

            if (th < eps) {
                // Series expansions around 0
                // A = sin(theta)/theta
                // B = (1-cos(theta))/theta^2
                // C = (theta - sin(theta))/theta^3
                // Use theta2 to avoid theta in denominator
                const S t2 = theta2;
                const S t4 = t2 * t2;
                const S t6 = t4 * t2;

                A = S(1) - t2 / S(6) + t4 / S(120) - t6 / S(5040);
                B = S(0.5) - t2 / S(24) + t4 / S(720) - t6 / S(40320);
                C = S(1.0 / 6.0) - t2 / S(120) + t4 / S(5040) - t6 / S(362880);
            }
            else {
                A = Sin(theta) / theta;
                B = (S(1) - Cos(theta)) / theta2;
                C = (theta - Sin(theta)) / (theta2 * theta);
            }
        }

        template <class S>
        inline Eigen::Matrix<S, 3, 3> so3_exp_R(const Eigen::Matrix<S, 3, 1>& w) {
            Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
            const auto W = Skew(w);
            const auto W2 = W * W;

            S A, B, C;
            so3_coeffs(w, A, B, C);

            // Rodrigues: R = I + A*W + B*W^2
            return I + A * W + B * W2;
        }

        template <class S>
        inline Eigen::Matrix<S, 3, 3> so3_left_jacobian(const Eigen::Matrix<S, 3, 1>& w) {
            Eigen::Matrix<S, 3, 3> I = Eigen::Matrix<S, 3, 3>::Identity();
            const auto W = Skew(w);
            const auto W2 = W * W;

            S A, B, C;
            so3_coeffs(w, A, B, C);

            // J = I + B*W + C*W^2
            return I + B * W + C * W2;
        }

    } // namespace detail

    template <class Scalar>
    struct SE3 {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using Tangent = Eigen::Matrix<Scalar, 6, 1>;
        using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

        // --- Public storage (keep for tests + integrator extractors) ---
        Eigen::Quaternion<Scalar> q = Eigen::Quaternion<Scalar>::Identity(); // <-- IMPORTANT
        Mat3 R = Mat3::Identity();                                           // <-- keep for tests
        Vec3 p = Vec3::Zero();                                               // <-- keep for tests

        SE3() = default;

        SE3(const Mat3& R_in, const Vec3& p_in)
            : q(Eigen::Quaternion<Scalar>(R_in)),
            R(R_in),
            p(p_in)
        {
            q.normalize();
            R = q.toRotationMatrix();
        }

        SE3(const Eigen::Quaternion<Scalar>& q_in, const Vec3& p_in)
            : q(q_in),
            R(Mat3::Identity()),
            p(p_in)
        {
            q.normalize();
            R = q.toRotationMatrix();
        }

        [[nodiscard]] static SE3 Identity() { return SE3{}; }

        // These now trivially satisfy any "q()/rotation()" requirement.
        [[nodiscard]] Eigen::Quaternion<Scalar> q_() const { return q; } // optional helper if you want
//        [[nodiscard]] Eigen::Quaternion<Scalar> q()  const { return q; } // if you had q() already, keep name consistent with your integrator
        [[nodiscard]] Eigen::Quaternion<Scalar> rotation() const { return q; }

        [[nodiscard]] Vec3 translation() const { return p; }

        friend SE3 operator*(const SE3& a, const SE3& b) {
            SE3 out;
            out.q = (a.q * b.q);
            out.q.normalize();
            out.R = out.q.toRotationMatrix();
            out.p = a.p + a.R * b.p;
            return out;
        }

        [[nodiscard]] static SE3 compose(const SE3& a, const SE3& b) { return a * b; }

        [[nodiscard]] SE3 inverse() const {
            SE3 out;
            out.q = q.conjugate();
            out.q.normalize();
            out.R = out.q.toRotationMatrix();
            out.p = -(out.R * p);
            return out;
        }

            // ad(xi) for se(3), xi=[w; v]
        [[nodiscard]] static Mat6 ad_matrix(const Tangent& xi) {
            const Vec3 w = xi.template head<3>();
            const Vec3 v = xi.template tail<3>();

            Mat6 A = Mat6::Zero();
            const Mat3 W = detail::Skew(w);
            const Mat3 V = detail::Skew(v);

            // [ W  0
            //   V  W ]
            A.template block<3, 3>(0, 0) = W;
            A.template block<3, 3>(3, 0) = V;
            A.template block<3, 3>(3, 3) = W;
            return A;
        }

        // Exp: xi = [w; v]
        [[nodiscard]] static SE3 Exp(const Tangent& xi) {
            const Vec3 w = xi.template head<3>();
            const Vec3 v = xi.template tail<3>();

            const Mat3 Rexp = detail::so3_exp_R(w);
            const Mat3 J = detail::so3_left_jacobian(w);
            const Vec3 pexp = J * v;

            return SE3(Rexp, pexp);
        }

        // Only needed when Scalar is NOT double (e.g., CppAD::AD<double>)
        template <class U = Scalar>
            requires (!std::is_same_v<U, double>)
        [[nodiscard]] static SE3 Exp(const Eigen::Matrix<double, 6, 1>& xi_d) {
            return Exp(xi_d.template cast<Scalar>());
        }

        [[nodiscard]] static SE3 exp(const Tangent& xi) { return Exp(xi); }

        template <class U = Scalar>
            requires (!std::is_same_v<U, double>)
        [[nodiscard]] static SE3 exp(const Eigen::Matrix<double, 6, 1>& xi_d) {
            return Exp(xi_d);
        }

        // dexp^{-1}(x) applied to y, using Bernoulli-series approximation:
        // J^{-1} ≈ I - 1/2 ad + 1/12 ad^2 - 1/720 ad^4
        [[nodiscard]] static Tangent dexp_inv(const Tangent& x, const Tangent& y) {
            const Mat6 adx = ad_matrix(x);
            const Mat6 adx2 = adx * adx;
            const Mat6 adx4 = adx2 * adx2;

            const Mat6 I = Mat6::Identity();
            const Mat6 Jinv = I
                - Scalar(0.5) * adx
                + (Scalar(1) / Scalar(12)) * adx2
                - (Scalar(1) / Scalar(720)) * adx4;

            return Jinv * y;
        }

        // J^{-1}(x) as a matrix (same Bernoulli approximation you already use)
        [[nodiscard]] static Mat6 dexp_inv(const Tangent& x) {
            const Mat6 adx = ad_matrix(x);
            const Mat6 adx2 = adx * adx;
            const Mat6 adx4 = adx2 * adx2;

            const Mat6 I = Mat6::Identity();
            return I
                - Scalar(0.5) * adx
                + (Scalar(1) / Scalar(12)) * adx2
                - (Scalar(1) / Scalar(720)) * adx4;
        }

        
    };
  
} // namespace Aetherion::ODE::RKMK::Lie

// ---- Eigen NumTraits for CppAD::AD<double> (optional but helpful) ----
#if __has_include(<cppad/cppad.hpp>)
namespace Eigen {
    template <>
    struct NumTraits<CppAD::AD<double>> : NumTraits<double> {
        using Real = CppAD::AD<double>;
        using NonInteger = Real;
        using Nested = Real;
        enum {
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3
        };
        static inline Real epsilon() { return NumTraits<double>::epsilon(); }
        static inline Real dummy_precision() { return NumTraits<double>::dummy_precision(); }
        static inline Real highest() { return NumTraits<double>::highest(); }
        static inline Real lowest() { return NumTraits<double>::lowest(); }
    };
} // namespace Eigen
#endif












