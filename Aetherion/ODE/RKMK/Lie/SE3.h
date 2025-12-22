// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// Project: Aetherion
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

        // Keep these PUBLIC to satisfy your existing tests: g.R(i,j), g.p(k)
        Mat3 R = Mat3::Identity();
        Vec3 p = Vec3::Zero();

        SE3() = default;

        SE3(const Mat3& R_in, const Vec3& p_in)
            : R(R_in), p(p_in) {
        }

        // Construct from quaternion + translation
        SE3(const Eigen::Quaternion<Scalar>& q_in, const Vec3& p_in)
            : R(q_in.normalized().toRotationMatrix()), p(p_in) {
        }

        [[nodiscard]] static SE3 Identity() { return SE3{}; }

        // Optional convenience expected by some helper code
        [[nodiscard]] Eigen::Quaternion<Scalar> q() const {
            // Eigen can build quaternion from rotation matrix for Scalar types
            Eigen::Quaternion<Scalar> qq(R);
            qq.normalize();
            return qq;
        }

        [[nodiscard]] Eigen::Quaternion<Scalar> rotation() const { return q(); }

        [[nodiscard]] Vec3 translation() const { return p; }

        // Group composition: (R1,p1)*(R2,p2) = (R1R2, p1 + R1 p2)
        friend SE3 operator*(const SE3& a, const SE3& b) {
            return SE3(a.R * b.R, a.p + a.R * b.p);
        }

        [[nodiscard]] SE3 inverse() const {
            const Mat3 Rt = R.transpose();
            return SE3(Rt, -(Rt * p));
        }

        [[nodiscard]] static SE3 compose(const SE3& a, const SE3& b) { return a * b; }

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


/*
#pragma once

#include <type_traits>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cppad/cppad.hpp>

// Make Eigen happy with CppAD::AD<double> scalars (only define once)
#ifndef AETHERION_EIGEN_NUMTRAITS_CPPAD_AD_DOUBLE_DEFINED
#define AETHERION_EIGEN_NUMTRAITS_CPPAD_AD_DOUBLE_DEFINED
namespace Eigen {
    template <>
    struct NumTraits<CppAD::AD<double>> : NumTraits<double> {
        using Real = CppAD::AD<double>;
        using NonInteger = CppAD::AD<double>;
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
    };
} // namespace Eigen
#endif

namespace Aetherion::ODE::RKMK::Lie {

    namespace detail {

        template <class S>
        inline S sin_s(const S& x) {
            using std::sin;
            using CppAD::sin;
            return sin(x);
        }

        template <class S>
        inline S cos_s(const S& x) {
            using std::cos;
            using CppAD::cos;
            return cos(x);
        }

        template <class S>
        inline S sqrt_s(const S& x) {
            using std::sqrt;
            using CppAD::sqrt;
            return sqrt(x);
        }

        template <class S>
        inline S abs_s(const S& x) {
            using std::abs;
            return abs(x);
        }

        // so(3) hat operator
        template <class S>
        inline Eigen::Matrix<S, 3, 3> hat3(const Eigen::Matrix<S, 3, 1>& w) {
            Eigen::Matrix<S, 3, 3> W;
            W << S(0), -w.z(), w.y(),
                w.z(), S(0), -w.x(),
                -w.y(), w.x(), S(0);
            return W;
        }

        // Coefficients for Rodrigues / left Jacobian
        // A = sin(theta)/theta
        // B = (1-cos(theta))/theta^2
        // C = (theta - sin(theta))/theta^3
        template <class S>
        inline void so3_ABC(const S& theta2, S& A, S& B, S& C) {
            // Small-angle series threshold
            const double eps = 1e-12;

            if (abs_s(theta2) < S(eps)) {
                // series in theta^2
                // A = 1 - t/6 + t^2/120
                // B = 1/2 - t/24 + t^2/720
                // C = 1/6 - t/120 + t^2/5040
                const S t = theta2;
                const S t2 = t * t;

                A = S(1) - t / S(6) + t2 / S(120);
                B = S(0.5) - t / S(24) + t2 / S(720);
                C = S(1) / S(6) - t / S(120) + t2 / S(5040);
                return;
            }

            const S theta = sqrt_s(theta2);
            const S s = sin_s(theta);
            const S c = cos_s(theta);

            A = s / theta;
            B = (S(1) - c) / theta2;
            C = (theta - s) / (theta2 * theta);
        }

        template <class S>
        inline Eigen::Matrix<S, 3, 3> so3_exp(const Eigen::Matrix<S, 3, 1>& w) {
            const S theta2 = w.squaredNorm();

            S A, B, C;
            so3_ABC(theta2, A, B, C);

            const Eigen::Matrix<S, 3, 3> W = hat3(w);
            const Eigen::Matrix<S, 3, 3> W2 = W * W;

            return Eigen::Matrix<S, 3, 3>::Identity() + A * W + B * W2;
        }

        template <class S>
        inline Eigen::Matrix<S, 3, 3> so3_left_jacobian(const Eigen::Matrix<S, 3, 1>& w) {
            const S theta2 = w.squaredNorm();

            S A, B, C;
            so3_ABC(theta2, A, B, C);

            (void)A; // not used here

            const Eigen::Matrix<S, 3, 3> W = hat3(w);
            const Eigen::Matrix<S, 3, 3> W2 = W * W;

            // J = I + B*W + C*W^2
            return Eigen::Matrix<S, 3, 3>::Identity() + B * W + C * W2;
        }

        // quaternion (w,x,y,z) -> rotation matrix, normalized
        template <class S>
        inline Eigen::Matrix<S, 3, 3> quat_to_R(const Eigen::Quaternion<S>& q_in) {
            S w = q_in.w();
            S x = q_in.x();
            S y = q_in.y();
            S z = q_in.z();

            const S n2 = w * w + x * x + y * y + z * z;
            const S inv_n = S(1) / sqrt_s(n2);
            w *= inv_n; x *= inv_n; y *= inv_n; z *= inv_n;

            const S two = S(2);

            const S xx = x * x;
            const S yy = y * y;
            const S zz = z * z;
            const S xy = x * y;
            const S xz = x * z;
            const S yz = y * z;
            const S wx = w * x;
            const S wy = w * y;
            const S wz = w * z;

            Eigen::Matrix<S, 3, 3> R;
            R(0, 0) = S(1) - two * (yy + zz);
            R(0, 1) = two * (xy - wz);
            R(0, 2) = two * (xz + wy);

            R(1, 0) = two * (xy + wz);
            R(1, 1) = S(1) - two * (xx + zz);
            R(1, 2) = two * (yz - wx);

            R(2, 0) = two * (xz - wy);
            R(2, 1) = two * (yz + wx);
            R(2, 2) = S(1) - two * (xx + yy);

            return R;
        }

    } // namespace detail

    template <class ScalarT>
    struct SE3 {
        using Scalar = ScalarT;
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using Tangent = Eigen::Matrix<Scalar, 6, 1>;
        using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

        // Tests expect these to be PUBLIC data members.
        Mat3 R = Mat3::Identity();
        Vec3 p = Vec3::Zero();

        SE3() = default;

        SE3(const Mat3& R_in, const Vec3& p_in) : R(R_in), p(p_in) {}

        // Construct from quaternion + position (supports double and AD, and double->AD)
        template <class S2>
            requires (std::is_convertible_v<S2, Scalar>)
        SE3(const Eigen::Quaternion<S2>& q_in, const Eigen::Matrix<S2, 3, 1>& p_in)
            : R(detail::quat_to_R<Scalar>(q_in.template cast<Scalar>()))
            , p(p_in.template cast<Scalar>()) {
        }

        // Cross-scalar copy/convert
        template <class S2>
            requires (std::is_convertible_v<S2, Scalar>)
        explicit SE3(const SE3<S2>& other)
            : R(other.R.template cast<Scalar>())
            , p(other.p.template cast<Scalar>()) {
        }

        [[nodiscard]] static SE3 Identity() { return SE3{}; }

        [[nodiscard]] SE3 inverse() const {
            SE3 out;
            out.R = R.transpose();
            out.p = -(out.R * p);
            return out;
        }

        friend SE3 operator*(const SE3& a, const SE3& b) {
            SE3 out;
            out.R = a.R * b.R;
            out.p = a.p + a.R * b.p;
            return out;
        }

        SE3& operator*=(const SE3& rhs) {
            p = p + R * rhs.p;
            R = R * rhs.R;
            return *this;
        }

        // Exp: xi = [w; v]
        [[nodiscard]] static SE3 Exp(const Tangent& xi) {
            const Vec3 w = xi.template head<3>();
            const Vec3 v = xi.template tail<3>();

            SE3 g;
            g.R = detail::so3_exp<Scalar>(w);

            const Mat3 J = detail::so3_left_jacobian<Scalar>(w);
            g.p = J * v;
            return g;
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

        // se(3) adjoint matrix (ad_xi) for xi=[w; v]
        [[nodiscard]] static Mat6 ad_matrix(const Tangent& xi) {
            const Vec3 w = xi.template head<3>();
            const Vec3 v = xi.template tail<3>();

            const Mat3 W = detail::hat3<Scalar>(w);
            const Mat3 V = detail::hat3<Scalar>(v);

            Mat6 A = Mat6::Zero();
            A.template block<3, 3>(0, 0) = W;
            A.template block<3, 3>(3, 0) = V;
            A.template block<3, 3>(3, 3) = W;
            return A;
        }

        // Approx dexp^{-1}(xi) * y using Bernoulli-series up to A^4
        [[nodiscard]] static Tangent dexp_inv(const Tangent& xi, const Tangent& y) {
            const Mat6 A = ad_matrix(xi);
            const Mat6 A2 = A * A;

            const Tangent Ay = A * y;
            const Tangent A2y = A * Ay;       // A^2 y
            const Tangent A4y = A2 * A2y;      // A^4 y

            return y - Scalar(0.5) * Ay + (Scalar(1) / Scalar(12)) * A2y - (Scalar(1) / Scalar(720)) * A4y;
        }

        template <class U = Scalar>
            requires (!std::is_same_v<U, double>)
        [[nodiscard]] static Eigen::Matrix<Scalar, 6, 1> dexp_inv(const Eigen::Matrix<double, 6, 1>& xi_d,
            const Eigen::Matrix<double, 6, 1>& y_d) {
            return dexp_inv(xi_d.template cast<Scalar>(), y_d.template cast<Scalar>());
        }
    };

} // namespace Aetherion::ODE::RKMK::Lie */









