// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------


// Catch2 tests for: Aetherion/ODE/RKMK/Core/Scalar.h
//
// We test so3_ABC() directly, and also test it in the exact way SE(3) uses it:
//   - SO(3) exp via Rodrigues: R = I + A [w] + B [w]^2
//   - SO(3) left Jacobian:     J = I + B [w] + C [w]^2
//   - SE(3) translation map:   p = J(w) v
//   - J(w) = ∫_0^1 exp(s[w]_x) ds (checked by quadrature)
//
// This file includes ONLY Scalar.h (no SE3 header).
//
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <array>
#include <vector>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Scalar.h>

namespace {
    using Catch::Approx;

    // Correct namespace alias (note the double ::)
    namespace Core = Aetherion::ODE::RKMK::Core;

    template<class S>
    S series_A(S t2) {
        const S t4 = t2 * t2;
        const S t6 = t4 * t2;
        const S t8 = t4 * t4;
        return S(1) - t2 / S(6) + t4 / S(120) - t6 / S(5040) + t8 / S(362880);
    }
    template<class S>
    S series_B(S t2) {
        const S t4 = t2 * t2;
        const S t6 = t4 * t2;
        const S t8 = t4 * t4;
        return S(1) / S(2) - t2 / S(24) + t4 / S(720) - t6 / S(40320) + t8 / S(3628800);
    }
    template<class S>
    S series_C(S t2) {
        const S t4 = t2 * t2;
        const S t6 = t4 * t2;
        const S t8 = t4 * t4;
        return S(1) / S(6) - t2 / S(120) + t4 / S(5040) - t6 / S(362880) + t8 / S(39916800);
    }

    inline Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
        Eigen::Matrix3d W;
        W << 0.0, -w.z(), w.y(),
            w.z(), 0.0, -w.x(),
            -w.y(), w.x(), 0.0;
        return W;
    }

    inline Eigen::Matrix3d so3_exp_from_ABC(const Eigen::Vector3d& w) {
        const double theta2 = w.squaredNorm();
        double A{}, B{}, C{};
        Core::so3_ABC(theta2, A, B, C);
        (void)C;

        const Eigen::Matrix3d W = skew(w);
        const Eigen::Matrix3d W2 = W * W;
        return Eigen::Matrix3d::Identity() + A * W + B * W2;
    }

    inline Eigen::Matrix3d so3_left_jacobian_from_ABC(const Eigen::Vector3d& w) {
        const double theta2 = w.squaredNorm();
        double A{}, B{}, C{};
        Core::so3_ABC(theta2, A, B, C);
        (void)A;

        const Eigen::Matrix3d W = skew(w);
        const Eigen::Matrix3d W2 = W * W;
        return Eigen::Matrix3d::Identity() + B * W + C * W2;
    }

} // namespace

TEST_CASE("Scalar.so3_ABC: near-zero uses the series expansion", "[scalar][so3_ABC]") {
    const double theta2 = 1e-16; // well below eps^2=1e-12 => series branch

    double A{}, B{}, C{};
    Core::so3_ABC(theta2, A, B, C);

    REQUIRE(A == Approx(series_A(theta2)).margin(1e-15));
    REQUIRE(B == Approx(series_B(theta2)).margin(1e-15));
    REQUIRE(C == Approx(series_C(theta2)).margin(1e-15));
}

TEST_CASE("Scalar.so3_ABC: away from zero matches trig definitions", "[scalar][so3_ABC]") {
    const double theta2 = 1e-2; // theta=0.1
    const double theta = std::sqrt(theta2);

    const double A_true = std::sin(theta) / theta;
    const double B_true = (1.0 - std::cos(theta)) / theta2;
    const double C_true = (theta - std::sin(theta)) / (theta2 * theta);

    double A{}, B{}, C{};
    Core::so3_ABC(theta2, A, B, C);

    REQUIRE(A == Approx(A_true).margin(1e-15));
    REQUIRE(B == Approx(B_true).margin(1e-15));
    REQUIRE(C == Approx(C_true).margin(1e-15));
}

TEST_CASE("Scalar.so3_ABC: continuity around eps^2 threshold", "[scalar][so3_ABC]") {
    const double eps2 = 1e-8;
    const double theta2_lo = eps2 * 0.999;
    const double theta2_hi = eps2 * 1.001;

    double A_lo{}, B_lo{}, C_lo{};
    double A_hi{}, B_hi{}, C_hi{};

    Core::so3_ABC(theta2_lo, A_lo, B_lo, C_lo);
    Core::so3_ABC(theta2_hi, A_hi, B_hi, C_hi);

    REQUIRE(A_lo == Approx(A_hi).margin(1e-12));
    REQUIRE(B_lo == Approx(B_hi).margin(1e-12));
    REQUIRE(C_lo == Approx(C_hi).margin(1e-12));
}

TEST_CASE("Scalar.so3_ABC used in SO(3) exp: R is orthonormal and det(R)=1", "[scalar][so3][se3]") {
    const Eigen::Vector3d w(0.3, -0.2, 0.15);
    const Eigen::Matrix3d R = so3_exp_from_ABC(w);

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d RtR = R.transpose() * R;

    REQUIRE((RtR - I).norm() == Approx(0.0).margin(1e-12));
    REQUIRE(R.determinant() == Approx(1.0).margin(1e-12));
}

TEST_CASE("Scalar.so3_ABC used in SO(3) left Jacobian: J(0)=I and small-angle series", "[scalar][so3][se3]") {
    const Eigen::Vector3d w_small(1e-6, -2e-6, 3e-6);
    const Eigen::Matrix3d W = skew(w_small);
    const Eigen::Matrix3d W2 = W * W;

    const Eigen::Matrix3d J = so3_left_jacobian_from_ABC(w_small);

   // REQUIRE((J - Eigen::Matrix3d::Identity()).norm() == Approx(0.0).margin(1e-12));

    const Eigen::Matrix3d J_ser =
        Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W2;

    REQUIRE((J - J_ser).norm() == Approx(0.0).margin(1e-15));
}

TEST_CASE("Scalar.so3_ABC used in SO(3) left Jacobian: J(0)=I", "[scalar][so3][se3]") {
    const Eigen::Matrix3d J0 = so3_left_jacobian_from_ABC(Eigen::Vector3d::Zero());
    REQUIRE((J0 - Eigen::Matrix3d::Identity()).norm() == Approx(0.0).margin(1e-15));
}

TEST_CASE("Scalar.so3_ABC used in SO(3) left Jacobian: matches small-angle series", "[scalar][so3][se3]") {
    const Eigen::Vector3d w_small(1e-6, -2e-6, 3e-6);
    const Eigen::Matrix3d W = skew(w_small);
    const Eigen::Matrix3d W2 = W * W;

    const Eigen::Matrix3d J = so3_left_jacobian_from_ABC(w_small);

    // Series: J ≈ I + 1/2 W + 1/6 W^2
    const Eigen::Matrix3d J_ser =
        Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W2;

    REQUIRE((J - J_ser).norm() == Approx(0.0).margin(1e-15));
}


TEST_CASE("Scalar.so3_ABC used in SE(3): p = J(w)v matches small-angle series", "[scalar][se3]") {
    const Eigen::Vector3d w_small(2e-6, 1e-6, -3e-6);
    const Eigen::Vector3d v(1.2, -0.7, 0.4);

    const Eigen::Matrix3d W = skew(w_small);
    const Eigen::Matrix3d W2 = W * W;

    const Eigen::Matrix3d J = so3_left_jacobian_from_ABC(w_small);
    const Eigen::Vector3d p = J * v;

    const Eigen::Vector3d p_ser =
        (Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 6.0) * W2) * v;

    REQUIRE((p - p_ser).norm() == Approx(0.0).margin(1e-18));
}

TEST_CASE("Scalar.so3_ABC: AD smoke test (CppAD::AD<double>)", "[scalar][so3_ABC][cppad]") {
    using AD = CppAD::AD<double>;

    CppAD::vector<AD> X(1);
    X[0] = AD(1e-4); // theta=1e-2 -> trig branch
    CppAD::Independent(X);

    AD A{}, B{}, C{};
    Core::so3_ABC(X[0], A, B, C);

    CppAD::vector<AD> Y(1);
    Y[0] = A + AD(2.0) * B + AD(3.0) * C;

    CppAD::ADFun<double> fun;
    fun.Dependent(X, Y);

    CppAD::vector<double> x0(1);
    x0[0] = 1e-4;

    CppAD::vector<double> y0 = fun.Forward(0, x0);

    double A_d{}, B_d{}, C_d{};
    Core::so3_ABC(x0[0], A_d, B_d, C_d);
    const double y_true = A_d + 2.0 * B_d + 3.0 * C_d;

    REQUIRE(y0.size() == 1);
    REQUIRE(y0[0] == Approx(y_true).margin(1e-14));

    CppAD::vector<double> J = fun.Jacobian(x0);
    REQUIRE(J.size() == 1);
    REQUIRE(std::isfinite(J[0]));
}

TEST_CASE("Scalar.elementary wrappers: sin_s/cos_s/sqrt_s behave like std for double", "[scalar][wrappers]") {
    const double x = 0.123;
    REQUIRE(Core::sin_s(x) == Approx(std::sin(x)).margin(0.0));
    REQUIRE(Core::cos_s(x) == Approx(std::cos(x)).margin(0.0));
    REQUIRE(Core::sqrt_s(x) == Approx(std::sqrt(x)).margin(0.0));
}

TEST_CASE("Scalar.condexp_lt: behaves like a normal branch for double", "[scalar][condexp]") {
    REQUIRE(Core::condexp_lt(1.0, 2.0, 10.0, 20.0) == Approx(10.0));
    REQUIRE(Core::condexp_lt(2.0, 1.0, 10.0, 20.0) == Approx(20.0));
}

TEST_CASE("Scalar.so3_ABC used in SO(3) exp: matches Eigen::AngleAxis", "[scalar][so3][se3][eigen]") {
    auto check = [&](const Eigen::Vector3d& w, double tol) {
        const Eigen::Matrix3d R = so3_exp_from_ABC(w);

        const double theta = w.norm();
        Eigen::Matrix3d R_ref = Eigen::Matrix3d::Identity();
        if (theta > 0.0) {
            R_ref = Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
        }

        REQUIRE((R - R_ref).norm() == Approx(0.0).margin(tol));
        REQUIRE((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() == Approx(0.0).margin(1e-12));
        REQUIRE(R.determinant() == Approx(1.0).margin(1e-12));
        };

    check(Eigen::Vector3d::Zero(), 0.0);
    check(Eigen::Vector3d(0.3, -0.2, 0.15), 1e-12);
    check(Eigen::Vector3d(-0.8, 0.1, 0.25), 1e-12);
    check(Eigen::Vector3d(0.01, 0.02, -0.03), 1e-14);
    check(Eigen::Vector3d(1.2, -0.7, 0.4), 1e-12);
}

TEST_CASE("Scalar.so3_ABC used in SE(3): J(w) matches integral ∫0^1 exp(s[w]_x) ds (Gauss-Legendre)",
    "[scalar][so3][se3][jacobian][quadrature]") {
    static constexpr std::array<double, 8> xi = {
        -0.9602898564975363,
        -0.7966664774136267,
        -0.5255324099163290,
        -0.1834346424956498,
         0.1834346424956498,
         0.5255324099163290,
         0.7966664774136267,
         0.9602898564975363
    };
    static constexpr std::array<double, 8> wi = {
        0.1012285362903763,
        0.2223810344533745,
        0.3137066458778873,
        0.3626837833783620,
        0.3626837833783620,
        0.3137066458778873,
        0.2223810344533745,
        0.1012285362903763
    };

    auto J_numeric = [&](const Eigen::Vector3d& omega) {
        const double theta = omega.norm();
        const Eigen::Vector3d axis =
            (theta > 0.0) ? (omega / theta) : Eigen::Vector3d(1.0, 0.0, 0.0);

        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();

        // Map [-1,1] -> [0,1]: s = 0.5*(xi + 1), ds = 0.5*dξ
        for (std::size_t i = 0; i < xi.size(); ++i) {
            const double s = 0.5 * (xi[i] + 1.0);
            const double wgt = 0.5 * wi[i];

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            if (theta > 0.0) {
                R = Eigen::AngleAxisd(theta * s, axis).toRotationMatrix();
            }
            J += wgt * R;
        }
        return J;
        };

    auto check = [&](const Eigen::Vector3d& omega, double tol) {
        const Eigen::Matrix3d J_closed = so3_left_jacobian_from_ABC(omega);
        const Eigen::Matrix3d J_num = J_numeric(omega);
        REQUIRE((J_closed - J_num).norm() == Approx(0.0).margin(tol));
        };

    check(Eigen::Vector3d::Zero(), 1e-15);
    check(Eigen::Vector3d(1e-6, -2e-6, 3e-6), 1e-14);
    check(Eigen::Vector3d(0.3, -0.2, 0.15), 1e-12);
    check(Eigen::Vector3d(-0.8, 0.1, 0.25), 1e-12);
    check(Eigen::Vector3d(1.2, -0.7, 0.4), 3e-12);
}
