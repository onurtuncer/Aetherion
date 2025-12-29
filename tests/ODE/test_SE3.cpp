// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Catch2 tests for: Aetherion/ODE/RKMK/Lie/SE3.h
//
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <array>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Lie/SE3.h>

//*****
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace {
    using Catch::Approx;

    namespace Lie = Aetherion::ODE::RKMK::Lie;
    using SE3d = Lie::SE3<double>;

    // 8-point Gauss–Legendre nodes/weights on [-1,1]
    static constexpr std::array<double, 8> gl_x = {
        -0.9602898564975363,
        -0.7966664774136267,
        -0.5255324099163290,
        -0.1834346424956498,
         0.1834346424956498,
         0.5255324099163290,
         0.7966664774136267,
         0.9602898564975363
    };

    static constexpr std::array<double, 8> gl_w = {
        0.1012285362903763,
        0.2223810344533745,
        0.3137066458778873,
        0.3626837833783620,
        0.3626837833783620,
        0.3137066458778873,
        0.2223810344533745,
        0.1012285362903763
    };

    inline Eigen::Matrix3d so3_exp_eigen(const Eigen::Vector3d& w) {
        const double theta = w.norm();
        if (theta == 0.0) return Eigen::Matrix3d::Identity();
        return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
    }

    // J(w) = ∫_0^1 exp(s[w]x) ds  via 8-pt Gauss–Legendre on [0,1]
    inline Eigen::Matrix3d so3_left_jacobian_integral(const Eigen::Vector3d& w) {
        const double theta = w.norm();
        const Eigen::Vector3d axis = (theta > 0.0) ? (w / theta) : Eigen::Vector3d(1.0, 0.0, 0.0);

        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();

        for (std::size_t i = 0; i < gl_x.size(); ++i) {
            const double s = 0.5 * (gl_x[i] + 1.0); // map [-1,1] -> [0,1]
            const double wgt = 0.5 * gl_w[i];

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            if (theta > 0.0) {
                R = Eigen::AngleAxisd(theta * s, axis).toRotationMatrix();
            }
            J += wgt * R;
        }
        return J;
    }

    // Series dexp(x) v = sum_{k>=0} ad_x^k v / (k+1)!
    // Truncate at k=5 (sufficient for small/moderate ||x|| in tests).
    inline SE3d::Tangent dexp_series(const SE3d::Tangent& x, const SE3d::Tangent& v) {
        const Eigen::Matrix<double, 6, 6> A = SE3d::ad_matrix(x);

        SE3d::Tangent Av = A * v;
        SE3d::Tangent A2v = A * Av;
        SE3d::Tangent A3v = A * A2v;
        SE3d::Tangent A4v = A * A3v;
        SE3d::Tangent A5v = A * A4v;

        // v + 1/2 Av + 1/6 A^2 v + 1/24 A^3 v + 1/120 A^4 v + 1/720 A^5 v
        SE3d::Tangent out = v;
        out.noalias() += (1.0 / 2.0) * Av;
        out.noalias() += (1.0 / 6.0) * A2v;
        out.noalias() += (1.0 / 24.0) * A3v;
        out.noalias() += (1.0 / 120.0) * A4v;
        out.noalias() += (1.0 / 720.0) * A5v;
        return out;
    }

    //**********************************************************************

    namespace Core = Aetherion::ODE::RKMK::Core;

    template<class S>
    using Vec6 = Eigen::Matrix<S, 6, 1>;

    Core::NewtonOptions tight_newton() {
        Core::NewtonOptions opt;
        opt.max_iters = 40;
        opt.throw_on_fail = true;
        return opt;
    }

    // Field: xi(t,g) = 0
    struct ZeroField final {
        template<class S>
        Vec6<S> operator()(S /*t*/, const Lie::SE3<S>& /*g*/) const {
            return Vec6<S>::Zero();
        }
    };

    // Field: xi(t,g) = [0; v] (pure translation), constant v
    struct PureTranslationConstant final {
        Eigen::Vector3d v = Eigen::Vector3d::Zero();

        template<class S>
        Vec6<S> operator()(S /*t*/, const Lie::SE3<S>& /*g*/) const {
            Vec6<S> xi = Vec6<S>::Zero();
            xi.template segment<3>(3) = v.template cast<S>();
            return xi;
        }
    };

    // Field: xi(t,g) = [0; v0 + a*t] (pure translation), time varying
    struct PureTranslationAffine final {
        Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a = Eigen::Vector3d::Zero();

        template<class S>
        Vec6<S> operator()(S t, const Lie::SE3<S>& /*g*/) const {
            Vec6<S> xi = Vec6<S>::Zero();
            const Eigen::Matrix<S, 3, 1> v = v0.template cast<S>() + a.template cast<S>() * t;
            xi.template segment<3>(3) = v;
            return xi;
        }
    };
} // namespace



TEST_CASE("SE3.Identity and composition", "[se3]") {
    const auto I = SE3d::Identity();

    SE3d g;
    g.R = Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    g.p = Eigen::Vector3d(1.0, -2.0, 0.5);

    const auto left = I * g;
    const auto right = g * I;

    REQUIRE((left.R - g.R).norm() == Approx(0.0).margin(1e-15));
    REQUIRE((left.p - g.p).norm() == Approx(0.0).margin(1e-15));
    REQUIRE((right.R - g.R).norm() == Approx(0.0).margin(1e-15));
    REQUIRE((right.p - g.p).norm() == Approx(0.0).margin(1e-15));
}

TEST_CASE("SE3.inverse: g * g^{-1} = I", "[se3]") {
    SE3d g;
    g.R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, -0.4, 1.0).normalized()).toRotationMatrix();
    g.p = Eigen::Vector3d(-3.0, 0.2, 7.0);

    const auto I = SE3d::Identity();
    const auto prod = g * g.inverse();

    REQUIRE((prod.R - I.R).norm() == Approx(0.0).margin(1e-12));
    REQUIRE((prod.p - I.p).norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("SE3.Exp: rotation is proper (orthonormal, det=1)", "[se3][exp]") {
    SE3d::Tangent xi;
    xi << 0.3, -0.2, 0.15, 1.0, -2.0, 0.5;

    const auto g = SE3d::Exp(xi);

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    REQUIRE((g.R.transpose() * g.R - I).norm() == Approx(0.0).margin(1e-12));
    REQUIRE(g.R.determinant() == Approx(1.0).margin(1e-12));
}

TEST_CASE("SE3.Exp: matches Eigen AngleAxis for rotation; translation uses integral Jacobian", "[se3][exp]") {
    // xi = [w; v]
    const Eigen::Vector3d w(0.3, -0.2, 0.15);
    const Eigen::Vector3d v(1.2, -0.7, 0.4);

    SE3d::Tangent xi;
    xi << w, v;

    const auto g = SE3d::Exp(xi);

    // Independent reference: R via AngleAxis, J via quadrature integral definition.
    const Eigen::Matrix3d R_ref = so3_exp_eigen(w);
    const Eigen::Matrix3d J_ref = so3_left_jacobian_integral(w);
    const Eigen::Vector3d p_ref = J_ref * v;

    REQUIRE((g.R - R_ref).norm() == Approx(0.0).margin(1e-12));
    REQUIRE((g.p - p_ref).norm() == Approx(0.0).margin(3e-12));
}

TEST_CASE("SE3.dexp_inv: at x=0 returns y exactly", "[se3][dexp_inv]") {
    SE3d::Tangent x = SE3d::Tangent::Zero();
    SE3d::Tangent y;
    y << 0.1, -0.2, 0.3, 1.0, 2.0, -3.0;

    const auto out = SE3d::dexp_inv(x, y);
    REQUIRE((out - y).norm() == Approx(0.0).margin(0.0));
}

TEST_CASE("SE3.dexp_inv: approximately inverts dexp series for small x", "[se3][dexp_inv]") {
    // Choose a "small-ish" x so the truncated Bernoulli/ad series is accurate.
    SE3d::Tangent x;
    x << 0.05, -0.04, 0.02, 0.1, -0.05, 0.03;

    SE3d::Tangent v;
    v << -0.2, 0.1, 0.3, 1.0, -2.0, 0.5;

    const auto y = dexp_series(x, v);
    const auto vinv = SE3d::dexp_inv(x, y);

    // Should recover v with good accuracy (series truncations on both sides).
    REQUIRE((vinv - v).norm() == Approx(0.0).margin(5e-10));
}

TEST_CASE("SE3 with CppAD: Exp is tape-able (smoke test)", "[se3][cppad]") {
    using AD = CppAD::AD<double>;
    using SE3ad = Lie::SE3<AD>;

    // Tape a simple scalar output from Exp(xi)
    CppAD::vector<AD> X(6);
    for (int i = 0; i < 6; ++i) X[i] = AD(0.0);
    CppAD::Independent(X);

    typename SE3ad::Tangent xi;
    for (int i = 0; i < 6; ++i) xi(i) = X[i];

    const auto g = SE3ad::Exp(xi);

    CppAD::vector<AD> Y(1);
    Y[0] = g.R(0, 0) + g.p(0); // arbitrary differentiable scalar
    CppAD::ADFun<double> f;
    f.Dependent(X, Y);

    std::vector<double> x0 = { 0.01, -0.02, 0.03,  1.0, -2.0, 0.5 };
    std::vector<double> y0 = f.Forward(0, x0);
    REQUIRE(y0.size() == 1);
    REQUIRE(std::isfinite(y0[0]));

    auto J = f.Jacobian(x0);
    REQUIRE(J.size() == 6);
    for (double ji : J) REQUIRE(std::isfinite(ji));
}
