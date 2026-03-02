// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>

#include "Aetherion/Spatial/Inertia.h"
#include "Aetherion/Spatial/Twist.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

    template<typename S>
    using Mat3 = Eigen::Matrix<S, 3, 3>;

    template<typename S>
    using Mat6 = Eigen::Matrix<S, 6, 6>;

}

TEST_CASE("SpatialInertia builds correct 6x6 matrix (parallel axis theorem)", "[spatial][inertia]")
{
    using Scalar = double;

    const Scalar mass = 2.0;

    Mat3<Scalar> I_com = Mat3<Scalar>::Identity();
    Vec3<Scalar> c(1.0, 0.0, 0.0); // offset 1m along x

    Aetherion::Spatial::Inertia<Scalar> I(mass, I_com, c);

    const Mat6<Scalar>& M = I.matrix();

    REQUIRE(M.rows() == 6);
    REQUIRE(M.cols() == 6);

    // Bottom-right block must equal m * I3
    auto BR = M.template block<3, 3>(3, 3);
    CHECK(BR.isApprox(mass * Mat3<Scalar>::Identity()));
}

TEST_CASE("Spatial momentum equals I * twist", "[spatial][momentum]")
{
    using Scalar = double;

    const Scalar mass = 3.0;

    Mat3<Scalar> I_com = Mat3<Scalar>::Identity();
    Vec3<Scalar> c = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Inertia<Scalar> I(mass, I_com, c);

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 1.0, 2.0, 3.0,   // ω
        4.0, 5.0, 6.0;  // linear

    auto h = I * v;

    // Direct comparison with matrix multiplication
    Eigen::Matrix<Scalar, 6, 1> expected = I.matrix() * v.v;

    CHECK(h.h.isApprox(expected));
}

TEST_CASE("Kinetic energy matches 1/2 vᵀ I v", "[spatial][energy]")
{
    using Scalar = double;

    const Scalar mass = 2.0;

    Mat3<Scalar> I_com = Mat3<Scalar>::Identity();
    Vec3<Scalar> c = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Inertia<Scalar> I(mass, I_com, c);

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 0.5, -0.3, 0.2,
        1.0, 2.0, 3.0;

    const Scalar T = I.kineticEnergy(v);

    const Scalar expected =
        0.5 * v.v.dot(I.matrix() * v.v);

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(T, WithinAbs(expected, 1e-12));
}

TEST_CASE("Spatial inertia transforms correctly", "[spatial][transform]")
{
    using Scalar = double;

    const Scalar mass = 1.0;

    Mat3<Scalar> I_com = Mat3<Scalar>::Identity();
    Vec3<Scalar> c = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Inertia<Scalar> I(mass, I_com, c);

    // Identity transform should not change inertia
    Mat3<Scalar> R = Mat3<Scalar>::Identity();
    Vec3<Scalar> r = Vec3<Scalar>::Zero();

    Mat6<Scalar> before = I.matrix();

    I.applyMotionTransform(R, r);

    CHECK(I.matrix().isApprox(before));
}

/*
// Windows headers sometimes poison code with min/max macros.
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

#include <Aetherion/Spatial/Inertia.h>
#include <Aetherion/Spatial/Skew.h>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace {

    // Absolute tolerance based on epsilon, scaled for a few Eigen ops.
    // Conservative default for MSVC + /O2.
    template <typename Scalar>
    constexpr Scalar abs_tol_for_linear_algebra(int scale = 400) {
        static_assert(std::is_floating_point_v<Scalar>,
            "abs_tol_for_linear_algebra expects a floating-point Scalar.");
        const Scalar eps = std::numeric_limits<Scalar>::epsilon();
        const Scalar base = Scalar(scale) * eps;
        const Scalar floor = Scalar(10) * std::numeric_limits<Scalar>::min();
        return (base > floor) ? base : floor;
    }

    // Relative tolerance (dimensionless)
    template <typename Scalar>
    constexpr Scalar rel_tol_for_linear_algebra(int scale = 50) {
        static_assert(std::is_floating_point_v<Scalar>,
            "rel_tol_for_linear_algebra expects a floating-point Scalar.");
        return Scalar(scale) * std::numeric_limits<Scalar>::epsilon();
    }

    // Robust matrix compare: |A-B| <= abs_tol + rel_tol * max(|A|,|B|)
    // NOTE: Avoid std::max to dodge Windows macro issues.
    template <typename DerivedA, typename DerivedB>
    void require_matrix_near_relabs(const Eigen::MatrixBase<DerivedA>& A,
        const Eigen::MatrixBase<DerivedB>& B,
        typename DerivedA::Scalar abs_tol,
        typename DerivedA::Scalar rel_tol) {
        using Scalar = typename DerivedA::Scalar;

        REQUIRE(A.rows() == B.rows());
        REQUIRE(A.cols() == B.cols());

        const Scalar err = (A - B).template lpNorm<Eigen::Infinity>();
        const Scalar aN = A.template lpNorm<Eigen::Infinity>();
        const Scalar bN = B.template lpNorm<Eigen::Infinity>();
        const Scalar ref = (aN > bN) ? aN : bN;

        REQUIRE(err <= abs_tol + rel_tol * ref);
    }

} // namespace

TEMPLATE_TEST_CASE("Spatial::Inertia build6x6 produces correct blocks (parallel-axis theorem)",
    "[Spatial][Inertia][build6x6]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

    const Scalar m = Scalar(12.3);

    Mat3 Icom = Mat3::Zero();
    Icom(0, 0) = Scalar(2.0); Icom(1, 1) = Scalar(3.0); Icom(2, 2) = Scalar(4.0);
    Icom(0, 1) = Icom(1, 0) = Scalar(0.1);
    Icom(0, 2) = Icom(2, 0) = Scalar(-0.2);
    Icom(1, 2) = Icom(2, 1) = Scalar(0.05);

    const Vec3 c(Scalar(0.4), Scalar(-0.2), Scalar(0.1));

    Aetherion::Spatial::Inertia<Scalar> J(m, Icom, c);

    const Mat3 C = Aetherion::Spatial::skew<Scalar>(c);
    const Mat3 I_part_expected = Icom + m * C * C.transpose();
    const Mat3 TR_expected = m * C;
    const Mat3 BL_expected = -m * C;
    const Mat3 BR_expected = m * Mat3::Identity();

    const auto TL = J.M.block(0, 0, 3, 3);
    const auto TR = J.M.block(0, 3, 3, 3);
    const auto BL = J.M.block(3, 0, 3, 3);
    const auto BR = J.M.block(3, 3, 3, 3);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(TL, I_part_expected, abs_tol, rel_tol);
    require_matrix_near_relabs(TR, TR_expected, abs_tol, rel_tol);
    require_matrix_near_relabs(BL, BL_expected, abs_tol, rel_tol);
    require_matrix_near_relabs(BR, BR_expected, abs_tol, rel_tol);

    // Sanity: spatial inertia should be symmetric
    require_matrix_near_relabs(J.M, Mat6(J.M.transpose()), abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::Inertia with zero offset yields simple block structure",
    "[Spatial][Inertia][zero-offset]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    const Scalar m = Scalar(5);

    Mat3 Icom = Mat3::Zero();
    Icom(0, 0) = Scalar(1);
    Icom(1, 1) = Scalar(2);
    Icom(2, 2) = Scalar(3);

    const Vec3 c = Vec3::Zero();
    Aetherion::Spatial::Inertia<Scalar> J(m, Icom, c);

    const auto TL = J.M.block(0, 0, 3, 3);
    const auto TR = J.M.block(0, 3, 3, 3);
    const auto BL = J.M.block(3, 0, 3, 3);
    const auto BR = J.M.block(3, 3, 3, 3);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(TL, Icom, abs_tol, rel_tol);
    require_matrix_near_relabs(TR, Mat3::Zero(), abs_tol, rel_tol);
    require_matrix_near_relabs(BL, Mat3::Zero(), abs_tol, rel_tol);
    require_matrix_near_relabs(BR, m * Mat3::Identity(), abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::Inertia toBlock returns the four 3x3 submatrices in order",
    "[Spatial][Inertia][toBlock]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    const Scalar m = Scalar(7.7);
    Mat3 Icom = Mat3::Identity() * Scalar(0.9);
    Vec3 c(Scalar(0.01), Scalar(0.02), Scalar(0.03));

    Aetherion::Spatial::Inertia<Scalar> J(m, Icom, c);

    const auto blocks = J.toBlock();

    const auto TL = J.M.block(0, 0, 3, 3);
    const auto TR = J.M.block(0, 3, 3, 3);
    const auto BL = J.M.block(3, 0, 3, 3);
    const auto BR = J.M.block(3, 3, 3, 3);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(blocks[0], TL, abs_tol, rel_tol);
    require_matrix_near_relabs(blocks[1], TR, abs_tol, rel_tol);
    require_matrix_near_relabs(blocks[2], BL, abs_tol, rel_tol);
    require_matrix_near_relabs(blocks[3], BR, abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::Inertia applyTransform: identity keeps inertia unchanged",
    "[Spatial][Inertia][applyTransform]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

    const Scalar m = Scalar(3.2);
    Mat3 Icom = Mat3::Identity() * Scalar(1.1);
    Vec3 c(Scalar(0.3), Scalar(0.0), Scalar(-0.2));

    Aetherion::Spatial::Inertia<Scalar> J(m, Icom, c);

    const Mat6 X = Mat6::Identity();
    const Mat6 before = J.M;

    J.applyTransform(X);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(J.M, before, abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::Inertia applyTransform: block-diagonal rotation matches X*M*X^T",
    "[Spatial][Inertia][applyTransform][rotation]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

    const Scalar m = Scalar(9.0);

    Mat3 Icom = Mat3::Zero();
    Icom(0, 0) = Scalar(2.0); Icom(1, 1) = Scalar(1.0); Icom(2, 2) = Scalar(3.0);
    Icom(0, 1) = Icom(1, 0) = Scalar(0.2);

    Vec3 c(Scalar(0.2), Scalar(0.4), Scalar(-0.1));

    Aetherion::Spatial::Inertia<Scalar> J(m, Icom, c);

    // Use std::numbers::pi (no variable-template angle brackets) + cast to Scalar.
    const Scalar ang = static_cast<Scalar>(std::numbers::pi) / Scalar(2);

    Mat3 R;
    R << std::cos(ang), -std::sin(ang), Scalar(0),
        std::sin(ang), std::cos(ang), Scalar(0),
        Scalar(0), Scalar(0), Scalar(1);

    Mat6 X = Mat6::Zero();
    X.block(0, 0, 3, 3) = R;
    X.block(3, 3, 3, 3) = R;

    const Mat6 expected = X * J.M * X.transpose();

    J.applyTransform(X);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(J.M, expected, abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::Inertia operator* returns spatial momentum p = M*v",
    "[Spatial][Inertia][momentum]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vec6 = Eigen::Matrix<Scalar, 6, 1>;

    const Scalar m = Scalar(4.4);
    Mat3 Icom = Mat3::Identity() * Scalar(0.33);
    Vec3 c(Scalar(0.12), Scalar(-0.05), Scalar(0.08));

    Aetherion::Spatial::Inertia<Scalar> J(m, Icom, c);

    Vec6 v;
    v << Scalar(1.0), Scalar(-2.0), Scalar(0.5),
        Scalar(3.0), Scalar(-1.0), Scalar(0.25);

    const Vec6 expected = J.M * v;
    const Vec6 got = J * v;

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(got, expected, abs_tol, rel_tol);
}
*/