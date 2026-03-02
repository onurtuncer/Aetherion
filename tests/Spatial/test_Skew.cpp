// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <Eigen/Dense>
#include <limits>
#include <type_traits>

#include <Aetherion/Spatial/Skew.h>   

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace {

    template <typename Scalar>
    constexpr Scalar abs_tol_for_linear_algebra(int scale = 200) {
        static_assert(std::is_floating_point_v<Scalar>,
            "abs_tol_for_linear_algebra expects a floating-point Scalar.");
        const Scalar eps = std::numeric_limits<Scalar>::epsilon();
        const Scalar base = Scalar(scale) * eps;
        const Scalar floor = Scalar(10) * std::numeric_limits<Scalar>::min();
        return (base > floor) ? base : floor;
    }

    template <typename Scalar>
    constexpr Scalar rel_tol_for_linear_algebra(int scale = 50) {
        static_assert(std::is_floating_point_v<Scalar>,
            "rel_tol_for_linear_algebra expects a floating-point Scalar.");
        return Scalar(scale) * std::numeric_limits<Scalar>::epsilon();
    }

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

TEMPLATE_TEST_CASE("Spatial::skew returns the correct skew-symmetric matrix",
    "[Spatial][Skew][skew]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    const Vec3 v(Scalar(1.25), Scalar(-2.5), Scalar(0.75));

    const Mat3 S = Aetherion::Spatial::skew<Scalar>(v);

    Mat3 expected;
    expected << Scalar(0), -v(2), v(1),
        v(2), Scalar(0), -v(0),
        -v(1), v(0), Scalar(0);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(S, expected, abs_tol, rel_tol);

    // Should be skew-symmetric: S^T = -S
    require_matrix_near_relabs(Mat3(S.transpose()), Mat3(-S), abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::skew implements the cross product: skew(a) * b == a x b",
    "[Spatial][Skew][cross]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    const Vec3 a(Scalar(0.3), Scalar(-1.2), Scalar(2.0));
    const Vec3 b(Scalar(1.7), Scalar(0.4), Scalar(-0.9));

    const Mat3 A = Aetherion::Spatial::skew<Scalar>(a);

    const Vec3 lhs = A * b;         // skew(a) b
    const Vec3 rhs = a.cross(b);    // a x b

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(lhs, rhs, abs_tol, rel_tol);
}

TEMPLATE_TEST_CASE("Spatial::skew(0) returns zero matrix",
    "[Spatial][Skew][zero]",
    float, double)
{
    using Scalar = TestType;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    const Vec3 z = Vec3::Zero();
    const Mat3 S = Aetherion::Spatial::skew<Scalar>(z);

    const Scalar abs_tol = abs_tol_for_linear_algebra<Scalar>();
    const Scalar rel_tol = rel_tol_for_linear_algebra<Scalar>();

    require_matrix_near_relabs(S, Mat3::Zero(), abs_tol, rel_tol);
}