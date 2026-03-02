// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD
//
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>

#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/CrossMotion.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

}

TEST_CASE("CrossMotion matches analytical formula", "[spatial][crossmotion]")
{
    using Scalar = double;

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 1.0, 2.0, 3.0,   // ω
        4.0, 5.0, 6.0;  // linear

    Aetherion::Spatial::Twist<Scalar> u{};
    u.v << -0.5, 0.7, -1.2,
        2.0, -3.0, 1.5;

    auto result = Aetherion::Spatial::CrossMotion(v, u);

    Vec3<Scalar> w1 = v.v.template segment<3>(0);
    Vec3<Scalar> v1 = v.v.template segment<3>(3);

    Vec3<Scalar> w2 = u.v.template segment<3>(0);
    Vec3<Scalar> v2 = u.v.template segment<3>(3);

    Vec3<Scalar> expected_w =
        w1.cross(w2);

    Vec3<Scalar> expected_v =
        w1.cross(v2) + v1.cross(w2);

    CHECK(result.v.template segment<3>(0).isApprox(expected_w));
    CHECK(result.v.template segment<3>(3).isApprox(expected_v));
}

TEST_CASE("CrossMotion matrix form matches operator form", "[spatial][crossmotion][matrix]")
{
    using Scalar = double;

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 0.2, -0.3, 0.4,
        1.0, 2.0, -1.0;

    Aetherion::Spatial::Twist<Scalar> u{};
    u.v << -1.0, 0.5, 0.7,
        0.3, -0.8, 2.0;

    auto matrix_form =
        Aetherion::Spatial::CrossMotionMatrix(v) * u.v;

    auto operator_form =
        Aetherion::Spatial::CrossMotion(v, u).v;

    CHECK(matrix_form.isApprox(operator_form));
}

TEST_CASE("CrossMotion is anti-symmetric", "[spatial][crossmotion][antisymmetry]")
{
    using Scalar = double;

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 0.6, -0.4, 0.1,
        1.2, 0.3, -0.9;

    Aetherion::Spatial::Twist<Scalar> u{};
    u.v << -0.2, 0.7, -0.5,
        0.8, -1.1, 0.4;

    auto v_cross_u =
        Aetherion::Spatial::CrossMotion(v, u).v;

    auto u_cross_v =
        Aetherion::Spatial::CrossMotion(u, v).v;

    CHECK(v_cross_u.isApprox(-u_cross_v));
}