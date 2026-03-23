// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD
//
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_Adjoint.cpp
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>

#include "Aetherion/Spatial/Adjoint.h"
#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Wrench.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

    template<typename S>
    using Mat6 = Eigen::Matrix<S, 6, 6>;

    template<typename S>
    inline Aetherion::Spatial::Twist<S> MakeTwist(
        const Vec3<S>& w,
        const Vec3<S>& v)
    {
        Aetherion::Spatial::Twist<S> out{};
        out.v.template segment<3>(0) = w;
        out.v.template segment<3>(3) = v;
        return out;
    }

    template<typename S>
    inline Aetherion::Spatial::Wrench<S> MakeWrench(
        const Vec3<S>& M,
        const Vec3<S>& F)
    {
        Aetherion::Spatial::Wrench<S> out{};
        out.f.template segment<3>(0) = M;
        out.f.template segment<3>(3) = F;
        return out;
    }

} // namespace

TEST_CASE("Adjoint: ad_star(xi) equals -ad(xi)^T", "[spatial][adjoint][duality]")
{
    using Scalar = double;

    const Vec3<Scalar> w(0.4, -0.2, 0.7);
    const Vec3<Scalar> v(1.0, 2.0, -1.0);

    const auto xi = MakeTwist(w, v);

    const Mat6<Scalar> A = Aetherion::Spatial::ad(xi);
    const Mat6<Scalar> As = Aetherion::Spatial::ad_star(xi);

    CHECK(As.isApprox(-A.transpose()));
}

TEST_CASE("Adjoint: ad(xi) matches analytical motion cross formula", "[spatial][adjoint][motion]")
{
    using Scalar = double;

    const Vec3<Scalar> w1(0.6, -0.4, 0.1);
    const Vec3<Scalar> v1(1.2, 0.3, -0.9);
    const auto xi = MakeTwist(w1, v1);

    const Vec3<Scalar> w2(-0.2, 0.7, -0.5);
    const Vec3<Scalar> v2(0.8, -1.1, 0.4);
    const auto u = MakeTwist(w2, v2);

    const auto res_vec = Aetherion::Spatial::ad(xi) * u.v;

    // Analytical:
    // [ w1x w2
    //   w1x v2 + v1x w2 ]
    const Vec3<Scalar> expected_top = w1.cross(w2);
    const Vec3<Scalar> expected_bot = w1.cross(v2) + v1.cross(w2);

    CHECK(res_vec.template segment<3>(0).isApprox(expected_top));
    CHECK(res_vec.template segment<3>(3).isApprox(expected_bot));
}

TEST_CASE("Adjoint: ad_star_times(xi,y) equals ad_star(xi)*y", "[spatial][adjoint][fast]")
{
    using Scalar = double;

    const Vec3<Scalar> w(0.5, -0.3, 0.2);
    const Vec3<Scalar> v(1.0, 2.0, 3.0);
    const auto xi = MakeTwist(w, v);

    Eigen::Matrix<Scalar, 6, 1> y;
    y << 2.0, -1.0, 0.5,
        4.0, 1.0, -2.0;

    const auto fast = Aetherion::Spatial::ad_star_times(xi, y);
    const auto mat = Aetherion::Spatial::ad_star(xi) * y;

    CHECK(fast.isApprox(mat));
}

TEST_CASE("Adjoint: ad_star(xi) matches analytical force cross formula", "[spatial][adjoint][force]")
{
    using Scalar = double;

    const Vec3<Scalar> w(1.0, 2.0, 3.0);
    const Vec3<Scalar> v(4.0, 5.0, 6.0);
    const auto xi = MakeTwist(w, v);

    const Vec3<Scalar> M(0.5, -1.0, 2.0);
    const Vec3<Scalar> F(3.0, 4.0, -2.0);
    const auto f = MakeWrench(M, F);

    const auto res_vec = Aetherion::Spatial::ad_star(xi) * f.f;

    // Analytical:
    // [ wx M + vx F
    //   wx F ]
    const Vec3<Scalar> expected_top = w.cross(M) + v.cross(F);
    const Vec3<Scalar> expected_bot = w.cross(F);

    CHECK(res_vec.template segment<3>(0).isApprox(expected_top));
    CHECK(res_vec.template segment<3>(3).isApprox(expected_bot));
}