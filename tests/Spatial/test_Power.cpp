// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD
//
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_Power.cpp
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cmath>

#include "Aetherion/Spatial/Power.h"
#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/Spatial/Transform.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

    template<typename S>
    using Mat3 = Eigen::Matrix<S, 3, 3>;

    template<typename S>
    inline Aetherion::Spatial::Twist<S> MakeTwist(const Vec3<S>& w, const Vec3<S>& v)
    {
        Aetherion::Spatial::Twist<S> out{};
        out.v.template segment<3>(0) = w;
        out.v.template segment<3>(3) = v;
        return out;
    }

    template<typename S>
    inline Aetherion::Spatial::Wrench<S> MakeWrench(const Vec3<S>& M, const Vec3<S>& F)
    {
        Aetherion::Spatial::Wrench<S> out{};
        out.f.template segment<3>(0) = M;
        out.f.template segment<3>(3) = F;
        return out;
    }

} // namespace

TEST_CASE("Power: equals f^T v (dot product of spatial vectors)", "[spatial][power]")
{
    using Scalar = double;

    const Vec3<Scalar> M(2.0, -1.0, 0.5);
    const Vec3<Scalar> F(4.0, 1.0, -2.0);

    const Vec3<Scalar> w(0.5, -0.3, 0.2);
    const Vec3<Scalar> v(1.0, 2.0, 3.0);

    const auto f = MakeWrench(M, F);
    const auto t = MakeTwist(w, v);

    const Scalar p = Aetherion::Spatial::Power(f, t);

    const Scalar expected =
        M.dot(w) + F.dot(v);

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(p, WithinAbs(expected, 1e-12));
}

TEST_CASE("Power: is bilinear", "[spatial][power][bilinear]")
{
    using Scalar = double;

    const auto f1 = MakeWrench(Vec3<Scalar>(1.0, 2.0, 3.0), Vec3<Scalar>(4.0, 5.0, 6.0));
    const auto f2 = MakeWrench(Vec3<Scalar>(-2.0, 1.0, 0.5), Vec3<Scalar>(0.0, -1.0, 2.0));

    const auto v1 = MakeTwist(Vec3<Scalar>(0.2, -0.1, 0.3), Vec3<Scalar>(1.0, 0.5, -0.2));
    const auto v2 = MakeTwist(Vec3<Scalar>(-0.4, 0.7, 0.1), Vec3<Scalar>(0.2, -1.0, 2.0));

    const Scalar a = 1.7;
    const Scalar b = -0.6;

    Aetherion::Spatial::Wrench<Scalar> f_lin{};
    f_lin.f = a * f1.f + b * f2.f;

    Aetherion::Spatial::Twist<Scalar> v_lin{};
    v_lin.v = a * v1.v + b * v2.v;

    const Scalar p_flin_v1 = Aetherion::Spatial::Power(f_lin, v1);
    const Scalar p_expected_1 = a * Aetherion::Spatial::Power(f1, v1) + b * Aetherion::Spatial::Power(f2, v1);

    const Scalar p_f1_vlin = Aetherion::Spatial::Power(f1, v_lin);
    const Scalar p_expected_2 = a * Aetherion::Spatial::Power(f1, v1) + b * Aetherion::Spatial::Power(f1, v2);

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(p_flin_v1, WithinAbs(p_expected_1, 1e-12));
    CHECK_THAT(p_f1_vlin, WithinAbs(p_expected_2, 1e-12));
}

TEST_CASE("Power: invariant under rigid transform (X*, X)", "[spatial][power][transform]")
{
    using Scalar = double;

    // Rotation about Z
    const Scalar theta = 0.3;
    Mat3<Scalar> R;
    R << std::cos(theta), -std::sin(theta), 0.0,
        std::sin(theta), std::cos(theta), 0.0,
        0.0, 0.0, 1.0;

    Vec3<Scalar> r(0.2, -0.4, 0.7);

    const auto vA = MakeTwist(Vec3<Scalar>(0.5, -0.3, 0.2), Vec3<Scalar>(1.0, 2.0, 3.0));
    const auto fA = MakeWrench(Vec3<Scalar>(2.0, -1.0, 0.5), Vec3<Scalar>(4.0, 1.0, -2.0));

    const Scalar pA = Aetherion::Spatial::Power(fA, vA);

    const auto vB = Aetherion::Spatial::TransformMotion(R, r, vA);
    const auto fB = Aetherion::Spatial::TransformForce(R, r, fA);

    const Scalar pB = Aetherion::Spatial::Power(fB, vB);

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(pA, WithinAbs(pB, 1e-12));
}