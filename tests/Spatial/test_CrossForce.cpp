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
#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/Spatial/CrossMotion.h"
#include "Aetherion/Spatial/CrossForce.h"
#include "Aetherion/Spatial/Power.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

}

TEST_CASE("CrossForce matches analytical formula", "[spatial][crossforce]")
{
    using Scalar = double;

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 1.0, 2.0, 3.0,   // ω
        4.0, 5.0, 6.0;  // linear

    Aetherion::Spatial::Wrench<Scalar> f{};
    f.f << 0.5, -1.0, 2.0,  // M
        3.0, 4.0, -2.0; // F

    auto result = Aetherion::Spatial::CrossForce(v, f);

    Vec3<Scalar> w = v.v.template segment<3>(0);
    Vec3<Scalar> lin = v.v.template segment<3>(3);

    Vec3<Scalar> M = f.f.template segment<3>(0);
    Vec3<Scalar> F = f.f.template segment<3>(3);

    Vec3<Scalar> expected_M =
        w.cross(M) + lin.cross(F);

    Vec3<Scalar> expected_F =
        w.cross(F);

    CHECK(result.f.template segment<3>(0).isApprox(expected_M));
    CHECK(result.f.template segment<3>(3).isApprox(expected_F));
}

TEST_CASE("CrossForce duality: crf = -crm^T", "[spatial][crossforce][duality]")
{
    using Scalar = double;

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 0.3, -0.5, 0.7,
        1.0, 2.0, -1.0;

    auto crm = Aetherion::Spatial::CrossMotionMatrix(v);
    auto crf = Aetherion::Spatial::CrossForceMatrix(v);

    CHECK(crf.isApprox(-crm.transpose()));
}

TEST_CASE("Cross operators satisfy power identity", "[spatial][crossforce][power]")
{
    using Scalar = double;

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 0.5, -0.3, 0.2,
        1.0, 2.0, 3.0;

    Aetherion::Spatial::Twist<Scalar> u{};
    u.v << -0.4, 0.8, 0.1,
        0.3, -1.0, 2.0;

    Aetherion::Spatial::Wrench<Scalar> f{};
    f.f << 2.0, -1.0, 0.5,
        4.0, 1.0, -2.0;

    auto v_cross_u =
        Aetherion::Spatial::CrossMotion(v, u);

    auto v_cross_star_f =
        Aetherion::Spatial::CrossForce(v, f);

    const Scalar lhs =
        Aetherion::Spatial::Power(f, v_cross_u);

    const Scalar rhs =
        Aetherion::Spatial::Power(v_cross_star_f, u);

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(lhs, WithinAbs(-rhs, 1e-12));
}