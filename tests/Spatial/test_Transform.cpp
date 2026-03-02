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
#include "Aetherion/Spatial/Transform.h"
#include "Aetherion/Spatial/Power.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

    template<typename S>
    using Mat3 = Eigen::Matrix<S, 3, 3>;

}

TEST_CASE("Motion transform identity leaves twist unchanged", "[spatial][transform][motion]")
{
    using Scalar = double;

    Mat3<Scalar> R = Mat3<Scalar>::Identity();
    Vec3<Scalar> r = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 1.0, 2.0, 3.0,
        4.0, 5.0, 6.0;

    auto v_trans =
        Aetherion::Spatial::TransformMotion(R, r, v);

    CHECK(v_trans.v.isApprox(v.v));
}

TEST_CASE("Force transform identity leaves wrench unchanged", "[spatial][transform][force]")
{
    using Scalar = double;

    Mat3<Scalar> R = Mat3<Scalar>::Identity();
    Vec3<Scalar> r = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Wrench<Scalar> f{};
    f.f << 1.0, 2.0, 3.0,
        4.0, 5.0, 6.0;

    auto f_trans =
        Aetherion::Spatial::TransformForce(R, r, f);

    CHECK(f_trans.f.isApprox(f.f));
}

TEST_CASE("Force transform is dual of motion transform", "[spatial][transform][duality]")
{
    using Scalar = double;

    Mat3<Scalar> R = Mat3<Scalar>::Identity();
    Vec3<Scalar> r(0.5, -0.2, 1.0);

    auto X_motion =
        Aetherion::Spatial::MotionTransformMatrix(R, r);

    auto X_force =
        Aetherion::Spatial::ForceTransformMatrix(R, r);

    // Duality condition:
    auto X_inv = X_motion.inverse();
    CHECK(X_force.isApprox(X_inv.transpose()));
}

TEST_CASE("Spatial power invariant under transform", "[spatial][transform][power]")
{
    using Scalar = double;

    // Simple rotation around Z
    Scalar theta = 0.3;

    Mat3<Scalar> R;
    R << std::cos(theta), -std::sin(theta), 0.0,
        std::sin(theta), std::cos(theta), 0.0,
        0.0, 0.0, 1.0;

    Vec3<Scalar> r(0.2, -0.4, 0.7);

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v << 0.5, -0.3, 0.2,
        1.0, 2.0, 3.0;

    Aetherion::Spatial::Wrench<Scalar> f{};
    f.f << 2.0, -1.0, 0.5,
        4.0, 1.0, -2.0;

    const Scalar power_original =
        Aetherion::Spatial::Power(f, v);

    auto v_trans =
        Aetherion::Spatial::TransformMotion(R, r, v);

    auto f_trans =
        Aetherion::Spatial::TransformForce(R, r, f);

    const Scalar power_transformed =
        Aetherion::Spatial::Power(f_trans, v_trans);

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(power_original, WithinAbs(power_transformed, 1e-12));
}