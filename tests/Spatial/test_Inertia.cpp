// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
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

TEST_CASE("Kinetic energy matches 1/2 v^T I v", "[spatial][energy]")
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
