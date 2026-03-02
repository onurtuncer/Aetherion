// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD
//
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_Momentum.cpp
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include "Aetherion/Spatial/Momentum.h"
#include "Aetherion/Spatial/Twist.h"
#include "Aetherion/Spatial/Inertia.h"

namespace {

    template<typename S>
    using Vec3 = Eigen::Matrix<S, 3, 1>;

    template<typename S>
    using Mat3 = Eigen::Matrix<S, 3, 3>;

    template<typename S>
    using Mat6 = Eigen::Matrix<S, 6, 6>;

    template<typename S>
    inline Aetherion::Spatial::Twist<S> MakeTwist(const Vec3<S>& w, const Vec3<S>& v)
    {
        Aetherion::Spatial::Twist<S> out{};
        out.v.template segment<3>(0) = w;
        out.v.template segment<3>(3) = v;
        return out;
    }

} // namespace

TEST_CASE("MomentumFromSpatialInertiaAndTwist: equals I*v", "[spatial][momentum]")
{
    using Scalar = double;

    Mat6<Scalar> I = Mat6<Scalar>::Zero();
    // Something simple but non-trivial
    I.template block<3, 3>(0, 0) = 2.0 * Mat3<Scalar>::Identity();
    I.template block<3, 3>(3, 3) = 5.0 * Mat3<Scalar>::Identity();

    const auto v = MakeTwist(Vec3<Scalar>(1.0, 2.0, 3.0), Vec3<Scalar>(4.0, 5.0, 6.0));

    const auto h = Aetherion::Spatial::MomentumFromSpatialInertiaAndTwist(I, v);

    const Eigen::Matrix<Scalar, 6, 1> expected = I * v.v;

    CHECK(h.h.isApprox(expected));
    CHECK(Aetherion::Spatial::AngularMomentum(h).isApprox(expected.template segment<3>(0)));
    CHECK(Aetherion::Spatial::LinearMomentum(h).isApprox(expected.template segment<3>(3)));
}

TEST_CASE("SpatialInertia operator*: returns Momentum consistent with matrix()", "[spatial][momentum][inertia]")
{
    using Scalar = double;

    const Scalar m = 3.0;

    Mat3<Scalar> Icom = Mat3<Scalar>::Zero();
    Icom(0, 0) = 2.0;
    Icom(1, 1) = 3.0;
    Icom(2, 2) = 4.0;

    Vec3<Scalar> c = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Inertia<Scalar> inertia(m, Icom, c);

    const auto v = MakeTwist(Vec3<Scalar>(0.5, -0.3, 0.2), Vec3<Scalar>(1.0, 2.0, 3.0));

    const auto h = inertia * v;

    const Eigen::Matrix<Scalar, 6, 1> expected = inertia.matrix() * v.v;

    CHECK(h.h.isApprox(expected));
}

TEST_CASE("Momentum linear part equals m*v when c=0 and omega=0", "[spatial][momentum][sanity]")
{
    using Scalar = double;

    const Scalar m = 2.5;

    Mat3<Scalar> Icom = Mat3<Scalar>::Identity(); // irrelevant here
    Vec3<Scalar> c = Vec3<Scalar>::Zero();

    Aetherion::Spatial::Inertia<Scalar> inertia(m, Icom, c);

    // omega = 0
    const Vec3<Scalar> omega = Vec3<Scalar>::Zero();
    const Vec3<Scalar> vlin(1.2, -0.4, 0.7);

    Aetherion::Spatial::Twist<Scalar> v{};
    v.v.template segment<3>(0) = omega;
    v.v.template segment<3>(3) = vlin;

    const auto h = inertia * v;

    const Vec3<Scalar> h_linear = h.h.template segment<3>(3);

    CHECK(h_linear.isApprox(m * vlin));
}