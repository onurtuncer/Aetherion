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
#include <cmath>

#include <Aetherion/ODE/RKMK/Lie/SO3.h>

namespace {
    using Vec3d = Eigen::Vector3d;
    using Mat3d = Eigen::Matrix3d;
    namespace SO3 = Aetherion::ODE::RKMK::Lie::SO3;
    using Catch::Matchers::WithinAbs;

    static constexpr double kPi = M_PI;

    void check_rotation_matrix(const Mat3d& R, double tol = 1e-11)
    {
        INFO("R^T R should be identity");
        CHECK((R.transpose() * R).isApprox(Mat3d::Identity(), tol));
        INFO("det(R) should be +1");
        CHECK_THAT(R.determinant(), WithinAbs(1.0, tol));
    }
}

// =============================================================================
// SO3::Exp_R
// =============================================================================

TEST_CASE("SO3::Exp_R: zero vector gives identity", "[SO3][Exp_R]")
{
    Mat3d R = SO3::Exp_R(Vec3d::Zero());
    CHECK(R.isApprox(Mat3d::Identity(), 1e-12));
}

TEST_CASE("SO3::Exp_R: 90-degree rotation around z", "[SO3][Exp_R]")
{
    Mat3d R = SO3::Exp_R(Vec3d(0.0, 0.0, kPi / 2.0));

    Mat3d expected;
    expected <<  0.0, -1.0, 0.0,
                 1.0,  0.0, 0.0,
                 0.0,  0.0, 1.0;

    CHECK(R.isApprox(expected, 1e-12));
}

TEST_CASE("SO3::Exp_R: 180-degree rotation around x", "[SO3][Exp_R]")
{
    Mat3d R = SO3::Exp_R(Vec3d(kPi, 0.0, 0.0));

    Mat3d expected;
    expected << 1.0,  0.0,  0.0,
                0.0, -1.0,  0.0,
                0.0,  0.0, -1.0;

    CHECK(R.isApprox(expected, 1e-12));
}

TEST_CASE("SO3::Exp_R: output is a valid rotation matrix for various angles", "[SO3][Exp_R]")
{
    const std::vector<Vec3d> cases = {
        Vec3d(0.3,  0.0,  0.0),
        Vec3d(0.0,  0.5,  0.0),
        Vec3d(0.0,  0.0,  0.7),
        Vec3d(0.1,  0.2,  0.3),
        Vec3d(-0.5, 0.4, -0.1),
        Vec3d(kPi, 0.0, 0.0),
        Vec3d(0.0, kPi, 0.0),
    };
    for (const auto& w : cases) {
        CAPTURE(w.transpose());
        check_rotation_matrix(SO3::Exp_R(w));
    }
}

TEST_CASE("SO3::Exp_R: small angle is approximately identity", "[SO3][Exp_R]")
{
    // For ||w|| < 1e-6 the result should be within 1e-6 of I
    Mat3d R = SO3::Exp_R(Vec3d(1e-7, 0.0, 0.0));
    CHECK(R.isApprox(Mat3d::Identity(), 1e-6));
}

TEST_CASE("SO3::Exp_R: composition property Exp(w1)*Exp(w2) is SO(3)", "[SO3][Exp_R]")
{
    Mat3d R1 = SO3::Exp_R(Vec3d(0.4, 0.0, 0.0));
    Mat3d R2 = SO3::Exp_R(Vec3d(0.0, 0.3, 0.0));
    check_rotation_matrix(R1 * R2);
}

TEST_CASE("SO3::Exp_R: inverse is transpose (Exp(-w) = Exp(w)^T)", "[SO3][Exp_R]")
{
    Vec3d w(0.2, -0.3, 0.5);
    Mat3d R  = SO3::Exp_R( w);
    Mat3d Ri = SO3::Exp_R(-w);
    CHECK(R.isApprox(Ri.transpose(), 1e-12));
}

// =============================================================================
// SO3::LeftJacobian
// =============================================================================

TEST_CASE("SO3::LeftJacobian: zero vector gives identity", "[SO3][LeftJacobian]")
{
    Mat3d J = SO3::LeftJacobian(Vec3d::Zero());
    CHECK(J.isApprox(Mat3d::Identity(), 1e-12));
}

TEST_CASE("SO3::LeftJacobian: small angle is approximately identity", "[SO3][LeftJacobian]")
{
    Mat3d J = SO3::LeftJacobian(Vec3d(1e-7, 0.0, 0.0));
    CHECK(J.isApprox(Mat3d::Identity(), 1e-5));
}

TEST_CASE("SO3::LeftJacobian: non-singular for general rotation vector", "[SO3][LeftJacobian]")
{
    const std::vector<Vec3d> cases = {
        Vec3d(0.3,  0.0,  0.0),
        Vec3d(0.1,  0.2,  0.3),
        Vec3d(-0.5, 0.4, -0.1),
    };
    for (const auto& w : cases) {
        CAPTURE(w.transpose());
        double det = SO3::LeftJacobian(w).determinant();
        CHECK_THAT(det, !WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("SO3::LeftJacobian: J(-w) = J(w)^T (adjoint symmetry)", "[SO3][LeftJacobian]")
{
    // The left Jacobian satisfies J_l(-w) = J_r(w) = J_l(w)^T
    Vec3d w(0.2, -0.3, 0.5);
    Mat3d J_pos = SO3::LeftJacobian( w);
    Mat3d J_neg = SO3::LeftJacobian(-w);
    CHECK(J_neg.isApprox(J_pos.transpose(), 1e-12));
}
