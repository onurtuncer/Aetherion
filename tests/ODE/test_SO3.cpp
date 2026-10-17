// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <numbers>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vector>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Lie/SO3.h>

namespace {
    using Vec3d = Eigen::Vector3d;
    using Mat3d = Eigen::Matrix3d;
    namespace SO3 = Aetherion::ODE::RKMK::Lie::SO3;
    using Catch::Matchers::WithinAbs;

    constexpr double kPi = std::numbers::pi;

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
    Vec3d w = Vec3d::Zero();
    Mat3d R = SO3::Exp_R(w);
    CHECK(R.isApprox(Mat3d::Identity(), 1e-12));
}

TEST_CASE("SO3::Exp_R: 90-degree rotation around z", "[SO3][Exp_R]")
{
    Vec3d w(0.0, 0.0, kPi / 2.0);
    Mat3d R = SO3::Exp_R(w);

    Mat3d expected;
    expected << 0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0;

    CHECK(R.isApprox(expected, 1e-12));
}

TEST_CASE("SO3::Exp_R: 180-degree rotation around x", "[SO3][Exp_R]")
{
    Vec3d w(kPi, 0.0, 0.0);
    Mat3d R = SO3::Exp_R(w);

    Mat3d expected;
    expected << 1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0;

    CHECK(R.isApprox(expected, 1e-12));
}

TEST_CASE("SO3::Exp_R: output is a valid rotation matrix for various angles", "[SO3][Exp_R]")
{
    const std::vector<Vec3d> cases = {
        Vec3d(0.3, 0.0, 0.0),
        Vec3d(0.0, 0.5, 0.0),
        Vec3d(0.0, 0.0, 0.7),
        Vec3d(0.1, 0.2, 0.3),
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
    Vec3d w(1e-7, 0.0, 0.0);
    Mat3d R = SO3::Exp_R(w);
    CHECK(R.isApprox(Mat3d::Identity(), 1e-6));
}

TEST_CASE("SO3::Exp_R: composition property Exp(w1)*Exp(w2) is SO(3)", "[SO3][Exp_R]")
{
    Vec3d w1(0.4, 0.0, 0.0);
    Vec3d w2(0.0, 0.3, 0.0);
    Mat3d R1 = SO3::Exp_R(w1);
    Mat3d R2 = SO3::Exp_R(w2);
    check_rotation_matrix(R1 * R2);
}

TEST_CASE("SO3::Exp_R: inverse is transpose (Exp(-w) = Exp(w)^T)", "[SO3][Exp_R]")
{
    Vec3d w(0.2, -0.3, 0.5);
    Vec3d wn(-0.2, 0.3, -0.5);
    Mat3d R = SO3::Exp_R(w);
    Mat3d Ri = SO3::Exp_R(wn);
    CHECK(R.isApprox(Ri.transpose(), 1e-12));
}

// =============================================================================
// SO3::LeftJacobian
// =============================================================================

TEST_CASE("SO3::LeftJacobian: zero vector gives identity", "[SO3][LeftJacobian]")
{
    Vec3d w = Vec3d::Zero();
    Mat3d J = SO3::LeftJacobian(w);
    CHECK(J.isApprox(Mat3d::Identity(), 1e-12));
}

TEST_CASE("SO3::LeftJacobian: small angle is approximately identity", "[SO3][LeftJacobian]")
{
    Vec3d w(1e-7, 0.0, 0.0);
    Mat3d J = SO3::LeftJacobian(w);
    CHECK(J.isApprox(Mat3d::Identity(), 1e-5));
}

TEST_CASE("SO3::LeftJacobian: non-singular for general rotation vector", "[SO3][LeftJacobian]")
{
    const std::vector<Vec3d> cases = {
        Vec3d(0.3, 0.0, 0.0),
        Vec3d(0.1, 0.2, 0.3),
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
    Vec3d w(0.2, -0.3, 0.5);
    Vec3d wn(-0.2, 0.3, -0.5);
    Mat3d J_pos = SO3::LeftJacobian(w);
    Mat3d J_neg = SO3::LeftJacobian(wn);
    CHECK(J_neg.isApprox(J_pos.transpose(), 1e-12));
}

// =============================================================================
// SO3::Log_R
// =============================================================================

namespace {
    const std::vector<Vec3d> kAxes = {
        Vec3d(1, 0, 0), Vec3d(0, 1, 0), Vec3d(0, 0, 1),
        Vec3d(1, 2, 3).normalized(), Vec3d(-1, 0.3, 0.2).normalized(), Vec3d(0.1, -0.9, 0.4).normalized()
    };
}

TEST_CASE("SO3::Log_R: inverts Exp_R for generic angles", "[SO3][Log_R]")
{
    for (const Vec3d& a : kAxes)
        for (double theta : { 1e-3, 0.1, 0.5, 1.0, 2.0, 3.0 })
        {
            const Vec3d w = theta * a;
            const Vec3d v = SO3::Log_R(SO3::Exp_R(w));
            INFO("theta = " << theta << ", axis = " << a.transpose());
            CHECK((v - w).norm() < 1e-9);
        }
}

TEST_CASE("SO3::Log_R: small angles through the series", "[SO3][Log_R]")
{
    for (const Vec3d& a : kAxes)
        for (double theta : { 1e-5, 1e-6, 1e-7 })
        {
            const Vec3d w = theta * a;
            const Vec3d v = SO3::Log_R(SO3::Exp_R(w));
            CHECK((v - w).norm() < 1e-12);
        }
    const Mat3d I = Mat3d::Identity();
    CHECK(SO3::Log_R(I).norm() == 0.0);
}

TEST_CASE("SO3::Log_R: rotations near a half turn are not collapsed", "[SO3][Log_R][regression]")
{
    // The small-angle switch is on sin²theta, which is also small at theta = pi.
    // Log_R(Exp_R((pi - 1e-5) a)) used to return a vector of norm 1e-5.
    for (const Vec3d& a : kAxes)
    {
        for (double theta : { kPi - 1e-3, kPi - 1e-5, kPi - 1e-7, kPi - 1e-9 })
        {
            const Vec3d w = theta * a;
            const Mat3d R = SO3::Exp_R(w);
            const Vec3d v = SO3::Log_R(R);
            INFO("theta = pi - " << kPi - theta << ", axis = " << a.transpose());
            CHECK_THAT(v.norm(), WithinAbs(theta, 1e-8));
            CHECK((v - w).norm() < 1e-7);               // same sign as well
            CHECK(SO3::Exp_R(v).isApprox(R, 1e-9));
        }
        // Exactly pi: +pi a and -pi a are the same rotation, so only the
        // angle and the rotation are checked.
        const Vec3d w_pi = kPi * a;
        const Mat3d R = SO3::Exp_R(w_pi);
        const Vec3d v = SO3::Log_R(R);
        CHECK_THAT(v.norm(), WithinAbs(kPi, 1e-9));
        CHECK(SO3::Exp_R(v).isApprox(R, 1e-9));
    }
}

TEST_CASE("SO3::Log_R: AD tape of Log(Exp(w)) has the identity Jacobian", "[SO3][Log_R][AD]")
{
    using AD = CppAD::AD<double>;
    std::vector<AD> aw{ AD(0.1), AD(-0.2), AD(0.3) };
    CppAD::Independent(aw);
    const Eigen::Matrix<AD, 3, 1> w(aw[0], aw[1], aw[2]);
    const Eigen::Matrix<AD, 3, 1> v = SO3::Log_R(SO3::Exp_R(w));
    std::vector<AD> av{ v(0), v(1), v(2) };
    CppAD::ADFun<double> f(aw, av);

    const std::vector<double> x{ 0.1, -0.2, 0.3 };
    const auto y = f.Forward(0, x);
    for (int i = 0; i < 3; ++i) CHECK_THAT(y[i], WithinAbs(x[i], 1e-12));
    const auto J = f.Jacobian(x);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_THAT(J[3 * i + j], WithinAbs(i == j ? 1.0 : 0.0, 1e-9));
}

TEST_CASE("SO3::Log_R: accurate in the ordinary branch just short of a half turn", "[SO3][Log_R][regression]")
{
    // theta / sin(theta) grows like 1 / sin^2(theta) towards pi. With sin
    // taken as sin(acos(c)) an O(1e-13) error in acos(c) became an O(1e-9)
    // error in the result at pi - 1e-3 (a Linux/GCC CI failure of the half-
    // turn test). With sin taken from R itself the result is as accurate as
    // the angle: a sweep over axes and angles in the ordinary branch
    // (sin^2 theta above 1e-8) must invert Exp_R to 1e-11.
    int n = 0;
    for (int i = 0; i < 40; ++i)
    {
        const double u = 0.37 * i, v = 0.91 * i + 0.3;
        const Vec3d a = Vec3d(std::cos(u) * std::sin(v), std::sin(u) * std::sin(v), std::cos(v)).normalized();
        for (double gap : { 1e-2, 3e-3, 1e-3, 3e-4, 1.2e-4 })
        {
            const Vec3d w = (kPi - gap) * a;
            const Mat3d R = SO3::Exp_R(w);
            const Vec3d r = SO3::Log_R(R);
            INFO("gap " << gap << ", axis " << a.transpose());
            CHECK((r - w).norm() < 1e-11);
            CHECK((SO3::Exp_R(r) - R).norm() < 1e-11);
            ++n;
        }
    }
    CHECK(n == 200);
}
