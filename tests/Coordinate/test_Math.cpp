// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <numbers>

#include <Aetherion/Coordinate/Math.h>

using Catch::Approx;

namespace AC = Aetherion::Coordinate;
namespace detail = Aetherion::Coordinate::detail;

template <class S>
static S Norm2(const AC::Vec3<S>& v) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

template <class S>
static S Norm(const AC::Vec3<S>& v) {
    return detail::SquareRoot(Norm2(v));
}

template <class S>
static AC::Vec3<S> Mat3MulVec3(const AC::Mat3<S>& R, const AC::Vec3<S>& v) {
    return AC::Vec3<S>{
        R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
            R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2]
    };
}

template <class S>
static AC::Mat3<S> Transpose(const AC::Mat3<S>& R) {
    AC::Mat3<S> Rt{};
    Rt[0] = R[0]; Rt[1] = R[3]; Rt[2] = R[6];
    Rt[3] = R[1]; Rt[4] = R[4]; Rt[5] = R[7];
    Rt[6] = R[2]; Rt[7] = R[5]; Rt[8] = R[8];
    return Rt;
}

template <class S>
static AC::Mat3<S> MatMul(const AC::Mat3<S>& A, const AC::Mat3<S>& B) {
    AC::Mat3<S> C{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            S sum = S(0);
            for (int k = 0; k < 3; ++k) {
                sum += A[3 * r + k] * B[3 * k + c];
            }
            C[3 * r + c] = sum;
        }
    }
    return C;
}

template <class S>
static S Det(const AC::Mat3<S>& R) {
    // row-major
    const S a = R[0], b = R[1], c = R[2];
    const S d = R[3], e = R[4], f = R[5];
    const S g = R[6], h = R[7], i = R[8];
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

template <class S>
static AC::Quat<S> NormalizeQuat(const AC::Quat<S>& q) {
    const S n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    const S invn = S(1) / detail::SquareRoot(n2);
    return AC::Quat<S>{ q[0] * invn, q[1] * invn, q[2] * invn, q[3] * invn };
}

template <class S>
static S QuatAbsDot(const AC::Quat<S>& a, const AC::Quat<S>& b) {
    // account for q and -q equivalence
    const S d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    return d < S(0) ? -d : d;
}

TEST_CASE("Math.detail: Dot basics", "[math]") {
    using S = double;

    const AC::Vec3<S> a{ 1.0, 2.0, 3.0 };
    const AC::Vec3<S> b{ 4.0, -5.0, 6.0 };

    REQUIRE(detail::Dot(a, b) == Approx(1 * 4 + 2 * (-5) + 3 * 6));
    REQUIRE(detail::Dot(a, b) == Approx(detail::Dot(b, a)));

    const AC::Vec3<S> c{ 7.0, 8.0, 9.0 };
    const S lhs = detail::Dot(a, AC::Vec3<S>{b[0] + c[0], b[1] + c[1], b[2] + c[2]});
    const S rhs = detail::Dot(a, b) + detail::Dot(a, c);
    REQUIRE(lhs == Approx(rhs));
}

TEST_CASE("Math.detail: NED <-> NEU conversions", "[math]") {
    using S = double;

    const AC::Vec3<S> v_ned{ 10.0, -2.0,  3.5 };
    const auto v_neu = detail::NEDToNEU(v_ned);
    REQUIRE(v_neu[0] == Approx(v_ned[0]));
    REQUIRE(v_neu[1] == Approx(v_ned[1]));
    REQUIRE(v_neu[2] == Approx(-v_ned[2]));

    const auto v_ned2 = detail::NEUToNED(v_neu);
    REQUIRE(v_ned2[0] == Approx(v_ned[0]));
    REQUIRE(v_ned2[1] == Approx(v_ned[1]));
    REQUIRE(v_ned2[2] == Approx(v_ned[2]));
}

TEST_CASE("Math.detail: elementary wrappers match std", "[math]") {
    using S = double;

    const S x = 0.37;
    const S y = -0.91;

    REQUIRE(detail::Sine(x) == Approx(std::sin(x)));
    REQUIRE(detail::Cosine(x) == Approx(std::cos(x)));
    REQUIRE(detail::ArcTangent2(y, x) == Approx(std::atan2(y, x)));
    REQUIRE(detail::ArcCos(0.25) == Approx(std::acos(0.25)));
    REQUIRE(detail::SquareRoot(2.0) == Approx(std::sqrt(2.0)));
}

TEST_CASE("Math.detail: Cross product properties", "[math]") {
    using S = double;

    const AC::Vec3<S> ex{ 1.0, 0.0, 0.0 };
    const AC::Vec3<S> ey{ 0.0, 1.0, 0.0 };
    const AC::Vec3<S> ez{ 0.0, 0.0, 1.0 };

    const auto ex_x_ey = detail::Cross(ex, ey);
    REQUIRE(ex_x_ey[0] == Approx(ez[0]));
    REQUIRE(ex_x_ey[1] == Approx(ez[1]));
    REQUIRE(ex_x_ey[2] == Approx(ez[2]));

    const auto ey_x_ex = detail::Cross(ey, ex);
    REQUIRE(ey_x_ex[0] == Approx(-ex_x_ey[0]));
    REQUIRE(ey_x_ex[1] == Approx(-ex_x_ey[1]));
    REQUIRE(ey_x_ex[2] == Approx(-ex_x_ey[2]));

    const AC::Vec3<S> a{ 1.5, -2.0, 0.7 };
    const AC::Vec3<S> b{ -0.4, 3.1, 2.2 };
    const auto axb = detail::Cross(a, b);

    REQUIRE(detail::Dot(a, axb) == Approx(0.0).margin(1e-12));
    REQUIRE(detail::Dot(b, axb) == Approx(0.0).margin(1e-12));
}

TEST_CASE("Math.detail: Normalize returns unit vector", "[math]") {
    using S = double;

    const AC::Vec3<S> v{ 3.0, 4.0, 12.0 };
    const auto u = detail::Normalize(v);

    REQUIRE(Norm(u) == Approx(1.0).margin(1e-12));

    // direction preserved (up to scale)
    const int idx = 2; // largest component in v
    const S scale = u[idx] / v[idx];
    REQUIRE(u[0] == Approx(scale * v[0]).margin(1e-12));
    REQUIRE(u[1] == Approx(scale * v[1]).margin(1e-12));
    REQUIRE(u[2] == Approx(scale * v[2]).margin(1e-12));
}

TEST_CASE("Math.detail: Quaternion <-> rotation matrix checks", "[math]") {
    using S = double;

    // Build a nontrivial unit quaternion via axis-angle
    const S angle = 0.7; // rad
    const AC::Vec3<S> axis = detail::Normalize(AC::Vec3<S>{0.2, -0.9, 0.4});
    const S half = angle * S(0.5);
    const S s = std::sin(half);

    const AC::Quat<S> q = NormalizeQuat(AC::Quat<S>{
        std::cos(half),
            axis[0] * s,
            axis[1] * s,
            axis[2] * s
    });

    const auto R = detail::QuaternionToRotationMatrix(q);

    SECTION("R is orthonormal: R^T R ~= I") {
        const auto Rt = Transpose(R);
        const auto I = MatMul(Rt, R);

        REQUIRE(I[0] == Approx(1.0).margin(1e-12));
        REQUIRE(I[4] == Approx(1.0).margin(1e-12));
        REQUIRE(I[8] == Approx(1.0).margin(1e-12));

        REQUIRE(I[1] == Approx(0.0).margin(1e-12));
        REQUIRE(I[2] == Approx(0.0).margin(1e-12));
        REQUIRE(I[3] == Approx(0.0).margin(1e-12));
        REQUIRE(I[5] == Approx(0.0).margin(1e-12));
        REQUIRE(I[6] == Approx(0.0).margin(1e-12));
        REQUIRE(I[7] == Approx(0.0).margin(1e-12));
    }

    SECTION("det(R) ~= +1") {
        REQUIRE(Det(R) == Approx(1.0).margin(1e-12));
    }

    SECTION("Round-trip q -> R -> q' (up to sign)") {
        // RotationMatrixToQuaternion expects columns as Vec3
        const AC::Vec3<S> x_b{ R[0], R[3], R[6] }; // col 0
        const AC::Vec3<S> y_b{ R[1], R[4], R[7] }; // col 1
        const AC::Vec3<S> z_b{ R[2], R[5], R[8] }; // col 2

        const AC::Quat<S> q2 = NormalizeQuat(detail::RotationMatrixToQuaternion(x_b, y_b, z_b));

        // Compare via |dot| close to 1 (since q and -q represent same rotation)
        REQUIRE(QuatAbsDot(q, q2) == Approx(1.0).margin(1e-12));
    }

    SECTION("Rotation preserves vector norm") {
        const AC::Vec3<S> v{ 1.2, -0.7, 3.4 };
        const auto v2 = Mat3MulVec3(R, v);

        REQUIRE(Norm(v2) == Approx(Norm(v)).margin(1e-12));
    }
}

TEST_CASE("Math.detail: QuaternionToRotationMatrix known rotations", "[math]") {
    using S = double;

    auto RequireMat = [](const AC::Mat3<S>& A, const AC::Mat3<S>& B, double tol = 1e-12) {
        for (int i = 0; i < 9; ++i) {
            REQUIRE(A[i] == Approx(B[i]).margin(tol));
        }
        };

    SECTION("Identity quaternion -> Identity matrix") {
        const AC::Quat<S> qI{ 1.0, 0.0, 0.0, 0.0 };
        const auto R = detail::QuaternionToRotationMatrix(qI);

        const AC::Mat3<S> I{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        RequireMat(R, I);

        // basis mapping sanity
        const AC::Vec3<S> ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };
        REQUIRE(Mat3MulVec3(R, ex)[0] == Approx(1.0));
        REQUIRE(Mat3MulVec3(R, ey)[1] == Approx(1.0));
        REQUIRE(Mat3MulVec3(R, ez)[2] == Approx(1.0));
    }

    SECTION("90 deg about +Z: x->y, y->-x, z->z") {
        const S c = std::cos(std::numbers::pi_v<S> / S(4));
        const S s = std::sin(std::numbers::pi_v<S> / S(4));
        const AC::Quat<S> q{ c, 0.0, 0.0, s }; // [w,x,y,z]

        const auto R = detail::QuaternionToRotationMatrix(q);

        const AC::Mat3<S> Rz90{
            0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0
        };
        RequireMat(R, Rz90);

        const AC::Vec3<S> ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };

        const auto ex2 = Mat3MulVec3(R, ex);
        REQUIRE(ex2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ex2[1] == Approx(1.0).margin(1e-12));
        REQUIRE(ex2[2] == Approx(0.0).margin(1e-12));

        const auto ey2 = Mat3MulVec3(R, ey);
        REQUIRE(ey2[0] == Approx(-1.0).margin(1e-12));
        REQUIRE(ey2[1] == Approx(0.0).margin(1e-12));
        REQUIRE(ey2[2] == Approx(0.0).margin(1e-12));

        const auto ez2 = Mat3MulVec3(R, ez);
        REQUIRE(ez2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ez2[1] == Approx(0.0).margin(1e-12));
        REQUIRE(ez2[2] == Approx(1.0).margin(1e-12));
    }

    SECTION("180 deg about +X: y->-y, z->-z") {
        const AC::Quat<S> q{ 0.0, 1.0, 0.0, 0.0 }; // unit

        const auto R = detail::QuaternionToRotationMatrix(q);

        const AC::Mat3<S> Rx180{
            1.0,  0.0,  0.0,
            0.0, -1.0,  0.0,
            0.0,  0.0, -1.0
        };
        RequireMat(R, Rx180);

        const AC::Vec3<S> ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };
        const auto ey2 = Mat3MulVec3(R, ey);
        const auto ez2 = Mat3MulVec3(R, ez);

        REQUIRE(ey2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ey2[1] == Approx(-1.0).margin(1e-12));
        REQUIRE(ey2[2] == Approx(0.0).margin(1e-12));

        REQUIRE(ez2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ez2[1] == Approx(0.0).margin(1e-12));
        REQUIRE(ez2[2] == Approx(-1.0).margin(1e-12));
    }

    SECTION("90 deg about +X: y->z, z->-y") {
        const S c = std::cos(std::numbers::pi_v<S> / S(4));
        const S s = std::sin(std::numbers::pi_v<S> / S(4));
        const AC::Quat<S> q{ c, s, 0.0, 0.0 }; // [w,x,y,z]

        const auto R = detail::QuaternionToRotationMatrix(q);

        const AC::Mat3<S> Rx90{
            1.0,  0.0,  0.0,
            0.0,  0.0, -1.0,
            0.0,  1.0,  0.0
        };
        RequireMat(R, Rx90);

        const AC::Vec3<S> ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };

        const auto ey2 = Mat3MulVec3(R, ey);
        REQUIRE(ey2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ey2[1] == Approx(0.0).margin(1e-12));
        REQUIRE(ey2[2] == Approx(1.0).margin(1e-12));

        const auto ez2 = Mat3MulVec3(R, ez);
        REQUIRE(ez2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ez2[1] == Approx(-1.0).margin(1e-12));
        REQUIRE(ez2[2] == Approx(0.0).margin(1e-12));
    }

    SECTION("90 deg about +Y: x->-z, z->x") {
        const S c = std::cos(std::numbers::pi_v<S> / S(4));
        const S s = std::sin(std::numbers::pi_v<S> / S(4));
        const AC::Quat<S> q{ c, 0.0, s, 0.0 }; // [w,x,y,z]

        const auto R = detail::QuaternionToRotationMatrix(q);

        const AC::Mat3<S> Ry90{
            0.0,  0.0,  1.0,
            0.0,  1.0,  0.0,
           -1.0,  0.0,  0.0
        };
        RequireMat(R, Ry90);

        const AC::Vec3<S> ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };

        const auto ex2 = Mat3MulVec3(R, ex);
        REQUIRE(ex2[0] == Approx(0.0).margin(1e-12));
        REQUIRE(ex2[1] == Approx(0.0).margin(1e-12));
        REQUIRE(ex2[2] == Approx(-1.0).margin(1e-12));

        const auto ez2 = Mat3MulVec3(R, ez);
        REQUIRE(ez2[0] == Approx(1.0).margin(1e-12));
        REQUIRE(ez2[1] == Approx(0.0).margin(1e-12));
        REQUIRE(ez2[2] == Approx(0.0).margin(1e-12));
    }

}

TEST_CASE("Math.detail: RotationMatrixToQuaternion known cases (up to sign)", "[math]") {
    using S = double;

    SECTION("Identity axes -> identity quaternion") {
        const AC::Vec3<S> x{ 1,0,0 };
        const AC::Vec3<S> y{ 0,1,0 };
        const AC::Vec3<S> z{ 0,0,1 };

        const auto q = NormalizeQuat(detail::RotationMatrixToQuaternion(x, y, z));
        const AC::Quat<S> qI{ 1,0,0,0 };

        REQUIRE(QuatAbsDot(q, qI) == Approx(1.0).margin(1e-12));
    }

    SECTION("Rz(90deg) -> quaternion (cos(pi/4), 0,0,sin(pi/4)) up to sign") {
        // Rz90 = [[0,-1,0],[1,0,0],[0,0,1]] in row-major
        const AC::Mat3<S> Rz90{
            0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0
        };

        // columns as Vec3 (as required by RotationMatrixToQuaternion)
        const AC::Vec3<S> x_b{ Rz90[0], Rz90[3], Rz90[6] };
        const AC::Vec3<S> y_b{ Rz90[1], Rz90[4], Rz90[7] };
        const AC::Vec3<S> z_b{ Rz90[2], Rz90[5], Rz90[8] };

        const auto q = NormalizeQuat(detail::RotationMatrixToQuaternion(x_b, y_b, z_b));

        const S c = std::cos(std::numbers::pi_v<S> / S(4));
        const S s = std::sin(std::numbers::pi_v<S> / S(4));
        const AC::Quat<S> q_expected{ c, 0.0, 0.0, s };

        REQUIRE(QuatAbsDot(q, q_expected) == Approx(1.0).margin(1e-12));

        SECTION("Rx(90deg) -> quaternion (cos(pi/4), sin(pi/4),0,0) up to sign") {
            const AC::Mat3<S> Rx90{
                1.0,  0.0,  0.0,
                0.0,  0.0, -1.0,
                0.0,  1.0,  0.0
            };

            const AC::Vec3<S> x_b{ Rx90[0], Rx90[3], Rx90[6] };
            const AC::Vec3<S> y_b{ Rx90[1], Rx90[4], Rx90[7] };
            const AC::Vec3<S> z_b{ Rx90[2], Rx90[5], Rx90[8] };

            const auto q = NormalizeQuat(detail::RotationMatrixToQuaternion(x_b, y_b, z_b));

            const S c = std::cos(std::numbers::pi_v<S> / S(4));
            const S s = std::sin(std::numbers::pi_v<S> / S(4));
            const AC::Quat<S> q_expected{ c, s, 0.0, 0.0 };

            REQUIRE(QuatAbsDot(q, q_expected) == Approx(1.0).margin(1e-12));
        }

        SECTION("Ry(90deg) -> quaternion (cos(pi/4), 0,sin(pi/4),0) up to sign") {
            const AC::Mat3<S> Ry90{
                0.0,  0.0,  1.0,
                0.0,  1.0,  0.0,
               -1.0,  0.0,  0.0
            };

            const AC::Vec3<S> x_b{ Ry90[0], Ry90[3], Ry90[6] };
            const AC::Vec3<S> y_b{ Ry90[1], Ry90[4], Ry90[7] };
            const AC::Vec3<S> z_b{ Ry90[2], Ry90[5], Ry90[8] };

            const auto q = NormalizeQuat(detail::RotationMatrixToQuaternion(x_b, y_b, z_b));

            const S c = std::cos(std::numbers::pi_v<S> / S(4));
            const S s = std::sin(std::numbers::pi_v<S> / S(4));
            const AC::Quat<S> q_expected{ c, 0.0, s, 0.0 };

            REQUIRE(QuatAbsDot(q, q_expected) == Approx(1.0).margin(1e-12));
        }

    }

    SECTION("Rx(180deg) -> quaternion (0,1,0,0) up to sign") {
        const AC::Mat3<S> Rx180{
            1.0,  0.0,  0.0,
            0.0, -1.0,  0.0,
            0.0,  0.0, -1.0
        };

        const AC::Vec3<S> x_b{ Rx180[0], Rx180[3], Rx180[6] };
        const AC::Vec3<S> y_b{ Rx180[1], Rx180[4], Rx180[7] };
        const AC::Vec3<S> z_b{ Rx180[2], Rx180[5], Rx180[8] };

        const auto q = NormalizeQuat(detail::RotationMatrixToQuaternion(x_b, y_b, z_b));
        const AC::Quat<S> q_expected{ 0.0, 1.0, 0.0, 0.0 };

        REQUIRE(QuatAbsDot(q, q_expected) == Approx(1.0).margin(1e-12));
    }
}
