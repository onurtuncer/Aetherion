// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Catch2 tests for: Aetherion/ODE/RKMK/Core/StagePack.h
//
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <Aetherion/ODE/RKMK/Core/StagePack.h>

namespace {
    namespace Core = Aetherion::ODE::RKMK::Core;

    template<class Vec>
    void fill_vector_sequential(Vec& v) {
        for (int i = 0; i < v.size(); ++i) v(i) = static_cast<typename Vec::Scalar>(i + 1);
    }

    template<class Mat>
    void fill_matrix_sequential(Mat& M) {
        int k = 1;
        for (int r = 0; r < M.rows(); ++r) {
            for (int c = 0; c < M.cols(); ++c) {
                M(r, c) = static_cast<typename Mat::Scalar>(k++);
            }
        }
    }
} // namespace

TEST_CASE("StagePack fixed EuclidDim: offsets and views are correct", "[stagepack]") {
    using Pack = Core::StagePack<double, 3, 2>; // s=3, m=2 => total = 3*(6+2)=24
    Pack pack;

    REQUIRE(pack.stages() == 3);
    REQUIRE(pack.lie_dim() == 6);
    REQUIRE(pack.euclid_dim() == 2);
    REQUIRE(pack.stage_dim() == 8);
    REQUIRE(pack.total_dim() == 24);

    auto x = pack.make_vector();
    fill_vector_sequential(x);

    // Stage 0 ranges
    REQUIRE(pack.stage_offset(0) == 0);
    REQUIRE(pack.xi_offset(0) == 0);
    REQUIRE(pack.u_offset(0) == 6);

    // Stage 1 ranges
    REQUIRE(pack.stage_offset(1) == 8);
    REQUIRE(pack.xi_offset(1) == 8);
    REQUIRE(pack.u_offset(1) == 14);

    // Check xi(1) equals x.segment(8,6)
    {
        const auto xi1 = pack.xi(x, 1);
        for (int k = 0; k < 6; ++k) {
            REQUIRE(xi1(k) == x(8 + k));
        }
    }

    // Check u(1) equals x.segment(14,2)
    {
        const auto u1 = pack.u(x, 1);
        for (int k = 0; k < 2; ++k) {
            REQUIRE(u1(k) == x(14 + k));
        }
    }

    // Write-through on xi and u modifies underlying x
    {
        auto xi2 = pack.xi(x, 2);
        xi2.setConstant(100.0);
        for (int k = 0; k < 6; ++k) {
            REQUIRE(x(pack.xi_offset(2) + k) == 100.0);
        }

        auto u0 = pack.u(x, 0);
        u0 << -1.0, -2.0;
        REQUIRE(x(pack.u_offset(0) + 0) == -1.0);
        REQUIRE(x(pack.u_offset(0) + 1) == -2.0);
    }

    // Stage view equals concatenation [xi; u]
    {
        const int i = 1;
        const auto st = pack.stage(x, i);
        const auto xi = pack.xi(x, i);
        const auto u = pack.u(x, i);

        REQUIRE(st.size() == pack.stage_dim());
        for (int k = 0; k < pack.lie_dim(); ++k) REQUIRE(st(k) == xi(k));
        for (int k = 0; k < pack.euclid_dim(); ++k) REQUIRE(st(pack.lie_dim() + k) == u(k));
    }
}

TEST_CASE("StagePack dynamic EuclidDim: offsets and views are correct", "[stagepack]") {
    using Pack = Core::StagePack<double, 2, Eigen::Dynamic>; // s=2, m runtime
    Pack pack(/*euclid_dim=*/4); // m=4 => total = 2*(6+4)=20

    REQUIRE(pack.stages() == 2);
    REQUIRE(pack.lie_dim() == 6);
    REQUIRE(pack.euclid_dim() == 4);
    REQUIRE(pack.stage_dim() == 10);
    REQUIRE(pack.total_dim() == 20);

    auto x = pack.make_vector();
    fill_vector_sequential(x);

    REQUIRE(pack.stage_offset(0) == 0);
    REQUIRE(pack.u_offset(0) == 6);
    REQUIRE(pack.stage_offset(1) == 10);
    REQUIRE(pack.u_offset(1) == 16);

    // u(1) == segment(16,4)
    {
        const auto u1 = pack.u(x, 1);
        for (int k = 0; k < 4; ++k) {
            REQUIRE(u1(k) == x(16 + k));
        }
    }
}

TEST_CASE("StagePack Jacobian block views map correctly", "[stagepack]") {
    using Pack = Core::StagePack<double, 3, 2>; // total=24
    Pack pack;

    auto J = pack.make_matrix();
    J.setZero();

    // Set a full stage block (i=1,j=2) and verify locations
    {
        auto blk = pack.stage_block(J, 1, 2);
        REQUIRE(blk.rows() == pack.stage_dim());
        REQUIRE(blk.cols() == pack.stage_dim());
        blk.setConstant(5.0);

        const int r0 = pack.stage_offset(1);
        const int c0 = pack.stage_offset(2);
        for (int r = 0; r < pack.stage_dim(); ++r) {
            for (int c = 0; c < pack.stage_dim(); ++c) {
                REQUIRE(J(r0 + r, c0 + c) == 5.0);
            }
        }
    }

    // Now set sub-blocks inside (i=0,j=0) and verify exact indices
    {
        pack.J_xi_xi(J, 0, 0).setConstant(11.0);
        pack.J_xi_u(J, 0, 0).setConstant(12.0);
        pack.J_u_xi(J, 0, 0).setConstant(13.0);
        pack.J_u_u(J, 0, 0).setConstant(14.0);

        const int xo = pack.xi_offset(0); // 0
        const int uo = pack.u_offset(0);  // 6

        // xi-xi: [0..5]x[0..5]
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                REQUIRE(J(xo + r, xo + c) == 11.0);

        // xi-u: [0..5]x[6..7]
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 2; ++c)
                REQUIRE(J(xo + r, uo + c) == 12.0);

        // u-xi: [6..7]x[0..5]
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 6; ++c)
                REQUIRE(J(uo + r, xo + c) == 13.0);

        // u-u: [6..7]x[6..7]
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 2; ++c)
                REQUIRE(J(uo + r, uo + c) == 14.0);
    }
}
