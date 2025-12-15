// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Dense>

TEST_CASE("Eigen basic matrix operations work", "[eigen]") {

    Eigen::Matrix2d A;
    A << 1.0, 2.0,
        3.0, 4.0;

    Eigen::Matrix2d B;
    B << 5.0, 6.0,
        7.0, 8.0;

    Eigen::Matrix2d C = A * B;

    // Expected result:
    // [ 1*5 + 2*7 , 1*6 + 2*8 ] = [ 19 , 22 ]
    // [ 3*5 + 4*7 , 3*6 + 4*8 ] = [ 43 , 50 ]

    REQUIRE(C(0, 0) == 19.0);
    REQUIRE(C(0, 1) == 22.0);
    REQUIRE(C(1, 0) == 43.0);
    REQUIRE(C(1, 1) == 50.0);
}
