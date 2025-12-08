# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_coordinate
                    test_coordinate.cpp  
                    test_QuaterniontoNED.cpp)
                 #   test_inertial_to_local.cpp)

target_link_libraries(test_coordinate
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_coordinate PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME coordinate_tests COMMAND test_coordinate)