# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_atmosphere
                    test_atmosphere.cpp )

target_link_libraries(test_atmosphere
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_atmosphere PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME atmosphere_tests COMMAND test_atmosphere)

add_executable(test_gravity
                    test_gravity.cpp )

target_link_libraries(test_gravity
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_gravity PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME gravity_tests COMMAND test_gravity)
