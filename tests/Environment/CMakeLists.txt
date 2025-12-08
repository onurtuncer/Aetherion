# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_environment
   test_atmosphere.cpp
)

target_link_libraries(test_environment
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_environment PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME environment_tests COMMAND test_environment)
