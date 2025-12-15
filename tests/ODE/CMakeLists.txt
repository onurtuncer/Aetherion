# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_scalar
                    test_Scalar.cpp)

target_compile_options(test_scalar PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_scalar
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_scalar PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME scalar_tests COMMAND test_scalar)

add_executable(test_concepts
                      test_Concepts.cpp)

target_compile_options(test_concepts PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_concepts
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_concepts PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME concepts_tests COMMAND test_concepts)

add_executable(test_se3
                      test_SE3.cpp)

target_compile_options(test_se3  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_se3
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_se3 PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME se3_tests COMMAND test_se3)

add_executable(test_newton
                      test_Newton.cpp)

target_compile_options(test_newton  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_newton
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_newton PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME newton_tests COMMAND test_newton)

