# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
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

include(Catch)
catch_discover_tests(test_scalar)
                                               
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

include(Catch)
catch_discover_tests(test_concepts)
                                               
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

include(Catch)
catch_discover_tests(test_se3)
                                               
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

include(Catch)
catch_discover_tests(test_newton)
                                               

add_executable(test_stage_pack
                      test_StagePack.cpp)

target_compile_options(test_stage_pack  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_stage_pack
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_stage_pack PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_stage_pack)
                                               
add_executable(test_radauiia_se3
                      test_RadauIIA_SE3.cpp)

target_compile_options(test_radauiia_se3  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_radauiia_se3
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_radauiia_se3 PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_radauiia_se3)        

add_executable(test_so3
                    test_SO3.cpp)

target_compile_options(test_so3 PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_so3
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad
        Eigen3::Eigen
)

target_include_directories(test_so3 PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_so3)

add_subdirectory(RKMK)
