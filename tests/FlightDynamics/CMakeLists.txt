# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_initial_state
                    test_BuildInitialStateVector.cpp)

target_compile_options(test_initial_state PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_initial_state
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad
        Eigen3::Eigen
)

target_include_directories(test_initial_state PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_initial_state)
