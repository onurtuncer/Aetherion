# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_simulation
                    test_Snapshot1.cpp
                    test_Config.cpp
                    test_Application.cpp
                    test_ISimulator.cpp)

target_link_libraries(test_simulation
    PRIVATE
        Aetherion::Simulation
     #   CppAD::cppad
        Catch2::Catch2WithMain
)

target_compile_options(test_simulation PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
# target_include_directories(test_simulation PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_simulation)